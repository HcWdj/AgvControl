#include <ros_handle.h>

using namespace std;
void handle_spin();
void MapCallback(const nav_msgs::OccupancyGridConstPtr& map);
ros::Publisher ros_cmd_vel_pub;
ros::Subscriber ros_map_sub;
uint8_t map_data_buff[MAP_SIZE_RANGE*MAP_SIZE_RANGE + sizeof(MapHead)];
float max_v_line=MAX_CMD_V_LINE;
float max_v_angle=MAX_CMD_V_ANGLE;
uint8_t agv_mode_state = AGV_INITAL_MODE;
SendRobotPose origin_pose={0.0,0.0};
float map_resolution=0.0;

void tf_listener();
bool HandleInit(ros::NodeHandle ros_nh)
{
    ros_cmd_vel_pub = ros_nh.advertise<geometry_msgs::Twist>("cmd_vel", 30);
    thread t2(tf_listener);
    t2.detach();
    return true;
}

void CmdSend(const int8_t id,const int8_t len,void *data_ptr)
{
    CmdMessage send_message;
    send_message.head.id = id;
    send_message.len = len;
    memcpy(send_message.data,data_ptr, len); // when send map , len is 0 , will pass
    send_message.data_ptr = data_ptr;
    Send(send_message);
}

void tf_listener()
{
    ros::Rate tf_listener_loop_rate(1);
    tf::TransformListener tf_pose_listener;
    while(1) {
        if(agv_mode_state==AGV_INITAL_MODE) {//req pose state == req map state
            // ROS_INFO("req_pose_state is false");
        }
        else {
            SendRobotPose *send_robot_pose = new SendRobotPose();
            tf::StampedTransform pose_transform;
            try {
                //tf_pose_listener.lookupTransform("/map", "/base_footprint",  // find tf
                //                                ros::Time(0), pose_transform);
                tf_pose_listener.lookupTransform("/map", ros::Time(0), "/base_footprint", // find tf
                                                 ros::Time(0), "/odom", pose_transform);
            }
            catch (tf::TransformException &ex) {
                //  ROS_ERROR("%s", ex.what());
                //ros::Duration(1.0).sleep();
                continue;
            }

            send_robot_pose->pose_x_m = (pose_transform.getOrigin().x() - origin_pose.pose_x_m);
            send_robot_pose->pose_y_m = (pose_transform.getOrigin().y() - origin_pose.pose_y_m);
            send_robot_pose->pose_angle = tf::getYaw(pose_transform.getRotation());
            ROS_INFO("Received robot pose from map to base_footprint --- x:%f  y:%f",
                     pose_transform.getOrigin().x(),
                     pose_transform.getOrigin().y());
            ROS_INFO("origin_pose.pose_y_m=%f",origin_pose.pose_y_m);
            CmdSend(CMD_SEND_ROBOT_POSE, sizeof(SendRobotPose), send_robot_pose);
            ROS_INFO("_________now location x=%f,y=%f",send_robot_pose->pose_x_m/0.05,send_robot_pose->pose_y_m/0.05);

        }
        tf_listener_loop_rate.sleep();
    }
}

void MapCallback(const nav_msgs::OccupancyGridConstPtr& map) {
    if(agv_mode_state==AGV_INITAL_MODE) {//req map state
        ROS_INFO("req map state is false");
        return;
    }
    if(agv_mode_state==AGV_SLAM_MODE) {
        ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                 map->info.width,
                 map->info.height,
                 map->info.resolution);
        origin_pose.pose_x_m = map->info.origin.position.x;
        origin_pose.pose_y_m = map->info.origin.position.y;
        origin_pose.pose_angle = tf::getYaw(map->info.origin.orientation);
        map_resolution = map->info.resolution;
        ROS_INFO("Received origin pose --- x:%f  y:%f",
                 origin_pose.pose_x_m,
                 origin_pose.pose_y_m);
        if (map->info.width < MAP_SIZE_RANGE && map->info.height < MAP_SIZE_RANGE) {
            MapMessage *send_map_message = new MapMessage();
            send_map_message->map_head.width.at = map->info.width;
            send_map_message->map_head.height.at = map->info.height;
            send_map_message->map_head.origin_pose.pose_x_m = -map->info.origin.position.x / map_resolution;
            send_map_message->map_head.origin_pose.pose_y_m = -map->info.origin.position.y / map_resolution;
            send_map_message->map_head.origin_pose.pose_angle = tf::getYaw(map->info.origin.orientation);
            memcpy(map_data_buff, send_map_message, sizeof(MapHead));
            int temp = 0;
            for (unsigned int y = 0; y < map->info.height; y++) {
                for (unsigned int x = 0; x < map->info.width; x++) {
                    unsigned int i = x + (map->info.height - y - 1) * map->info.width;
                    map_data_buff[sizeof(MapHead) + temp] = map->data[i];
                    temp++;
                }
            }
            send_map_message->data_ptr = map_data_buff;
            CmdSend(CMD_SEND_MAP, 0, send_map_message->data_ptr);
        } else
            ROS_ERROR("map is out of size!");
    }
    if(agv_mode_state==AGV_NAVIGATION_MODE){
        origin_pose.pose_x_m = map->info.origin.position.x;
        origin_pose.pose_y_m = map->info.origin.position.y;
        origin_pose.pose_angle = tf::getYaw(map->info.origin.orientation);
        map_resolution = map->info.resolution;
    }
}
void ProcessModeCtl(int8_t mode)
{
    static int8_t mode_save=AGV_INITAL_MODE;
    if((mode==AGV_INITAL_MODE)||(mode==mode_save)) {
        return;
    }
    if(mode==AGV_SLAM_MODE) {
        ROS_INFO("start slam success!!!!!!!!!!!");
        // execl("/home/ray/start_test.sh","1",NULL);
//        system("gnome-terminal -x bash /home/ray/ROS/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation_lx_autorun.sh 1;");
        system("gnome-terminal -x bash /home/wdjie/Downloads/run_slam.sh");

    }
    else if(mode==AGV_NAVIGATION_MODE){
        ROS_INFO("start navigation success!!!!!!!!!!!");
//        system("gnome-terminal -x bash /home/ray/ROS/turtlebot3_ws/src/turtlebot3/turtlebot3_navigation/launch/turtlebot3_navigation_lx_autorun.sh 2;");
        // execl("/home/ray/start_test.sh","2",NULL);
    }
    mode_save=mode;
}

void CmdCtlSetMode(const SetMode *value){
    ros::NodeHandle nh_map;
    ros_map_sub = nh_map.subscribe("/map",1,&MapCallback);
    agv_mode_state=value->value;
    ROS_INFO("Receive req_mode_state: %d",agv_mode_state );
    ProcessModeCtl(agv_mode_state);//start process with diffrent mode
}

void CmdCtlSetAgvSpeed(const SetAgvSpeed *value){

    geometry_msgs::Twist cmd_vel_;
    cmd_vel_.linear.x = (value->v_line * max_v_line) / 128.0 ;
    cmd_vel_.angular.z = -(value->v_angle * max_v_angle) /128.0 ;
    cmd_vel_.linear.y=0.0;
    cmd_vel_.linear.z=0.0;
    cmd_vel_.angular.x=0.0;
    cmd_vel_.angular.y=0.0;
    ros_cmd_vel_pub.publish(cmd_vel_);

    static int time=0;
    time++;
    if(time==50) {
        time=0;
        ROS_INFO("Receive cmd_vel_.linear.x: %f  cmd_vel_.angular.z: %f",
                 cmd_vel_.linear.x, cmd_vel_.angular.z);
    }
}
void CmdCtlSetSpeedRange(const SetSpeedRange *value){
    ROS_INFO("1.Receive max_v_line: %f  max_v_angle: %f",value->max_v_line,value->max_v_angle );
    if((value->max_v_line>2*MAX_CMD_V_LINE)||(value->max_v_angle>2*MAX_CMD_V_ANGLE)){
        ROS_INFO("Set Speed Range is error!");
        return;
    }
    max_v_line=value->max_v_line;
    max_v_angle=value->max_v_angle;
    ROS_INFO("2.Receive max_v_line: %f  max_v_angle: %f",max_v_line,max_v_angle);
}

void CmdCtlSetGoalPose(const SetGoalPose *value){
    if(agv_mode_state!=AGV_NAVIGATION_MODE) {//agv_mode_state
        //ROS_INFO("agv mode state is false");
        return;
    }
    static actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> client("move_base", true);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    // set frame
    goal.target_pose.header.frame_id = "map";
    // set position
    goal.target_pose.pose.position.x = value->pose_x_m * map_resolution + origin_pose.pose_x_m;
    goal.target_pose.pose.position.y = value->pose_y_m * map_resolution + origin_pose.pose_y_m;
    goal.target_pose.pose.position.z = 0;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(value->pose_angle*PI/180);
    goal.target_pose.pose.orientation = q;
    client.sendGoal(goal);
    ROS_INFO("Received goal pose --- x:%f  y:%f",
             goal.target_pose.pose.position.x ,
             goal.target_pose.pose.position.y);

}

void CmdCtlSetCmd(const SetCmd *value){
    switch (value->value)
    {
        case SET_CMD_SAVE_MAP:{//save map
            system("bash /home/ray/save_map_test.sh");
            ROS_INFO("save map success!");
        }break;
    }

}

void CmdProcess(const CmdMessage* recv)
{

    switch (recv->head.id)
    {
        case CMD_SET_MODE:{//set mode state
            if(recv->len!=sizeof(SetMode))break;
            SetMode *set_mode=new SetMode();
            memcpy(set_mode, recv->data, sizeof(SetMode));
            CmdCtlSetMode(set_mode);
        }break;
        case CMD_SET_AGV_SPEED: {//set agv robot control speed
            if(recv->len!=sizeof(SetAgvSpeed))break;
            SetAgvSpeed *set_agv_speed = new SetAgvSpeed();
            memcpy(set_agv_speed, recv->data, sizeof(SetAgvSpeed));
            CmdCtlSetAgvSpeed(set_agv_speed);
        }break;
        case CMD_SET_SPEED_RANGE:{//set agv robot control speed range
            if(recv->len!=sizeof(SetSpeedRange))break;
            SetSpeedRange *set_speed_range=new SetSpeedRange();
            memcpy(set_speed_range, recv->data, sizeof(SetSpeedRange));
            CmdCtlSetSpeedRange(set_speed_range);
        }break;
        case CMD_SET_GOAL_POSE:{//set agv robot goal pose
            if(recv->len!=sizeof(SetGoalPose))break;
            SetGoalPose *set_goal_pose=new SetGoalPose();
            memcpy(set_goal_pose, recv->data, sizeof(SetGoalPose));
            CmdCtlSetGoalPose(set_goal_pose);
        }break;
        case CMD_SET_CMD:{//set cmd
            if(recv->len!=sizeof(SetCmd))break;
            SetCmd *set_cmd=new SetCmd();
            memcpy(set_cmd, recv->data, sizeof(SetCmd));
            CmdCtlSetCmd(set_cmd);
        }break;
    }
}


void handle_spin()
{
    CmdMessage *recv_container=new CmdMessage();
    if(Take(recv_container))
    {
        CmdProcess(recv_container); //cmd process
    }
}

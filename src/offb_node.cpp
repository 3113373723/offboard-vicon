#include "offboard/offb_node.h"

OffbNode::OffbNode() :
    nh_()
{
    state_sub_ = nh_.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &OffbNode::state_cb, this);
    vicon_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/uav1/pose", 10, &OffbNode::vicon_pose_cb, this);

    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    vision_pub = nh_.advertise<geometry_msgs::PoseStamped>
            ("mavros/vision_pose/pose", 10);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    vision_timer = nh_.createTimer(ros::Duration(0.05),&OffbNode::vicon_timer_cb, this);
}

void OffbNode::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
    ROS_INFO("************");
}

void OffbNode::vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose_ = *msg;
}

void OffbNode::vicon_timer_cb(const ros::TimerEvent &event){
    current_pose_.header.stamp = ros::Time::now();
    vision_pub.publish(current_pose_);
}

void OffbNode::run(){
    ROS_INFO("Entering run*********************************************");
    ros::Rate rate(20.0);
    ROS_INFO("ROS OK%d",ros::ok());
    ROS_INFO("ROS state%d",current_state_.connected);
    while(ros::ok() && !current_state_.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("ROS OK%d",ros::ok());
        ROS_INFO("ROS state%d",current_state_.connected);
        ROS_INFO("###********************************************");
    }
    ROS_INFO("Entering run1*********************************************");
    // goal_.type_mask = 0b100111111000;
    // goal_.coordinate_frame = 1;
    goal_.pose.position.x = 0.0;
    goal_.pose.position.y = 0.0;
    goal_.pose.position.z = 1.0;
    // goal_.yaw = 0;

    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_.publish(goal_);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Entering run2*********************************************");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    int offboard_flag = 0;//防止切land降落之后又去解锁切offboard

    ros::Time last_request = ros::Time::now();
    ROS_INFO("Ready##################################################");

    while(ros::ok()){

        if( current_state_.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
            ROS_INFO("wait for offbard");
            // if( set_mode_client_.call(offb_set_mode) &&
            //     offb_set_mode.response.mode_sent){
            //     ROS_INFO("Offboard enabled");
            // }
            
            last_request = ros::Time::now();
        } else {
            if( !current_state_.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))&&(offboard_flag==0)){
                ROS_INFO("wait for arm");
                // if( arming_client_.call(arm_cmd) &&
                //     arm_cmd.response.success){
                //     ROS_INFO("Vehicle armed");
                // }
                last_request = ros::Time::now();//可以通过这个方式保持当前指点持续几秒钟。
            }
        }
    

        if (ros::Time::now() - last_request > ros::Duration(15.0)){
            mavros_msgs::SetMode land_set_mode;
            land_set_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client_.call(land_set_mode) && land_set_mode.response.mode_sent) {
                ROS_INFO("Landing enabled");
                offboard_flag = 1;//防止切land降落之后又去解锁切offboard
                break;
            }
        }

        local_pos_pub_.publish(goal_);

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");

    ROS_INFO("Entering main");
    OffbNode offb_node;
    offb_node.run();

    return 0;
}



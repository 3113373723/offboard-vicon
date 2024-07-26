#ifndef OFFB_NODE_H
#define OFFB_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "mavros_msgs/PositionTarget.h"

class OffbNode
{
public:
    OffbNode();
    void run();

private:
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void vicon_timer_cb(const ros::TimerEvent &event);

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber vicon_pose_sub_;
    ros::Publisher local_pos_pub_;
    ros::Publisher vision_pub;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    ros::Timer vision_timer;
    bool AUTO_ARM_OFFBOARD;

    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped goal_;
    // mavros_msgs::PositionTarget goal_;
};

#endif // OFFB_NODE_H
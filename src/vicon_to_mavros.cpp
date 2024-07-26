/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

// #include <Eigen/Eigen>
// #include <nav_msgs/Odometry.h>


// Eigen::Vector3d pos_drone_t265;
// Eigen::Quaterniond q_t265;

geometry_msgs::PoseStamped vision;

void vicon_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vision = *msg;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "t265_to_mavros");
    ros::NodeHandle nh;


        //  【订阅】估计位置
    ros::Subscriber vicon_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/uav1/pose", 100, vicon_cb);

    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);



    ros::Time last_request = ros::Time::now();

    while(ros::ok()){


        vision.header.stamp = ros::Time::now();
        vision_pub.publish(vision);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


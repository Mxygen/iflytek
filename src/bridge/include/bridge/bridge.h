#ifndef __BRIDGE_H__
#define __BRIDGE_H__
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <cmath>

using namespace std;

class Bridge {
public:
    Bridge(string odom_topic, string velocity_topic, string new_odom_topic, string new_velocity_topic)
    {
        // Initialize the ROS node handle
        ros::NodeHandle nh;

        // Subscribe to the original odometry and velocity topics
        odom_sub_ = nh.subscribe(odom_topic, 10, &Bridge::odomCallback, this);
        velocity_sub_ = nh.subscribe(velocity_topic, 15, &Bridge::VelocityCallback, this);

        // Advertise the new odometry and velocity topics
        newOdom_pub_ = nh.advertise<nav_msgs::Odometry>(new_odom_topic, 10);
        newVel_pub_ = nh.advertise<geometry_msgs::Twist>(new_velocity_topic, 10);
        
        // Initialize variables
        first_odom_ = true;
        first_cmd_ = true;
        integrated_yaw_ = 0.0;
        virtual_x_ = 0.0;
        virtual_y_ = 0.0;
    }
    virtual ~Bridge() = default;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
    void spin()
    {
        // Spin the ROS node to process callbacks
        ros::spin();
    }

private:
    // ROS related members
    ros::Publisher newOdom_pub_;
    ros::Publisher newVel_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber velocity_sub_;

    // Odometry integration related members
    ros::Time last_odom_time_;
    ros::Time last_cmd_time_;
    nav_msgs::Odometry last_odom_;  // 改为存储完整的 Odometry 消息
    geometry_msgs::Twist last_velocity_;  // 存储上一次的速度命令
    bool first_odom_;
    bool first_cmd_;
    double integrated_yaw_;  // 通过角速度积分得到的偏航角
    double virtual_x_;       // 虚拟位置x
    double virtual_y_;       // 虚拟位置y
    
    // Helper functions
    double quaternionToYaw(const geometry_msgs::Quaternion& q);
    geometry_msgs::Quaternion yawToQuaternion(double yaw);
    double EulerFromQuaternion(const geometry_msgs::Quaternion& q);
};

#endif
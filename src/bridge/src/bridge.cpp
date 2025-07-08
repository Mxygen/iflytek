#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "bridge/bridge.h"

// Helper function to convert quaternion to yaw angle
double Bridge::quaternionToYaw(const geometry_msgs::Quaternion& q) {
    // Convert quaternion to yaw angle using tf2
    tf2::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

// Helper function to convert yaw angle to quaternion
geometry_msgs::Quaternion Bridge::yawToQuaternion(double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    
    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = tf_quat.x();
    quat_msg.y = tf_quat.y();
    quat_msg.z = tf_quat.z();
    quat_msg.w = tf_quat.w();
    
    return quat_msg;
}

// Alternative helper function (keeping for compatibility)
double Bridge::EulerFromQuaternion(const geometry_msgs::Quaternion& q) {
    return quaternionToYaw(q);
}

void Bridge::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Handle first odometry message
    if (first_odom_) {
        last_odom_time_ = msg->header.stamp;
        last_odom_ = *msg;  // 存储完整的 Odometry 消息
        
        // Initialize integrated yaw with the current orientation
        integrated_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
        
        // Initialize virtual position with current position
        virtual_x_ = msg->pose.pose.position.x;
        virtual_y_ = msg->pose.pose.position.y;
        
        first_odom_ = false;
        
        ROS_INFO("First odometry received. Initial yaw: %f rad (%f deg), pos: (%f, %f)", 
                 integrated_yaw_, integrated_yaw_ * 180.0 / M_PI, virtual_x_, virtual_y_);
        return;
    }
    
    // Calculate time interval
    ros::Duration time_interval = msg->header.stamp - last_odom_time_;
    double dt = time_interval.toSec();
    
    if (dt <= 0) {
        ROS_WARN("Invalid time interval: %f seconds", dt);
        return;
    }
    
    // Use the angular velocity from the last velocity command to integrate yaw
    double angular_vel_z = last_velocity_.angular.z;  
    
    // Integrate angular velocity to get yaw
    integrated_yaw_ += angular_vel_z * dt;
    
    // Normalize yaw to [-pi, pi]
    while (integrated_yaw_ > M_PI) integrated_yaw_ -= 2.0 * M_PI;
    while (integrated_yaw_ < -M_PI) integrated_yaw_ += 2.0 * M_PI;
    
    // Create new odometry message with integrated orientation and original position/velocity
    nav_msgs::Odometry new_odom = *msg;
    
    // Update orientation with integrated yaw
    new_odom.pose.pose.orientation = yawToQuaternion(integrated_yaw_);
    
    // Keep the original linear position and velocity from sensor
    // but update the angular velocity in twist to match our integrated approach
    new_odom.twist.twist.angular.z = angular_vel_z;
    
    // Publish the new odometry
    newOdom_pub_.publish(new_odom);
    
    // Update last odometry data
    last_odom_time_ = msg->header.stamp;
    last_odom_ = *msg;  // 存储完整的 Odometry 消息
    
    // Log information (reduce frequency to avoid spam)
    static int log_counter = 0;
    if (++log_counter % 50 == 0) {  // Log every 50 messages
        double original_yaw = quaternionToYaw(msg->pose.pose.orientation);
        ROS_INFO("Original yaw: %.3f, Integrated yaw: %.3f, Angular vel: %.3f", 
                 original_yaw, integrated_yaw_, angular_vel_z);
    }
}
void Bridge::VelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // Handle first velocity command
    if (first_cmd_) {
        last_cmd_time_ = ros::Time::now();
        last_velocity_ = *msg;
        first_cmd_ = false;
        ROS_INFO("First velocity command received");
        return;
    }
    
    // Calculate time interval
    ros::Time current_time = ros::Time::now();
    ros::Duration time_interval = current_time - last_cmd_time_;
    double dt = time_interval.toSec();
    
    if (dt <= 0) {
        ROS_WARN("Invalid velocity time interval: %f seconds", dt);
        return;
    }
    
    // Update integrated yaw with current angular velocity
    double angular_vel_z = msg->angular.z;
    integrated_yaw_ += angular_vel_z * dt;
    
    // Normalize yaw to [-pi, pi]
    while (integrated_yaw_ > M_PI) integrated_yaw_ -= 2.0 * M_PI;
    while (integrated_yaw_ < -M_PI) integrated_yaw_ += 2.0 * M_PI;
    
    // Update virtual position using current virtual yaw and linear velocity
    double linear_vel_x = msg->linear.x;
    virtual_x_ += linear_vel_x * std::cos(integrated_yaw_) * dt;
    virtual_y_ += linear_vel_x * std::sin(integrated_yaw_) * dt;
    
    // Create new velocity command with decomposed velocities and no angular velocity
    geometry_msgs::Twist new_cmd_vel;
    
    // Decompose the commanded velocity into global x,y components using virtual yaw
    new_cmd_vel.linear.x = msg->linear.x * std::cos(integrated_yaw_) - msg->angular.z * std::sin(integrated_yaw_);
    new_cmd_vel.linear.y = msg->linear.x * std::sin(integrated_yaw_) + msg->angular.z * std::cos(integrated_yaw_);
    new_cmd_vel.linear.z = 0.0;
    
    // Set angular velocity to zero (as per requirement)
    new_cmd_vel.angular.x = 0.0;
    new_cmd_vel.angular.y = 0.0;
    new_cmd_vel.angular.z = 0.0;
    
    // Store the last velocity command and time
    last_velocity_ = *msg;
    last_cmd_time_ = current_time;
    
    // Publish the new velocity command
    newVel_pub_.publish(new_cmd_vel);
    
    // Log velocity commands (reduce frequency to avoid spam)
    static int vel_log_counter = 0;
    if (++vel_log_counter % 20 == 0) {  // Log every 20 messages
        ROS_INFO("Velocity command - Linear: x=%.3f, y=%.3f, z=%.3f | Angular: x=%.3f, y=%.3f, z=%.3f", 
                 msg->linear.x, msg->linear.y, msg->linear.z,
                 msg->angular.x, msg->angular.y, msg->angular.z);
    }
}







int main(int argc, char** argv) {
    ros::init(argc, argv, "bridge_node");
    ros::NodeHandle nh;

    Bridge bridge("/odom", "/new_cmd_vel","/new_odom","/cmd_vel");
    bridge.spin();
    return 0;
}





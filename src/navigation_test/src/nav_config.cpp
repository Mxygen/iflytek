#include <ros/ros.h>
#include "client.h"
#include <boost/function.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/client.h>
#include "teb_local_planner/TebLocalPlannerReconfigureConfig.h"
#include "costmap_2d/InflationPluginConfig.h"
#include "costmap_2d/Costmap2DConfig.h"
#include "costmap_2d/ObstaclePluginConfig.h"
#include <std_msgs/Int32.h>
#include <map>


ros::Subscriber flag_sub;
typedef boost::function<void(const teb_local_planner::TebLocalPlannerReconfigureConfig &)> CallBack;
typedef boost::function<void(const costmap_2d::InflationPluginConfig &)> CallBack2;
teb_local_planner::TebLocalPlannerReconfigureConfig config, config2, config3, config4, config5;
costmap_2d::InflationPluginConfig config6, config7;
costmap_2d::Costmap2DConfig config8, config9;
costmap_2d::ObstaclePluginConfig config10;
int state = 0, t_aisle = 0, t_ramp = 0;

std::map<int, std::map<std::string, double>> state_params;

void loadParams(ros::NodeHandle &nh)
{
    for (int i = 1; i <= 4; i++)
    {
        std::map<std::string, double> params;
        nh.getParam("/states/state_" + std::to_string(i), params);
        // ROS_INFO("TTTTTTTTTTTTTTTTT %f",params["weight_optimaltime"]);
        state_params[i] = params;
    }

}


void flagCB(const std_msgs::Int32 &msg)
{
    if(msg.data == 1)
    {
        state = 1;
        ROS_INFO("GO GO GO");
    }
    else if(msg.data==2)
    {
        state = 2;
        ROS_INFO("FROM B TO UNKNOWN");
    }
    else if(msg.data==3)
    {
        state = 3;
        ROS_INFO("FROM UNKNOWN TO J");

    }
    else if(msg.data==4)
    {
        state = 4;
        ROS_INFO("FROM J TO A");

    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dyn_client");
    ROS_INFO("Spinning node");
    ros::NodeHandle n("~");
    CallBack tmpdata;
    loadParams(n);

    // CallBack tmpdata;
    dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("move_base/TebLocalPlannerROS");
    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client2("move_base/global_costmap/inflation_layer");
    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client3("move_base/local_costmap/inflation_layer");
    dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig> client4("move_base/global_costmap");
    dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig> client5("move_base/local_costmap");
    flag_sub= n.subscribe("/flag2", 100, flagCB);
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        
        if(state != 0)
        {
            auto &params = state_params[state];
            
            client.getCurrentConfiguration(config2,ros::Duration(1));
            config2.weight_optimaltime = params["weight_optimaltime"];
            config2.xy_goal_tolerance = params["xy_goal_tolerance"];
            config2.yaw_goal_tolerance = params["yaw_goal_tolerance"];
            config2.weight_obstacle = params["weight_obstacle"];
            config2.weight_inflation = params["weight_inflation"];
            config2.weight_acc_lim_x = params["weight_acc_lim_x"];
            config2.weight_max_vel_x = params["weight_max_vel_x"];
            config2.weight_acc_lim_y = params["weight_acc_lim_y"];
            config2.weight_max_vel_y = params["weight_max_vel_y"];
            config2.weight_acc_lim_theta = params["weight_acc_lim_theta"];
            config2.weight_max_vel_theta = params["weight_max_vel_theta"];
            config2.acc_lim_x = params["acc_lim_x"];
            config2.acc_lim_y = params["acc_lim_y"];
            config2.max_vel_x = params["max_vel_x"];
            config2.max_vel_y = params["max_vel_y"];
            config2.max_vel_theta = params["max_vel_theta"];
            config2.acc_lim_theta = params["acc_lim_theta"];
            config2.min_obstacle_dist = params["min_obstacle_dist"];
            config2.inflation_dist = params["inflation_dist"];
            config2.min_turning_radius = params["min_turning_radius"];
            config2.weight_kinematics_nh = params["weight_kinematics_nh"];
            client.setConfiguration(config2);

            client2.getCurrentConfiguration(config6,ros::Duration(1));
            config6.cost_scaling_factor = params["cost_scaling_factor_global"];
            config6.inflation_radius = params["inflation_radius_global"];
            client2.setConfiguration(config6);

            client3.getCurrentConfiguration(config7,ros::Duration(1));
            config7.cost_scaling_factor = params["cost_scaling_factor_local"];
            config7.inflation_radius = params["inflation_radius_local"];
            client3.setConfiguration(config7);

            client4.getCurrentConfiguration(config8,ros::Duration(1));
            config8.robot_radius = params["robot_radius_global"];
            client4.setConfiguration(config8);

            client5.getCurrentConfiguration(config9,ros::Duration(1));
            config9.robot_radius = params["robot_radius_local"];
            client5.setConfiguration(config9);
        }
        state = 0;
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
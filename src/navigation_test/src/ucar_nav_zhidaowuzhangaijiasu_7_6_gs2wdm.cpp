#include <ros/ros.h>
#include "client.h"
#include <dynamic_reconfigure/server.h>
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
#include<std_msgs/Int32.h>
ros::Subscriber flag_sub ;
typedef boost::function<void(const teb_local_planner::TebLocalPlannerReconfigureConfig &)> CallBack;
typedef boost::function<void(const costmap_2d::InflationPluginConfig &)> CallBack2;
teb_local_planner::TebLocalPlannerReconfigureConfig config,config2,config3,config4,config5;
costmap_2d::InflationPluginConfig config6,config7;
costmap_2d::Costmap2DConfig config8,config9;
costmap_2d::ObstaclePluginConfig config10;
int state = 0,t_aisle = 0,t_ramp = 0;
void dynCallBack(const teb_local_planner::TebLocalPlannerReconfigureConfig &data)
{
}
void dynCallBack2(const costmap_2d::InflationPluginConfig &data)
{
}
void dynCallBack3(const costmap_2d::InflationPluginConfig &data)
{
}
void dynCallBack4(const costmap_2d::Costmap2DConfig &data)
{
}
void dynCallBack5(const costmap_2d::Costmap2DConfig &data)
{
}
void dynCallBack6(const costmap_2d::ObstaclePluginConfig &data)
{
}
void flagCB(const std_msgs::Int32& msg)
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
        ROS_INFO("FROM UNKNOWN TO picture");

    }
    else if(msg.data==4)
    {
        state = 4;
        ROS_INFO("FROM picture TO J");

    }
    else if(msg.data==5)
    {
        state = 5;
        ROS_INFO("FROM J TO A");

    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_client");
    ROS_INFO("Spinning node");
    ros::NodeHandle n("~");
    CallBack tmpdata;
    dynamic_reconfigure::Client<teb_local_planner::TebLocalPlannerReconfigureConfig> client("/move_base/TebLocalPlannerROS/", dynCallBack);
    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client2("/move_base/global_costmap/inflation_layer/", dynCallBack2);
    dynamic_reconfigure::Client<costmap_2d::InflationPluginConfig> client3("/move_base/local_costmap/inflation_layer/", dynCallBack3);
    dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig> client4("/move_base/global_costmap/", dynCallBack4);
    dynamic_reconfigure::Client<costmap_2d::Costmap2DConfig> client5("/move_base/local_costmap/", dynCallBack5);
    dynamic_reconfigure::Client<costmap_2d::ObstaclePluginConfig> client6("/move_base/global_costmap/obstacle_layer/", dynCallBack6);
    flag_sub= n.subscribe("/flag1", 100, flagCB);
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        /*
        *client--TEB--config/config2/config3/config4/config5
        *client2--globalcostmap--config6--inflation_radius
        *client3--localcostmap--config7--inflation_radius
        *client4--globalcostmap--config8--robot_radius
        *client5--localcostmap--config9--robot_radius
        *client6--obstaclelayer--config10--clearing
        */
        //weight_optimaltime�????1.当小车在某些位置卡住不敢走（还没撞）时，考虑膨胀半径是否应该调小，和weight_optimaltime是否应该调大�????
        //max_global_plan_lookahead_dist�????2.刚开始调，可以速度调慢点，max_global_plan_lookahead_dist（向前规划距离）调小，可以使得局部路径较贴近全局路径并完成跑图，再慢慢调大找最佳�?
        /*
        *inflation_radius/robot_radius：如遇正面碰撞，先看rviz，看膨胀够不够大，小车的模型是否避开膨胀，若避开不及时或是贴很近，但导航路径是有避开的，则修改inflation_radius(增大)或增加小车半径robot_radius
        *正常来说现在膨胀这一系列的权重还行的，不用修�????
        *weight_acc_lim_theta/weight_max_vel_theta增加小车角速度�????2个权重，可以让小车走路时更倾向于用角度来走，减小而是倾向Y轴走
        *weight_optimaltime:增大会让小车更激进，但路径规划会更切�????
        *inflation_dist障碍物周围的缓冲区，小车进入这个里面会减速，减小这个，可能可以提高小车在区域范围内的速度，但有碰撞的风险，在提速的代码中，可以减小这个
        *min_obstacle_dist，最小障碍物距离，小车里障碍物最小的距离，这个值必须比上面的inflation_dist�????
        *cost_scaling_factor，障碍物膨胀比例系数，如果看到小车规划的路径有比较明显的锯齿状，要提高这个，也就是让路径圆滑
        */
            if(state == 1 )             //前往B区域�????
            {
                state=10;
                client.getCurrentConfiguration(config2,ros::Duration(1));
                config2.weight_optimaltime=35;    
                config2.xy_goal_tolerance=0.1;                                                    
                config2.yaw_goal_tolerance=0.1;
                config2.weight_obstacle=800;
                config2.weight_inflation=0.5;
                config2.weight_acc_lim_x=10;
                config2.weight_max_vel_x=10;
                config2.weight_acc_lim_y=2;
                config2.weight_max_vel_y=2;
                config2.weight_acc_lim_theta = 25; 
                config2.weight_max_vel_theta = 25;
                config2.acc_lim_x=0.6;
                config2.acc_lim_y=0.2;
                config2.max_vel_x=0.4;
                config2.max_vel_y=0.2;
                config2.max_vel_theta=3;
                config2.acc_lim_theta=3;
                config2.min_obstacle_dist=0.28;
                config2.inflation_dist=0.3;
                config2.min_turning_radius = 0.00;
                config2.weight_kinematics_nh = 100;
                client.setConfiguration(config2);

                // //client2   config6
                client2.getCurrentConfiguration(config6,ros::Duration(1));
                config6.cost_scaling_factor = 20.0;
                config6.inflation_radius=0.13;
                client2.setConfiguration(config6);
                // //client3   config7
                client3.getCurrentConfiguration(config7,ros::Duration(1));
                config7.cost_scaling_factor = 15.0;
                config7.inflation_radius=0.1;
                client3.setConfiguration(config7);

                // 一般不开
                client4.getCurrentConfiguration(config8,ros::Duration(1));
                config8.robot_radius=0.2;
                client4.setConfiguration(config8);
                client5.getCurrentConfiguration(config9,ros::Duration(1));
                config9.robot_radius=0.2;
                client5.setConfiguration(config9);


            }
            else if(state == 2)                 //从B前往未知区域
            {
                state=10;
                // printf("Earea state!\n");
                client.getCurrentConfiguration(config2,ros::Duration(1));
                config2.weight_optimaltime=20;                     
                config2.xy_goal_tolerance=0.1;                                                     
                config2.yaw_goal_tolerance=0.1;
                config2.weight_obstacle=800;
                config2.weight_inflation=0.5;
                config2.weight_acc_lim_x=7;
                config2.weight_max_vel_x=7;
                config2.weight_acc_lim_y=2;
                config2.weight_max_vel_y=2;
                config2.weight_acc_lim_theta = 25; 
                config2.weight_max_vel_theta = 25;
                config2.acc_lim_x=2;
                config2.acc_lim_y=0.8;
                config2.max_vel_x=1.5;
                config2.max_vel_y=0.5;    
                config2.max_vel_theta=3;                     
                config2.acc_lim_theta=3;
                config2.min_obstacle_dist=0.28; 
                config2.inflation_dist=0.3;
                config2.min_turning_radius = 0.00;
                config2.weight_kinematics_nh = 350;    //设置比较大时，将倾向于纵向运动；当此参数设置较小时，将更倾向于与横向运动 400
                client.setConfiguration(config2);

                // //client2   config6
                client2.getCurrentConfiguration(config6,ros::Duration(1));
                config6.cost_scaling_factor = 15.0;
                config6.inflation_radius=0.13;
                client2.setConfiguration(config6);
                // //client3   config7
                client3.getCurrentConfiguration(config7,ros::Duration(1));
                config7.cost_scaling_factor = 15.0;
                config7.inflation_radius=0.1;
                client3.setConfiguration(config7);

                client4.getCurrentConfiguration(config8,ros::Duration(1));
                config8.robot_radius=0.18;
                client4.setConfiguration(config8);
                client5.getCurrentConfiguration(config9,ros::Duration(1));
                config9.robot_radius=0.18;
                client5.setConfiguration(config9);
            }
            else if(state == 3)                 
            {
                state=10;
                t_ramp++;
                printf("%d ramp state!\n",t_ramp);
                client.getCurrentConfiguration(config2,ros::Duration(1));
                config2.weight_optimaltime=20;                  //基于规划的时间上的权�???? 提高�????3-5时，车辆在直道上快速加速，并靠近路径边缘沿切线过弯（不怎么动）                                    
                config2.xy_goal_tolerance=0.06;                  //xy方向的偏移容忍度        （这个值差不多的）                               
                config2.yaw_goal_tolerance=0.15;                 //偏航角的偏移容忍�????
                config2.weight_obstacle=800;                    //和障碍物最小距离的权重      （最�????1000，拉大一点）
                config2.weight_inflation=0.5;                   //膨胀区权�????                （最�????10，没动过，可以拉高一点试试）
                config2.weight_acc_lim_x=10;                    //最大加速度权重             （最�????1000，也可以调一调）
                config2.weight_max_vel_x=10;                    //最大x速度权重              （最�????1000�????
                config2.weight_acc_lim_y=10;                                 
                config2.weight_max_vel_y=10;                                 
                config2.weight_acc_lim_theta = 15;                      
                config2.weight_max_vel_theta = 15;                      
                config2.acc_lim_x=0.6;                          //最大x加速度               （main)
                config2.acc_lim_y=0.5;                                      
                config2.max_vel_x=0.7;                           //最大x速度                 （main)
                config2.max_vel_y=0.8;                                    
                config2.max_vel_theta=2;                         //最大角速度                （main)
                config2.acc_lim_theta=2;                                    
                config2.min_obstacle_dist=0.26;                 //和障碍物最小距�????
                config2.inflation_dist=0.28;                                
                config2.min_turning_radius = 0.00;              //最小转弯半�????
                config2.weight_kinematics_nh = 400;             //设置比较大时，将倾向于纵向运动；当此参数设置较小时，将更倾向于与横向运动  （最�????10000�????
                client.setConfiguration(config2);

                // //client2   config6  globalcostmap
                client2.getCurrentConfiguration(config6,ros::Duration(1));
                config6.cost_scaling_factor = 15.0;             //代价比例系数，越大则代价值越�????
                config6.inflation_radius=0.1;                  //膨胀半径
                client2.setConfiguration(config6);
                // //client3   config7  localcostmap
                client3.getCurrentConfiguration(config7,ros::Duration(1));
                config7.inflation_radius=0.1;
                client3.setConfiguration(config7);

                client4.getCurrentConfiguration(config8,ros::Duration(1));
                config8.robot_radius=0.2;                       //小车半径
                client4.setConfiguration(config8);
                client5.getCurrentConfiguration(config9,ros::Duration(1));
                config9.robot_radius=0.2;
                client5.setConfiguration(config9);
            }
            else if(state == 4)             //从识别板到J或者下一个点
            {
                state=10;
                client.getCurrentConfiguration(config2,ros::Duration(1));
                config2.weight_optimaltime=2;                                         
                config2.xy_goal_tolerance=0.1;                                                    
                config2.yaw_goal_tolerance=0.15;
                config2.weight_obstacle=800;
                config2.weight_inflation=0.5;
                config2.weight_acc_lim_x=10;
                config2.weight_max_vel_x=10;
                config2.weight_acc_lim_y=10;
                config2.weight_max_vel_y=10;
                config2.weight_acc_lim_theta = 10;
                config2.weight_max_vel_theta = 10;
                config2.acc_lim_x=2.5;
                config2.acc_lim_y=2.5;
                config2.max_vel_x=1.7;
                config2.max_vel_y=1.7;
                config2.max_vel_theta=3;
                config2.acc_lim_theta=3;
                config2.min_obstacle_dist=0.28;
                config2.inflation_dist=0.3;
                config2.min_turning_radius = 0.00;
                config2.weight_kinematics_nh = 100;
                client.setConfiguration(config2);

                // //client2   config6
                client2.getCurrentConfiguration(config6,ros::Duration(1));
                config6.cost_scaling_factor = 15.0;
                config6.inflation_radius=0.14;
                client2.setConfiguration(config6);
                // //client3   config7
                client3.getCurrentConfiguration(config7,ros::Duration(1));
                config7.cost_scaling_factor = 15.0;
                config7.inflation_radius=0.16;
                client3.setConfiguration(config7);

                client4.getCurrentConfiguration(config8,ros::Duration(1));
                config8.robot_radius=0.19;
                client4.setConfiguration(config8);
                client5.getCurrentConfiguration(config9,ros::Duration(1));
                config9.robot_radius=0.19;
                client5.setConfiguration(config9);
            }
            else if(state == 5)
            {
                state=10;
                client.getCurrentConfiguration(config2,ros::Duration(1));
                config2.weight_optimaltime=2;                                         
                config2.xy_goal_tolerance=0.1;                                                    
                config2.yaw_goal_tolerance=0.15;
                config2.weight_obstacle=800;
                config2.weight_inflation=0.5;
                config2.weight_acc_lim_x=10;
                config2.weight_max_vel_x=10;
                config2.weight_acc_lim_y=5;
                config2.weight_max_vel_y=5;
                config2.weight_acc_lim_theta = 25;
                config2.weight_max_vel_theta = 25;
                config2.acc_lim_x=3;
                config2.acc_lim_y=1;
                config2.max_vel_x=2;
                config2.max_vel_y=1;
                config2.max_vel_theta=3;
                config2.acc_lim_theta=2.1;
                config2.min_obstacle_dist=0.28;
                config2.inflation_dist=0.3;
                config2.min_turning_radius = 0.00;
                config2.weight_kinematics_nh = 300;
                client.setConfiguration(config2);

                // //client2   config6
                client2.getCurrentConfiguration(config6,ros::Duration(1));
                config6.cost_scaling_factor = 15.0;
                config6.inflation_radius=0.14;
                client2.setConfiguration(config6);
                // //client3   config7
                client3.getCurrentConfiguration(config7,ros::Duration(1));
                config7.cost_scaling_factor = 15.0;
                config7.inflation_radius=0.16;
                client3.setConfiguration(config7);

            }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Spinning node shutdown...");
    return 0;
}

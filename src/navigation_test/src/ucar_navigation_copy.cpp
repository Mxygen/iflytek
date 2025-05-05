#include <ros/ros.h>
#include <cstdlib>

#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <tf/transform_listener.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/QR>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pudding/Pudding.h>

/******************************变量定义************************************/

using namespace std;

bool use_slam = false,only_line = false;

int isone = 0, get_yaw = 0,need_twice = 0,shutdown_flag = 0;
int numnumnum = 0;                                                      ////////////////////////////////////////////////////////////////////////////////////////////////

double goal_areaB_x;
double goal_areaB_y;
double goal_areaB_z;

double goal_areaR_x;
double goal_areaR_y;
double goal_areaR_z;

double goal_areaJ_1_x;
double goal_areaJ_1_y;
double goal_areaJ_1_z;

double goal_areaJ_2_x;
double goal_areaJ_2_y;
double goal_areaJ_2_z;

double goal_areaJ_3_x;
double goal_areaJ_3_y;
double goal_areaJ_3_z;

double goal_areaJ_x;
double goal_areaJ_y;
double goal_areaJ_z;

double goal_areaA_x;
double goal_areaA_y;
double goal_areaA_z;
double goal_areaA_zs = 0;

double goal_areaGO_x;
double goal_areaGO_y;
double goal_areaGO_z;

double goal_areaGO2_x;
double goal_areaGO2_y;
double goal_areaGO2_z;

double goal_angle_R[10] = {0, 0, 0, 0, 0, 0,0,0,0,0}; // 一个点给到三个角度
double goal_angle_J_1[10] = {0, 0, 0, 0, 0, 0,0,0,0,0}; // 一个点给到三个角度
double goal_angle_J_3[10] = {0, 0, 0, 0, 0, 0,0,0,0,0}; // 一个点给到三个角度
double goal_angle_J[10] = {0, 0, 0, 0, 0, 0,0,0,0,0}; // 一个点给到三个角度
double goal_angle_J_2[10] = {0, 0, 0, 0, 0, 0,0,0,0,0}; // 一个点给到三个角度
int R_num = 7;
int J_1_num = 7;
int J_3_num = 7;
int J_num = 7;
int J_2_num = 7;

double turn_vel_J = 0.2;

double dead_dis = 0.4;
double dead_dis_2 = 1.0;


#define sleep_count 30


/*标志�?????????????????*/
#define status_stop -2
#define status_waitting_for_awake -1
#define status_origin 0

#define status_goingto_areaB 1
#define status_arrived_areaB 2

#define status_goingto_areaR 3
#define status_arrived_areaR 4


#define status_arrived_areaJ_1 6

#define status_arrived_areaJ_2 10
#define status_arrived_areaJ_3 12

#define status_goingto_areaJ 13
#define status_arrived_areaJ 14

#define status_goingto_areaA 15
#define status_arrived_areaA 16

#define status_arrived_areaGO 18
#define status_arrived_areaGO2 20
#define status_arrived_areaGO3 22


/*摄像头标志位*/
#define cv_area_get 14
#define cv_stop 22
#define cv_line 33
#define video_over 66

/*参数配置标志*/
#define config_1 1
#define config_2 2
#define config_3 3
#define config_4 4
#define config_5 5



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
typedef actionlib::SimpleClientGoalState State;
State::StateEnum state;
// ros::ServiceClient mapclean_client;


move_base_msgs::MoveBaseGoal goal;
geometry_msgs::PoseWithCovarianceStamped amcl_pose_now;

ros::Subscriber fake_vel_sub;
ros::Subscriber cv_mode_sub;
ros::Subscriber cv_position_sub;
ros::Subscriber cv_line_sub;

ros::Publisher flag_dp_pub;
ros::Publisher flag_config_pub;
ros::Publisher pub_ClearCostmap;
ros::Publisher init_pub;
ros::Publisher cmd_vel_pub;

ros::Timer timer1;

geometry_msgs::Twist fake_vel_now;

geometry_msgs::Pose pose_areaB;
geometry_msgs::Pose pose_areaR;
geometry_msgs::Pose pose_areaJ_1;
geometry_msgs::Pose pose_areaJ_2;
geometry_msgs::Pose pose_areaJ_3;
geometry_msgs::Pose pose_areaJ;
geometry_msgs::Pose pose_areaA;
geometry_msgs::Pose pose_area_GO;
geometry_msgs::Pose pose_area_GO2;

int clear_map_B = 0, clear_map_R = 0, clear_map_J_1 = 0, clear_map_J_2 = 0, clear_map_J_3 = 0, clear_map_J = 0,clear_map_GO = 0,clear_map_GO2 = 0;
char goal_received_situation = 0, goal_received_judge = 0, already_awake = 0;
int num_turn = 0,last_num_turn = -1, get_pic_flag = 0, stop_flag = 0, get_terrorist_success = 0,special_flag = 0;
double now_yaw = 0, now_x = 0, now_y = 0, px = 0, py = 0, mid_dis = 0, cv_yaw = 0, angle_increment = 0, goal_dis = 0,last_now_yaw = 0;
double cospsi = 0, sinpsi = 0, cospsiR = 0, sinpsiR = 0;
int time_count = 0;
double turn_line = 0, last_turn_line = 0, Kp, Kd, Kp_midline, Kd_midline, Kp_turnright, Kd_turnright, Kp_turnleft, Kd_turnleft;
int voice_to_play = 0, replan_over = 0;
double temp_yaw = 0;

std_msgs::Int8 cv_mode, ucar_state_now, hor1_mode, hor2_mode, hor3_mode, B_area_mode;
std_msgs::Int32 x_cv;
std_msgs::String clearcostmap;
std_msgs::Int32 flag_dp_now;
std_msgs::Int32 flag_config_now;

typedef enum
{
    SUCCEEDED,
    ACTIVE,
    PENDING,
    ABORTED,
    REJECTED
} GoalStatus;

GoalStatus goal_status;

void shutdownNode(const std::string& node_name)
{
    std::string command = "rosnode kill " + node_name;
    int ret = system(command.c_str());
    if (ret == 0)
    {
        ROS_INFO("Successfully killed node: %s", node_name.c_str());
    }
    else
    {
        ROS_ERROR("Failed to kill node: %s", node_name.c_str());
    }
}


double replandeal(double x1, double x2)
{
    if (x1 - x2 > 0)
    {
        return x2 + 0.1;
    }
    else if (x1 - x2 < 0)
    {
        return x2 - 0.1;
    }
    else
    {
        return x2;
    }
}

geometry_msgs::Point setPoint(double _x, double _y, double _z)
{
    geometry_msgs::Point m_point;
    m_point.x = _x;
    m_point.y = _y;
    m_point.z = _z;
    return m_point;
}
geometry_msgs::Quaternion setQuaternion(double _angleRan)
{
    geometry_msgs::Quaternion m_quaternion;
    m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan); // 这是一个欧拉角转四元素，车子只有Yaw这个自由�?????????????????
    return m_quaternion;
}

/*****************************************************************************************/

void LidarCallback(const sensor_msgs::LaserScan msg)
{
    mid_dis = msg.ranges[395];           
    // goal_dis = msg.ranges[567];  
    angle_increment = msg.angle_increment; // 角度增量
    // ROS_WARN("正前方测距：%f",mid_dis);
    if (get_terrorist_success && isone == 0 && get_yaw == 2)
    {
        int num_dis = 395 + (int)(temp_yaw / angle_increment);
        // goal_dis = msg.ranges[num_dis]; // 雷达定位到的板点到车身的距离     ////////////////////////////////////////////////////////////////////////////////////////////////
        ROS_INFO("goal_dis = %f",goal_dis);
        if(goal_dis < 3.0 && goal_dis > 0)
        {
            if(goal_dis > 1.2)
            {
                dead_dis = 0.35;
            }
            ROS_INFO("goal_dis = %f now_yaw = %f px = %f py = %f temp = %f ", goal_dis,now_yaw,px,py,temp_yaw);
            numnumnum = 0;                                                  ////////////////////////////////////////////////////////////////////////////////////////////////
            goal_areaGO_y = sin(now_yaw) * ((cospsi * goal_dis) - (dead_dis * cospsiR)) + cos(now_yaw) * ((sinpsi * goal_dis) - (dead_dis * sinpsiR)) + py;
            goal_areaGO_x = cos(now_yaw) * ((cospsi * goal_dis) - (dead_dis * cospsiR)) - sin(now_yaw) * ((sinpsi * goal_dis) - (dead_dis * sinpsiR)) + px;
            goal_areaGO_z = cv_yaw;
            pose_area_GO.position = setPoint(goal_areaGO_x, goal_areaGO_y, 0);
            pose_area_GO.orientation = setQuaternion(goal_areaGO_z);
            isone = 1;
            get_yaw = 0;
            ROS_WARN("SS %f SSS %f SSS %f", goal_areaGO_x, goal_areaGO_y, goal_areaGO_z);
        }
        else if((goal_dis >= 3.0 && goal_dis < 4.0) || numnumnum > 10)       ////////////////////////////////////////////////////////////////////////////////////////////////
        {
            ROS_INFO("goal_dis = %f now_yaw = %f px = %f py = %f temp = %f ", goal_dis,now_yaw,px,py,temp_yaw);
            numnumnum = 0;                                                  ////////////////////////////////////////////////////////////////////////////////////////////////
            goal_dis = 1.3;                                                 /////////////////////////////////////////////////////////////////////////////////////////////////
            goal_areaGO_y = sin(now_yaw) * ((cospsi * goal_dis) - (dead_dis_2 * cospsiR)) + cos(now_yaw) * ((sinpsi * goal_dis) - (dead_dis_2 * sinpsiR)) + py;
            goal_areaGO_x = cos(now_yaw) * ((cospsi * goal_dis) - (dead_dis_2 * cospsiR)) - sin(now_yaw) * ((sinpsi * goal_dis) - (dead_dis_2 * sinpsiR)) + px;
            goal_areaGO_z = cv_yaw;
            pose_area_GO.position = setPoint(goal_areaGO_x, goal_areaGO_y, 0);
            pose_area_GO.orientation = setQuaternion(goal_areaGO_z);
            isone = 1;
            get_yaw = 0;
            ROS_WARN("SS %f SSS %f SSS %f", goal_areaGO_x, goal_areaGO_y, goal_areaGO_z);
            need_twice = 1;
        }
        else
        {
            numnumnum++;                                                    ////////////////////////////////////////////////////////////////////////////////////////////////
        }
    }
    if(need_twice == 2 && get_yaw == 2 && isone == 1)
    {
        int num_dis = 395 + (int)(temp_yaw / angle_increment);
        goal_dis = msg.ranges[num_dis]; // 雷达定位到的板点到车身的距离
        if((goal_dis < 4.0 && goal_dis > 0) || numnumnum > 10)               ////////////////////////////////////////////////////////////////////////////////////////////////
        {
            ROS_INFO("goal_dis = %f now_yaw = %f px = %f py = %f temp = %f ", goal_dis,now_yaw,px,py,temp_yaw);
            numnumnum = 0;                                                  ////////////////////////////////////////////////////////////////////////////////////////////////
            goal_areaGO2_y = sin(now_yaw) * ((cospsi * goal_dis) - (dead_dis * cospsiR)) + cos(now_yaw) * ((sinpsi * goal_dis) - (dead_dis * sinpsiR)) + py;
            goal_areaGO2_x = cos(now_yaw) * ((cospsi * goal_dis) - (dead_dis * cospsiR)) - sin(now_yaw) * ((sinpsi * goal_dis) - (dead_dis * sinpsiR)) + px;
            goal_areaGO2_z = cv_yaw;
            pose_area_GO2.position = setPoint(goal_areaGO2_x, goal_areaGO2_y, 0);
            pose_area_GO2.orientation = setQuaternion(goal_areaGO2_z);
            isone = 2;
            ROS_WARN("SS %f SSS %f SSS %f", goal_areaGO2_x, goal_areaGO2_y, goal_areaGO2_z);
        }
        else
        {
            numnumnum++;                                                    ////////////////////////////////////////////////////////////////////////////////////////////////
        }
    }
}

// void yaw_CB(const std_msgs::Float32 &msg)
// {
//     cv_yaw = msg.data + now_yaw; 
//     if(cv_yaw > M_PI)
//         cv_yaw = cv_yaw - M_PI*2;
//     else if(cv_yaw < -M_PI)
//         cv_yaw = cv_yaw + M_PI*2;
//     cospsiR = cos(msg.data);     
//     sinpsiR = sin(msg.data);    
//     get_yaw = 1;
//     ROS_INFO("HHH %f HHHH %f HHHHHH % f HHHHH", px,py,now_yaw);
// }

void yaw_CB(const pudding::Pudding &msg)
{
    cv_yaw = msg.fitting_yaw + now_yaw; 
    goal_dis = msg.temp_dis;
    if(cv_yaw > M_PI)
        cv_yaw = cv_yaw - M_PI*2;
    else if(cv_yaw < -M_PI)
        cv_yaw = cv_yaw + M_PI*2;
    cospsiR = cos(msg.fitting_yaw);     
    sinpsiR = sin(msg.fitting_yaw);    
    get_yaw = 1;
    ROS_INFO("HHH %f HHHH %f HHHHHH % f HHHHH", px,py,now_yaw);
}

// void pic_positionCB(const pudding::Pudding::ConstPtr& pic)
// {
//     int mid = (pic->left + pic->right)/2;
//     if (get_yaw == 1 && mid != -1)
//     {
//         temp_yaw = -M_PI * (0.125 * mid - 40) / 180;
//         cospsi = cos(temp_yaw); 
//         sinpsi = sin(temp_yaw); 
//         get_yaw = 2;   
//         ROS_WARN("temp_yaw : %f",temp_yaw);
//     }
//     if(need_twice == 2 && mid != -1)
//     {
//         ROS_WARN("needtwice 2 : %d",mid);
//         temp_yaw = -M_PI * (0.125 * mid - 40) / 180;
//         cospsi = cos(temp_yaw); 
//         sinpsi = sin(temp_yaw); 
//         cospsiR = cos(cv_yaw - now_yaw);
//         sinpsiR = sin(cv_yaw - now_yaw);
//         get_yaw = 2;

//     }
// }

void pic_positionCB(const std_msgs::Int32 &pic)
{
    if (get_yaw == 1 && pic.data != -1)
    {
        temp_yaw = -M_PI * (0.125 * pic.data - 40) / 180;
        cospsi = cos(temp_yaw); 
        sinpsi = sin(temp_yaw); 
        get_yaw = 2;   
        ROS_WARN("temp_yaw : %f",temp_yaw);
    }
    if(need_twice == 2 && pic.data != -1)
    {
        ROS_WARN("needtwice 2 : %d",pic.data);
        temp_yaw = -M_PI * (0.125 * pic.data - 40) / 180;
        cospsi = cos(temp_yaw); 
        sinpsi = sin(temp_yaw); 
        cospsiR = cos(cv_yaw - now_yaw);
        sinpsiR = sin(cv_yaw - now_yaw);
        get_yaw = 2;

    }
}

void poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_poseMsg)
{
    amcl_pose_now = *amcl_poseMsg;
    tf::Pose pose_tf;
    tf::poseMsgToTF(amcl_pose_now.pose.pose, pose_tf);
    px = amcl_pose_now.pose.pose.position.x; 
    py = amcl_pose_now.pose.pose.position.y; 
    double psi = tf::getYaw(pose_tf.getRotation());
    now_yaw = psi; 
    if(now_yaw > M_PI)
    {
        now_yaw = now_yaw - M_PI*2;
    }
    if(psi < -M_PI)
    {
        now_yaw = now_yaw + M_PI*2;
    }
    
}

void Send_to_pic(void)
{
    flag_dp_now.data = cv_area_get;
    flag_dp_pub.publish(flag_dp_now);
}
void Send_to_config(int v)
{
    flag_config_now.data = v;
    flag_config_pub.publish(flag_config_now);
}



void init_param_goal()
{

    ros::NodeHandle n("~");
    n.param<bool>("use_slam_", use_slam, false);
    n.param<bool>("only_line_", only_line, false);

    // 区域B，第一个停车点
    n.param<double>("goal_areaB_x_", goal_areaB_x, 0);
    n.param<double>("goal_areaB_y_", goal_areaB_y, 0);
    n.param<double>("goal_areaB_z_", goal_areaB_z, 0);

    // 区域J，坡前的一个点
    n.param<double>("goal_areaR_x_", goal_areaR_x, 0);
    n.param<double>("goal_areaR_y_", goal_areaR_y, 0);
    n.param<double>("goal_areaR_z_", goal_areaR_z, 0);

    // 区域J，第一�???????
    n.param<double>("goal_areaJ_1_x_", goal_areaJ_1_x, 0);
    n.param<double>("goal_areaJ_1_y_", goal_areaJ_1_y, 0);
    n.param<double>("goal_areaJ_1_z_", goal_areaJ_1_z, 0);
    // 区域J，第二个
    n.param<double>("goal_areaJ_2_x_", goal_areaJ_2_x, 0);
    n.param<double>("goal_areaJ_2_y_", goal_areaJ_2_y, 0);
    n.param<double>("goal_areaJ_2_z_", goal_areaJ_2_z, 0);
    // 区域J，第三个
    n.param<double>("goal_areaJ_3_x_", goal_areaJ_3_x, 0);
    n.param<double>("goal_areaJ_3_y_", goal_areaJ_3_y, 0);
    n.param<double>("goal_areaJ_3_z_", goal_areaJ_3_z, 0);
    // 急救包固定点
    n.param<double>("goal_areaJ_x_", goal_areaJ_x, 0);
    n.param<double>("goal_areaJ_y_", goal_areaJ_y, 0);
    n.param<double>("goal_areaJ_z_", goal_areaJ_z, 0);
    // 区域A
    n.param<double>("goal_areaA_x_", goal_areaA_x, 0);
    n.param<double>("goal_areaA_y_", goal_areaA_y, 0);
    n.param<double>("goal_areaA_z_", goal_areaA_z, 0);
    n.param<double>("goal_areaA_zs_", goal_areaA_zs, 0);

    n.param<double>("goal_angle_R_1_", goal_angle_R[0], 0);
    n.param<double>("goal_angle_R_2_", goal_angle_R[1], 0);
    n.param<double>("goal_angle_R_3_", goal_angle_R[2], 0);
    n.param<double>("goal_angle_R_4_", goal_angle_R[3], 0);
    n.param<double>("goal_angle_R_5_", goal_angle_R[4], 0);
    n.param<double>("goal_angle_R_6_", goal_angle_R[5], 0);
    n.param<double>("goal_angle_R_7_", goal_angle_R[6], 0);
    n.param<double>("goal_angle_R_8_", goal_angle_R[7], 0);

    n.param<double>("goal_angle_J_1_1_", goal_angle_J_1[0], 0);
    n.param<double>("goal_angle_J_1_2_", goal_angle_J_1[1], 0);
    n.param<double>("goal_angle_J_1_3_", goal_angle_J_1[2], 0);
    n.param<double>("goal_angle_J_1_4_", goal_angle_J_1[3], 0);
    n.param<double>("goal_angle_J_1_5_", goal_angle_J_1[4], 0);
    n.param<double>("goal_angle_J_1_6_", goal_angle_J_1[5], 0);
    n.param<double>("goal_angle_J_1_7_", goal_angle_J_1[6], 0);
    n.param<double>("goal_angle_J_1_8_", goal_angle_J_1[7], 0);

    n.param<double>("goal_angle_J_3_1_", goal_angle_J_3[0], 0);
    n.param<double>("goal_angle_J_3_2_", goal_angle_J_3[1], 0);
    n.param<double>("goal_angle_J_3_3_", goal_angle_J_3[2], 0);
    n.param<double>("goal_angle_J_3_4_", goal_angle_J_3[3], 0);
    n.param<double>("goal_angle_J_3_5_", goal_angle_J_3[4], 0);
    n.param<double>("goal_angle_J_3_6_", goal_angle_J_3[5], 0);
    n.param<double>("goal_angle_J_3_7_", goal_angle_J_3[6], 0);
    n.param<double>("goal_angle_J_3_8_", goal_angle_J_3[7], 0);

    n.param<double>("goal_angle_J_1_", goal_angle_J[0], 0);
    n.param<double>("goal_angle_J_2_", goal_angle_J[1], 0);
    n.param<double>("goal_angle_J_3_", goal_angle_J[2], 0);
    n.param<double>("goal_angle_J_4_", goal_angle_J[3], 0);

    n.param<double>("goal_angle_J_2_1_", goal_angle_J_2[0], 0);
    n.param<double>("goal_angle_J_2_2_", goal_angle_J_2[1], 0);
    n.param<double>("goal_angle_J_2_3_", goal_angle_J_2[2], 0);
    n.param<double>("goal_angle_J_2_4_", goal_angle_J_2[3], 0);
    n.param<double>("goal_angle_J_2_5_", goal_angle_J_2[4], 0);
    n.param<double>("goal_angle_J_2_6_", goal_angle_J_2[5], 0);
    n.param<double>("goal_angle_J_2_7_", goal_angle_J_2[6], 0);
    n.param<double>("goal_angle_J_2_8_", goal_angle_J_2[7], 0);

    n.param<int>("R_num_", R_num, 7);
    n.param<int>("J_1_num_", J_1_num, 7);
    n.param<int>("J_3_num_", J_3_num, 7);
    n.param<int>("J_num_", J_num, 4);
    n.param<int>("J_2_num_", J_2_num, 7);


    n.param<double>("turn_vel_J_", turn_vel_J, 0);

    n.param<double>("dead_dis_", dead_dis, 0.4);
    n.param<double>("dead_dis_2_", dead_dis_2, 1.0);

    pose_areaB.position = setPoint(goal_areaB_x, goal_areaB_y, 0);
    pose_areaB.orientation = setQuaternion(goal_areaB_z);
    pose_areaR.position = setPoint(goal_areaR_x, goal_areaR_y, 0);
    pose_areaR.orientation = setQuaternion(goal_areaR_z);
    pose_areaJ_1.position = setPoint(goal_areaJ_1_x, goal_areaJ_1_y, 0);
    pose_areaJ_1.orientation = setQuaternion(goal_areaJ_1_z);
    pose_areaJ_2.position = setPoint(goal_areaJ_2_x, goal_areaJ_2_y, 0);
    pose_areaJ_2.orientation = setQuaternion(goal_areaJ_2_z);
    pose_areaJ_3.position = setPoint(goal_areaJ_3_x, goal_areaJ_3_y, 0);
    pose_areaJ_3.orientation = setQuaternion(goal_areaJ_3_z);
    pose_areaJ.position = setPoint(goal_areaJ_x, goal_areaJ_y, 0);
    pose_areaJ.orientation = setQuaternion(goal_areaJ_z);
    if (use_slam)
    {
        pose_areaA.position = setPoint(0, -2.0, 0);
        pose_areaA.orientation = setQuaternion(-1.57);
    } 
    else
    {
        pose_areaA.position = setPoint(goal_areaA_x, goal_areaA_y, 0);
        pose_areaA.orientation = setQuaternion(goal_areaA_z);
    }
        
}

/*小车唤醒判断*/
void flag_Callback(std_msgs::Int32 msg_flag)
{
    if (!already_awake && msg_flag.data == 1)
    {
        ucar_state_now.data = status_origin;
        already_awake = 1;
    }
}


void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{

    if (state == State::SUCCEEDED)
    {
        if (ucar_state_now.data == status_goingto_areaB)
        {
            ucar_state_now.data = status_arrived_areaB;
            ROS_INFO("到达区域B");
        }
        else if (ucar_state_now.data == status_goingto_areaR)
        {
            ucar_state_now.data = status_arrived_areaR;
            ROS_INFO("到达区域R");
        }
        else if (ucar_state_now.data == status_goingto_areaA)
        {
            ucar_state_now.data = status_arrived_areaA;
            ROS_INFO("到达区域A");
        }
        else if (ucar_state_now.data == status_goingto_areaJ)
        {
            ucar_state_now.data = status_arrived_areaJ;
            ROS_INFO("到达区域J");
        }
    }
}

/*速度发布回调函数*/
void directionControlLoopCB(const ros::TimerEvent &event)
{
    cmd_vel_pub.publish(fake_vel_now);
}

void fake_velCB(const geometry_msgs::Twist::ConstPtr &fake_velMsg)
{
    fake_vel_now = *fake_velMsg;
}

void activeCb()
{
    goal_received_situation = 1;
}

double my_abs(double x)
{
    if (x > 0)
        return x;
    else
        return -x;
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    for (const auto &status : msg->status_list)
    {
        if (status.status == actionlib_msgs::GoalStatus::PENDING)
        {
            goal_status = PENDING;
            // ROS_INFO("Goal is pending. Successfully planned but not yet started.");
        }
        else if (status.status == actionlib_msgs::GoalStatus::ACTIVE)
        {
            goal_status = ACTIVE;
        }
        else if (status.status == actionlib_msgs::GoalStatus::SUCCEEDED)
        {
            goal_status = SUCCEEDED;
        }
        else if (status.status == actionlib_msgs::GoalStatus::ABORTED)
        {
            goal_status = ABORTED;
        }
        else if (status.status == actionlib_msgs::GoalStatus::REJECTED)
        {
            goal_status = REJECTED;
        }
        else
        {
        }
    }
}

void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr &feedback)
{
    if(ucar_state_now.data == status_goingto_areaA)
    {
        flag_dp_now.data = video_over;
        flag_dp_pub.publish(flag_dp_now);
    }
}

void playing(int voice)
{
    switch(voice)
    {
        case 1:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/num_1.wav");
            ROS_INFO("恐怖分子数量为一个");
            break;
        case 2:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/num_2.wav");
            ROS_INFO("恐怖分子数量为两个");
            break;
        case 3:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/num_3.wav");
            ROS_INFO("恐怖分子数量为三个");
            break;
        case 4:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/Baton.wav");
            ROS_INFO("我已取到警棍");
            break;
        case 5:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/cloth.wav");
            ROS_INFO("我已取到防弹衣");
            break;
        case 6:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/tear.wav");
            ROS_INFO("我已取到催泪瓦斯");
            break;
        case 9:
            system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/aid.wav");
            ROS_INFO("我已取到急救包");
            break;
        default:
            break;
    }
}

void pic_CB(const std_msgs::Int8 &pic)
{
    cv_mode = pic;
    if (ucar_state_now.data != status_arrived_areaA)
    {
        if (cv_mode.data != 0 && get_pic_flag == 0)
        {
            get_pic_flag = 1;
            voice_to_play = cv_mode.data;
            cv_mode.data = 0;
        }
    }
}

void Dis_cv_control()
{
    if (!get_pic_flag)
    {
        time_count++;
        if (mid_dis < 0.2 && mid_dis > 0)
        {
            fake_vel_now.linear.x = -0.6;
            fake_vel_now.angular.z = -turn_vel_J;
        }
        else
        {
            fake_vel_now.linear.x = 0;
            fake_vel_now.angular.z = -turn_vel_J;
        }
        if (turn_vel_J * time_count * 0.02 > 6.0)
        {
            stop_flag = 1;
            time_count = 0;
        }
    }
    else
    {
        // turn_to_angle(now_yaw-cv_yaw,-turn_vel_J);
        fake_vel_now.linear.x = 0;
        fake_vel_now.angular.z = 0;
        get_terrorist_success = 1; 
        stop_flag = 2;
    }
    cmd_vel_pub.publish(fake_vel_now);
}

void GOfix()                                         ////////////////////////////////////////////////////////////////////////////
{
    ros::Rate rate(50); // 50Hz
    while(ros::ok() && mid_dis > 0.4)
    {
        fake_vel_now.linear.x = 0.2;
        cmd_vel_pub.publish(fake_vel_now);
        ros::spinOnce();
        rate.sleep();
    }
    fake_vel_now.linear.x = 0;
    cmd_vel_pub.publish(fake_vel_now);
}

void rotateAndStop(double target_angle)
{
    ros::Rate rate(50); // 50Hz
    double error = target_angle - now_yaw;
    if(error > M_PI)
        error = error - 2*M_PI;
    else if(error < -M_PI)
        error = error + 2*M_PI;
    while (ros::ok() && my_abs(error) > 0.2) // 误差小于0.01时停�?????
    {
        error = target_angle - now_yaw;
        if(error > M_PI)
            error = error - 2*M_PI;
        else if(error < -M_PI)
            error = error + 2*M_PI;
        fake_vel_now.angular.z = 1.5*error; // 简单的比例控制
        cmd_vel_pub.publish(fake_vel_now);
        ros::spinOnce();
        rate.sleep();
    }
    last_num_turn = num_turn;
    // 停止旋转
    fake_vel_now.angular.z = 0;
    cmd_vel_pub.publish(fake_vel_now); 
    
    // flag_dp_now.data = cv_area_get;
    // flag_dp_pub.publish(flag_dp_now);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "control_test");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

    init_param_goal();
    ucar_state_now.data = status_waitting_for_awake;
    if(only_line)
    {
        ucar_state_now.data = status_arrived_areaA;
    }
    
    cv_mode.data = 0;

    geometry_msgs::PoseWithCovarianceStamped init_pose;
    geometry_msgs::Twist vel_cmd;

    /*开一个定时器用来发送速度信息*/
    timer1 = n.createTimer(ros::Duration(0.02), directionControlLoopCB); // 50hz

    /*订阅话题*/
    ros::Subscriber flag_sub = n.subscribe("/xf_mic_asr_offline/awake_flag_topic", 1, flag_Callback);
    fake_vel_sub = n.subscribe("/fake_vel", 1, fake_velCB);
    ros::Subscriber yaw_sub = n.subscribe("/yaw_fromscan", 1, yaw_CB);
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 100, poseCB);
    cv_mode_sub = n.subscribe("/pic_data", 100, pic_CB);
    cv_position_sub = n.subscribe("/pic_position", 150, pic_positionCB);
    ros::Subscriber sub = n.subscribe("/move_base/status", 1, statusCallback);
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, &LidarCallback);

    /*发布话题*/
    flag_dp_pub = n.advertise<std_msgs::Int32>("/flag2", 100);              //图像
    flag_config_pub = n.advertise<std_msgs::Int32>("/flag1", 100);          //参数
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1); // 速度信息
    init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
    pub_ClearCostmap = n.advertise<std_msgs::String>("/flag_Clear", 5); // 发布清除costmap的信�????????????????
    // mapclean_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");


    // std_srvs::Empty srv;
    Client ac("move_base", true);

    if (!ac.waitForServer(ros::Duration(6)))
    {
        ROS_ERROR("Can't connected to move base server");
        assert(0);
    }
    while (ros::ok())
    {
        if (!ac.isServerConnected())
        {
            ROS_WARN("Move_Base Actionlib Server Disonnected");
            if (!ac.waitForServer(ros::Duration(6)))
            {
                ROS_ERROR("Can't connected to move base server");
                assert(0);
            }
        }
        switch (ucar_state_now.data)
        {
            case status_origin:
                if (!goal_received_situation) 
                {
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = pose_areaB;
                    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                }
                else
                {
                    flag_config_now.data = config_1;
                    flag_config_pub.publish(flag_config_now);
                    ucar_state_now.data = status_goingto_areaB;
                    goal_received_judge = 1;
                }
                break;

            case status_arrived_areaB:
                
                if (goal_received_judge == 1)
                {
                    goal_received_situation = 0;
                    goal_received_judge = 0;
                    flag_config_now.data = config_2;
                    flag_config_pub.publish(flag_config_now);  
                }

                if (get_pic_flag)
                {
                    if (!goal_received_situation)
                    {
                        
                        if (clear_map_B == 0)
                        {
                            for(int i = 0;i<10;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapB");
                            }
                            playing(voice_to_play);
                            clear_map_B = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = pose_areaR;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
                    }
                    else
                    {
                        
                        ucar_state_now.data = status_goingto_areaR; 
                        cv_mode.data = 0;
                        //                       x_cv.data = 0; //x_cv要置0
                        get_pic_flag = 0;
                        goal_received_judge = 1;
                    }
                }
                else
                {
                    flag_dp_now.data = cv_area_get;
                    flag_dp_pub.publish(flag_dp_now);
                }

                break;

            case status_arrived_areaR:
                
                if (!stop_flag)
                {
                    if(!get_pic_flag)
                    {
                        if(num_turn != last_num_turn)
                        {
                            ROS_WARN("1111111111");
                            rotateAndStop(goal_angle_R[num_turn]);
                            ROS_WARN("2222222222");
                        }
                        if(time_count < sleep_count)
                        {
                            time_count++;
                            flag_dp_now.data = cv_area_get;
                            flag_dp_pub.publish(flag_dp_now);
                        }
                        else
                        {
                            time_count = 0;
                            num_turn++;
                            ROS_WARN("33333333333 %d %d",num_turn,last_num_turn);
                        }
                        if(num_turn >= R_num)
                        {
                            num_turn = 0;
                            stop_flag = 1;
                        }
                    }
                    else
                    {
                        num_turn = 0;
                        time_count = 0;
                        last_num_turn = -1;
                        get_terrorist_success = 1;
                        stop_flag = 2;
                    } 
                }
                else if (stop_flag == 2)
                {
                    if (!isone)
                    {
                        flag_dp_now.data = cv_area_get;
                        flag_dp_pub.publish(flag_dp_now);
                    }
                    else
                    {
                        if (goal_received_judge == 1)
                        {
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                            
                        }
                        if (!goal_received_situation)                           //前往识别�???
                        {
                            if (clear_map_R == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapR");
                                }
                                flag_config_now.data = config_3;
                                flag_config_pub.publish(flag_config_now);
                                clear_map_R = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_area_GO;
                            ROS_WARN("R send the goal: %f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,cv_yaw);

                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            if (goal_status == ABORTED || goal_status == REJECTED)
                            {
                                ROS_INFO("ABORTED or REJECTED");
                                pose_area_GO.position.x = replandeal(px, pose_area_GO.position.x);
                                pose_area_GO.position.y = replandeal(py, pose_area_GO.position.y);
                                ROS_INFO("now: %f %f %f------", pose_area_GO.position.x, pose_area_GO.position.y,now_yaw);
                                goal_received_judge = 1;
                            }
                            else if (goal_status == SUCCEEDED)
                            {
                                ROS_INFO("SUCCESS arive GO");
                                ucar_state_now.data = status_arrived_areaGO;
                                // stop_flag = 1;
                                cv_mode.data = 0;
                                get_pic_flag = 0;
                                goal_received_judge = 1;
                            }
                        }
                    }
                }
                else if (stop_flag == 1)
                {
                    if (goal_received_judge == 1)
                    {
                        goal_received_situation = 0;
                        goal_received_judge = 0;
                    }
                    if (!goal_received_situation)
                    {
                        if (clear_map_R == 0)
                        {
                            for(int i = 0;i<5;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapR");
                            }
                            flag_config_now.data = config_4;
                            flag_config_pub.publish(flag_config_now);
                            clear_map_R = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        if (get_terrorist_success == 1)
                            goal.target_pose.pose = pose_areaJ;
                        else
                            goal.target_pose.pose = pose_areaJ_1;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                    }
                    else
                    {
                        if (goal_status == ABORTED || goal_status == REJECTED)
                        {
                            ROS_INFO("ABORTED or REJECTED");
                            pose_areaJ_1.position.x = replandeal(pose_areaR.position.x, pose_areaJ_1.position.x);
                            pose_areaJ_1.position.y = replandeal(pose_areaR.position.y, pose_areaJ_1.position.y);
                            ROS_INFO("now: %f %f %f------", pose_areaJ_1.position.x, pose_areaJ_1.position.y,now_yaw);
                            goal_received_judge = 1;
                        }
                        else if (goal_status == SUCCEEDED)
                        {
                            if (get_terrorist_success == 1)
                            {
                                ucar_state_now.data = status_arrived_areaJ; 
                            }
                            else
                            {
                                ucar_state_now.data = status_arrived_areaJ_1; 
                            }
                            cv_mode.data = 0;
                            stop_flag = 0;
                            get_pic_flag = 0;
                            goal_received_judge = 1;
                        }
                    }
                }
                break;

            case status_arrived_areaJ_1:
                if (!stop_flag)
                {
                    if(!get_pic_flag)
                    {
                        if(num_turn != last_num_turn)
                        {
                            ROS_WARN("1111111111");
                            rotateAndStop(goal_angle_J_1[num_turn]);
                            ROS_WARN("2222222222");
                        }
                        // Dis_cv_control();
                        if(time_count < sleep_count)
                        {
                            time_count++;
                            flag_dp_now.data = cv_area_get;
                            flag_dp_pub.publish(flag_dp_now);
                        }
                        else
                        {
                            time_count = 0;
                            num_turn++;
                            ROS_WARN("33333333333 %d %d",num_turn,last_num_turn);
                        }
                        if(num_turn >= J_1_num)
                        {
                            num_turn = 0;
                            stop_flag = 1;
                        }
                    }
                    else
                    {
                        get_terrorist_success = 1;
                        stop_flag = 2;
                    } 
                }
                else if (stop_flag == 2)
                {
                    if(!isone)
                    {
                        flag_dp_now.data = cv_area_get;
                        flag_dp_pub.publish(flag_dp_now);
                    }
                    else
                    {
                        if (goal_received_judge == 1)
                        {
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                            
                        }
                        if (!goal_received_situation)                   //前往识别�???
                        {
                            if (clear_map_J_1 == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapJ_1");
                                }
                                flag_config_now.data = config_3;
                                flag_config_pub.publish(flag_config_now);
                                clear_map_J_1 = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_area_GO;
                            ROS_WARN("J_1 send the goal: %f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,cv_yaw);

                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            if (goal_status == ABORTED || goal_status == REJECTED)
                            {
                                ROS_INFO("ABORTED or REJECTED");
                                pose_area_GO.position.x = replandeal(px, pose_area_GO.position.x);
                                pose_area_GO.position.y = replandeal(py, pose_area_GO.position.y);
                                ROS_INFO("now: %f %f %f------", pose_area_GO.position.x, pose_area_GO.position.y,now_yaw);
                                goal_received_judge = 1;
                            }
                            else if (goal_status == SUCCEEDED)
                            {
                                ROS_INFO("SUCCESS arive GO");
                                // stop_flag = 1;
                                ucar_state_now.data = status_arrived_areaGO;
                                cv_mode.data = 0;
                                get_pic_flag = 0;
                                goal_received_judge = 1;
                            }
                        }
                    }
                }
                else if (stop_flag == 1) 
                {
                    if (goal_received_judge == 1)
                    {
                        goal_received_situation = 0;
                        goal_received_judge = 0;
                    }
                    if (!goal_received_situation)
                    {
                        if (clear_map_J_1 == 0)
                        {
                            for(int i = 0;i<5;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapJ_1");
                            }
                            flag_config_now.data = config_4;
                            flag_config_pub.publish(flag_config_now);
                            clear_map_J_1 = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        if (get_terrorist_success == 1)
                            goal.target_pose.pose = pose_areaJ;
                        else
                            goal.target_pose.pose = pose_areaJ_3;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                    }
                    else
                    {
                        if (goal_status == ABORTED || goal_status == REJECTED)
                        {
                            ROS_INFO("ABORTED or REJECTED");
                            pose_areaJ_3.position.x = replandeal(pose_areaJ_1.position.x, pose_areaJ_3.position.x);
                            pose_areaJ_3.position.y = replandeal(pose_areaJ_1.position.y, pose_areaJ_3.position.y);
                            ROS_INFO("now: %f %f %f------", pose_areaJ_3.position.x, pose_areaJ_3.position.y,now_yaw);
                            goal_received_judge = 1;
                        }
                        else if (goal_status == SUCCEEDED)
                        {
                            if (get_terrorist_success == 1)
                            {
                                ucar_state_now.data = status_arrived_areaJ; 
                            }
                            else
                            {
                                ucar_state_now.data = status_arrived_areaJ_3; 
                            }
                            cv_mode.data = 0;
                            stop_flag = 0;
                            get_pic_flag = 0;
                            goal_received_judge = 1;
                        }
                    }
                }
                break;

            case status_arrived_areaJ_2:
                if (!stop_flag)
                {
                    if(!get_pic_flag)
                    {
                        if(num_turn != last_num_turn)
                        {
                            ROS_WARN("1111111111");
                            rotateAndStop(goal_angle_J_2[num_turn]);
                            ROS_WARN("2222222222");
                        }
                        // Dis_cv_control();
                        if(time_count < sleep_count)
                        {
                            time_count++;
                            flag_dp_now.data = cv_area_get;
                            flag_dp_pub.publish(flag_dp_now);
                        }
                        else
                        {
                            time_count = 0;
                            num_turn++;
                            ROS_WARN("33333333333 %d %d",num_turn,last_num_turn);
                        }
                        if(num_turn >= J_2_num)
                        {
                            num_turn = 0;
                            stop_flag = 1;
                        }
                    }
                    else
                    {
                        get_terrorist_success = 1;
                        stop_flag = 2;
                    } 
                }
                else if (stop_flag == 2)
                {
                    if (!isone)
                    {
                        flag_dp_now.data = cv_area_get;
                        flag_dp_pub.publish(flag_dp_now);
                    }
                    else
                    {
                        if (goal_received_judge == 1)
                        {
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                        }
                        if (!goal_received_situation)
                        {
                            if (clear_map_J_2 == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapJ_2");
                                }
                                flag_config_now.data = config_3;
                                flag_config_pub.publish(flag_config_now);
                                clear_map_J_2 = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_area_GO;
                            ROS_WARN("J_2 send the goal: %f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,cv_yaw);

                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            if (goal_status == ABORTED || goal_status == REJECTED)
                            {
                                ROS_INFO("ABORTED or REJECTED");
                                pose_area_GO.position.x = replandeal(px, pose_area_GO.position.x);
                                pose_area_GO.position.y = replandeal(py, pose_area_GO.position.y);
                                ROS_INFO("now: %f %f %f------", pose_area_GO.position.x, pose_area_GO.position.y,now_yaw);
                                goal_received_judge = 1;
                            }
                            else if (goal_status == SUCCEEDED)
                            {
                                ROS_INFO("SUCCESS arive GO");
                                ucar_state_now.data = status_arrived_areaGO;
                                // stop_flag = 1;
                                cv_mode.data = 0;
                                get_pic_flag = 0;
                                goal_received_judge = 1;
                            }
                        }
                    }
                }
                else if (stop_flag == 1)
                {
                    if (goal_received_judge == 1)
                    {
                        goal_received_situation = 0;
                        goal_received_judge = 0;
                    }
                    if (!goal_received_situation)
                    {
                        if (clear_map_J_2 == 0)
                        {
                            for(int i = 0;i<5;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapJ_2");
                            }
                            flag_config_now.data = config_5;
                            flag_config_pub.publish(flag_config_now);
                            clear_map_J_2 = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = pose_areaA;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                    }
                    else
                    {
                        ucar_state_now.data = status_goingto_areaA; 
                        cv_mode.data = 0;
                        get_pic_flag = 0;
                        goal_received_judge = 1;
                    }
                }
                break;

            case status_arrived_areaJ_3:
                if (!stop_flag)
                {
                    if(!get_pic_flag)
                    {
                        if(num_turn != last_num_turn)
                        {
                            ROS_WARN("1111111111");
                            rotateAndStop(goal_angle_J_3[num_turn]);
                            ROS_WARN("2222222222");
                        }
                        if(time_count < sleep_count)
                        {
                            time_count++;
                            flag_dp_now.data = cv_area_get;
                            flag_dp_pub.publish(flag_dp_now);
                        }
                        else
                        {
                            time_count = 0;
                            num_turn++;
                            ROS_WARN("33333333333 %d %d",num_turn,last_num_turn);
                        }
                        if(num_turn >= J_3_num)
                        {
                            num_turn = 0;
                            stop_flag = 1;
                        }
                    }
                    else
                    {
                        get_terrorist_success = 1;
                        stop_flag = 2;
                    } 
                }
                else if (stop_flag == 2)
                {
                    if (!isone)
                    {
                        flag_dp_now.data = cv_area_get;
                        flag_dp_pub.publish(flag_dp_now);
                    }
                    else
                    {
                        if (goal_received_judge == 1)
                        {
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                            
                        }
                        if (!goal_received_situation)
                        {
                            if (clear_map_J_3 == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapJ_3");
                                }
                                flag_config_now.data = config_3;
                                flag_config_pub.publish(flag_config_now);
                                clear_map_J_3 = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_area_GO;
                            ROS_WARN("J_3 send the goal: %f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,cv_yaw);
                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            if (goal_status == ABORTED || goal_status == REJECTED)
                            {
                                ROS_INFO("ABORTED or REJECTED");
                                pose_area_GO.position.x = replandeal(px, pose_area_GO.position.x);
                                pose_area_GO.position.y = replandeal(py, pose_area_GO.position.y);
                                ROS_INFO("now: %f %f %f------", pose_area_GO.position.x, pose_area_GO.position.y,now_yaw);
                                goal_received_judge = 1;
                            }
                            else if (goal_status == SUCCEEDED)
                            {
                                ROS_INFO("SUCCESS arive GO");
                                ucar_state_now.data = status_arrived_areaGO;
                                // stop_flag = 1;
                                cv_mode.data = 0;
                                get_pic_flag = 0;
                                goal_received_judge = 1;
                            }
                        }
                    }
                }
                else if (stop_flag == 1) 
                {
                    if (goal_received_judge == 1)
                    {
                        goal_received_situation = 0;
                        goal_received_judge = 0;
                    }
                    if (!goal_received_situation)
                    {
                        if (clear_map_J_3 == 0)
                        {
                            for(int i = 0;i<5;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapJ_3");
                            }
                            flag_config_now.data = config_4;
                            flag_config_pub.publish(flag_config_now);
                            clear_map_J_3 = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = pose_areaJ;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                    }
                    else
                    {
                        if (goal_status == SUCCEEDED)
                        {
                            ucar_state_now.data = status_arrived_areaJ; 
                            cv_mode.data = 0;
                            stop_flag = 0;
                            get_pic_flag = 0;
                            goal_received_judge = 1;
                        }
                    
                    }
                }
                break;
            
            case status_arrived_areaGO:
                // if (goal_received_judge == 1)
                // {
                //     ROS_WARN("FFFFFFFFFFFFFF %d",need_twice);
                //     goal_received_situation = 0;
                //     goal_received_judge = 0;
                //                                                                 /////////////////////////////////////////////////////////////////////////    
                // }
                if(need_twice)
                {
                    need_twice = 2;
                    if (isone != 2)
                    {
                        flag_dp_now.data = cv_area_get;
                        flag_dp_pub.publish(flag_dp_now);
                    }
                    else
                    {
                        if (goal_received_judge == 1)
                        {
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                        }
                        if (!goal_received_situation)
                        {
                            if (clear_map_GO == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapGO");
                                }
                                clear_map_GO = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_area_GO2;
                            ROS_WARN("GO send the goal: %f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,cv_yaw);

                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            if (goal_status == ABORTED || goal_status == REJECTED)
                            {
                                ROS_INFO("ABORTED or REJECTED");
                                pose_area_GO2.position.x = replandeal(px, pose_area_GO2.position.x);
                                pose_area_GO2.position.y = replandeal(py, pose_area_GO2.position.y);
                                ROS_INFO("now: %f %f %f------", pose_area_GO2.position.x, pose_area_GO2.position.y,now_yaw);
                                goal_received_judge = 1;
                            }
                            else if (goal_status == SUCCEEDED)
                            {
                                ROS_INFO("SUCCESS arive GO");
                                ucar_state_now.data = status_arrived_areaGO2;
                                need_twice = 2;
                                // stop_flag = 1;
                                cv_mode.data = 0;
                                get_pic_flag = 0;
                                goal_received_judge = 1;
                            }
                        }
                    }
                }
                else
                {
                    if (isone == 1)
                    {                                       
                        if (goal_received_judge == 1)
                        {
                            GOfix();                                            /////////////////////////////////////////////////////////////////////////
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                            if(!need_twice)                                     /////////////////////////////////////////////////////////////////////////
                                playing(voice_to_play);                         /////////////////////////////////////////////////////////////////////////
                        }
                        if (!goal_received_situation)
                        {
                            if (clear_map_GO == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapGO");
                                }
                                flag_config_now.data = config_4;
                                if(special_flag)
                                {
                                    flag_config_now.data = config_5;
                                }
                                flag_config_pub.publish(flag_config_now); 
                                clear_map_GO = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_areaJ;
                            if(special_flag)
                            {
                                goal.target_pose.pose = pose_areaA;
                            }
                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            
                            ucar_state_now.data = status_goingto_areaJ; 
                            if(special_flag)
                                ucar_state_now.data = status_goingto_areaA; 
                            cv_mode.data = 0;
                            get_pic_flag = 0;
                            goal_received_judge = 1;
                        }
                    }
                }
            
                break;

            case status_arrived_areaGO2:
                if (goal_received_judge == 1)
                {
                    goal_received_situation = 0;
                    goal_received_judge = 0;
                    playing(voice_to_play);
                }
                if (!goal_received_situation)
                {
                    if (clear_map_GO2 == 0) 
                    {
                        for(int i = 0;i<5;i++)
                        {
                            clearcostmap.data = "true";
                            pub_ClearCostmap.publish(clearcostmap);
                            ROS_INFO("already clear the costmapGO2");
                        }
                        flag_config_now.data = config_4;
                        flag_config_pub.publish(flag_config_now); 
                        clear_map_GO2 = 1;
                    }
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();
                    goal.target_pose.pose = pose_areaJ;
                    if(special_flag)
                        goal.target_pose.pose = pose_areaA;
                    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                }
                else
                {
                    
                    ucar_state_now.data = status_goingto_areaJ; 
                    if(special_flag)
                        ucar_state_now.data = status_goingto_areaA; 
                    cv_mode.data = 0;
                    get_pic_flag = 0;
                    goal_received_judge = 1;
                }
            
                break;

            case status_arrived_areaJ: 
                
                if (goal_received_judge == 1)
                {
                    goal_received_situation = 0;
                    goal_received_judge = 0;
                    playing(9); 
                }
                if (get_terrorist_success)
                {
                    if (!goal_received_situation)
                    {
                        if (clear_map_J == 0) 
                        {
                            for(int i = 0;i<5;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapJ");
                            }
                            flag_config_now.data = config_5;
                            flag_config_pub.publish(flag_config_now);
                            clear_map_J = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = pose_areaA;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); 
                    }
                    else
                    {
                        ucar_state_now.data = status_goingto_areaA; 
                        cv_mode.data = 0;
                        get_pic_flag = 0;
                        goal_received_judge = 1;
                    }
                }
                else
                {
                    ucar_state_now.data = status_arrived_areaGO3; 
                    cv_mode.data = 0;
                    get_pic_flag = 0;
                    special_flag = 1;
                    goal_received_judge = 1;
                }
                break;

            case status_arrived_areaGO3:
                if (!stop_flag)
                {
                    if(!get_pic_flag)
                    {
                        if(num_turn != last_num_turn)
                        {
                            ROS_WARN("1111111111");
                            rotateAndStop(goal_angle_J[num_turn]);
                            ROS_WARN("2222222222");
                        }
                        if(time_count < sleep_count)
                        {
                            time_count++;
                            flag_dp_now.data = cv_area_get;
                            flag_dp_pub.publish(flag_dp_now);
                        }
                        else
                        {
                            time_count = 0;
                            num_turn++;
                            ROS_WARN("33333333333 %d %d",num_turn,last_num_turn);
                        }
                        if(num_turn >= J_num)
                        {
                            num_turn = 0;
                            stop_flag = 1;
                        }
                    }
                    else
                    {
                        get_terrorist_success = 1;
                        stop_flag = 2;
                    } 
                    
                }
                else if (stop_flag == 2)
                {
                    if (!isone)
                    {
                        flag_dp_now.data = cv_area_get;
                        flag_dp_pub.publish(flag_dp_now);
                    }
                    else
                    {
                        if (goal_received_judge == 1)
                        {
                            goal_received_situation = 0;
                            goal_received_judge = 0;
                        }
                        if (!goal_received_situation)
                        {
                            if (clear_map_J == 0)
                            {
                                for(int i = 0;i<5;i++)
                                {
                                    clearcostmap.data = "true";
                                    pub_ClearCostmap.publish(clearcostmap);
                                    ROS_INFO("already clear the costmapJ");
                                }
                                flag_config_now.data = config_3;
                                flag_config_pub.publish(flag_config_now);
                                clear_map_J = 1;
                            }
                            goal.target_pose.header.frame_id = "map";
                            goal.target_pose.header.stamp = ros::Time::now();
                            goal.target_pose.pose = pose_area_GO;
                            ROS_WARN("GO3 send the goal: %f,%f,%f",goal.target_pose.pose.position.x,goal.target_pose.pose.position.y,cv_yaw);
                            ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                        }
                        else
                        {
                            if (goal_status == ABORTED || goal_status == REJECTED)
                            {
                                ROS_INFO("ABORTED or REJECTED");
                                pose_area_GO.position.x = replandeal(px, pose_area_GO.position.x);
                                pose_area_GO.position.y = replandeal(py, pose_area_GO.position.y);
                                ROS_INFO("now: %f %f %f------", pose_area_GO.position.x, pose_area_GO.position.y,now_yaw);
                                goal_received_judge = 1;
                            }
                            else if (goal_status == SUCCEEDED)
                            {
                                ROS_INFO("SUCCESS arive GO");
                                ucar_state_now.data = status_arrived_areaGO;
                                // stop_flag = 1;
                                cv_mode.data = 0;
                                get_pic_flag = 0;
                                goal_received_judge = 1;
                            }
                        }
                    }
                }
                else if (stop_flag == 1) 
                {

                    if (goal_received_judge == 1)
                    {
                        goal_received_situation = 0;
                        goal_received_judge = 0;
                    }
                    if (!goal_received_situation)
                    {
                        if (clear_map_J == 0)
                        {
                            for(int i = 0;i<5;i++)
                            {
                                clearcostmap.data = "true";
                                pub_ClearCostmap.publish(clearcostmap);
                                ROS_INFO("already clear the costmapJ");
                            }
                            flag_config_now.data = config_4;
                            flag_config_pub.publish(flag_config_now);
                            clear_map_J = 1;
                        }
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = pose_areaJ_2;
                        ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                    }
                    else
                    {

                        if (goal_status == ABORTED || goal_status == REJECTED)
                        {
                            ROS_INFO("ABORTED or REJECTED");
                            pose_areaJ_2.position.x = replandeal(pose_areaJ.position.x, pose_areaJ_2.position.x);
                            pose_areaJ_2.position.y = replandeal(pose_areaJ.position.y, pose_areaJ_2.position.y);
                            ROS_INFO("now: %f %f %f------", pose_areaJ_2.position.x, pose_areaJ_2.position.y,now_yaw);
                            goal_received_judge = 1;
                        }
                        else if (goal_status == SUCCEEDED)
                        {
                            ucar_state_now.data = status_arrived_areaJ_2; 
                            cv_mode.data = 0;
                            stop_flag = 0;
                            get_pic_flag = 0;
                            goal_received_judge = 1;
                        }
                    }
                }
                break;

            case status_arrived_areaA:
                
                if(use_slam)
                {
                    system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/finish.wav");
                    ROS_INFO("已完成人质营救工作，请快速增派支援进行人质救援");
                }
                else
                {
                    if(num_turn != last_num_turn)
                    {
                        rotateAndStop(-3.12);
                        ROS_WARN("AAAAAAAAAAAAA");
                    }
                    flag_dp_now.data = cv_line;
                    flag_dp_pub.publish(flag_dp_now);
                }
                if(shutdown_flag == 0)
                {
                    // shutdownNode("/get_pic_FL");
                    shutdownNode("/lidar_undistortion_node");
                    shutdownNode("/ucar_nav_zhidaowuzhangaijiasu_7_6_gs2wdm");
                    shutdownNode("/xf_asr_offline_node");
                    shutdownNode("/ydlidar_node");
                    shutdownNode("/amcl");
                    shutdownNode("/map_server");
                    shutdownNode("/ucar_navigation_copy");
                    shutdown_flag = 1;
                }
                break;

            default:
                break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

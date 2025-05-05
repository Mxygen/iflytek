#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
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
#include <nav_msgs/Odometry.h>
using namespace std;
//当前置停车点可以识别时，取false
bool need_two_goal_for_parkingE = true; //需要两个停车点，区域E
// bool need_two_goal_for_parkingD = false; //需要两个停车点，区域D
bool need_two_goal_for_parkingC = false; //需要两个停车点，区域C
bool need_two_goal_for_parkingB = false; //需要两个停车点，区域B
int areaE_flag = 0;
int areaD_flag = 0;
int areaC_flag = 0;
int areaB_flag = 0;
int areaF_flag = 0;
/*****************************************************
*初始化区域目标点
*****************************************************/
double goal_areaE_1_x = 2.429;
double goal_areaE_1_y = -0.0661;
double goal_areaE_1_z = -0.466957;
double goal_areaE_2_x = 2.798;
double goal_areaE_2_y = -2.4216;
double goal_areaE_2_z = 1.975928;

double goal_areaD_1_x = 2.8627;
double goal_areaD_1_y = -4.0899;
double goal_areaD_1_z = -1.996575;

double goal_areaC_1_x = 5.216;//4.816
double goal_areaC_1_y = -1.9062;//-1.5562
double goal_areaC_1_z = -2.628430;//-2.628430
/*double goal_areaC_2_x = 4.79;
double goal_areaC_2_y = -0.25;
double goal_areaC_2_z = -2.628430;*/

double goal_areaB_1_x = 5.216;//4.9022
double goal_areaB_1_y = -3.2546;//-3.9846
double goal_areaB_1_z = 2.965937;//2.965937
/*double goal_areaB_2_x = 4.79;
double goal_areaB_2_y = -5;
double goal_areaB_2_z = 2.965937;*/
/*坡道注释
double goal_ramp_1_x = 0.9675;
double goal_ramp_1_y = -0.60;
double goal_ramp_1_z = -1.6;
*/
/*
double goal_ramp_2_x = 0.9675;
double goal_ramp_2_y = -0.65;
double goal_ramp_2_z = 1.6;
*/
double goal_areaF_1_x = 0.9675;
double goal_areaF_1_y = -2.5;//-3.5
double goal_areaF_1_z = 2.40;
double goal_areaF_2_x = 0.9675;
double goal_areaF_2_y = -4.75;
double goal_areaF_2_z = 0.785;
/*double goal_areaF_3_x = 1.65;
double goal_areaF_3_y = -4.75;
double goal_areaF_3_z = 2.355;*/

double goal_final_x = -0.039747;
double goal_final_y = 0.023227;
double goal_final_z = 0.00242+3.14159;

/*****************************************************
*宏定义标志位
*每个区域停车点预留标志位：0~30
*每个区域摄像头预留标志位：31~50
*其他模块可使用预留标志位：51~70
*每个区域转向点预留标志位：-1~-50
*****************************************************/
#define  status_origin  0

#define  status_goingto_areaE  1
#define  status_arrived_areaE  2
#define  status_goingto_areaE_second  3
#define  status_arrived_areaE_second  4

#define  status_goingto_areaD  5
#define  status_arrived_areaD  6
#define  status_arrived_areaD_last  -13

#define  status_goingto_areaC  7
#define  status_arrived_areaC  8
#define  status_arrived_areaC_last  -23

#define  status_goingto_areaB  9
#define  status_arrived_areaB  10
#define  status_arrived_areaB_last  -33
/*
#define  status_goingto_ramp_1  22
#define  status_arrived_ramp_1  23
#define  status_goingto_ramp_2  24
#define  status_arrived_ramp_2  25
*/
#define  status_goingto_areaF  11
#define  status_arrived_areaF  12
#define  status_arrived_areaF_last  -67

#define  status_goingto_final_place  13
#define  status_arrived_final_place  14

/*#define  status_goingto_areaC_second  15
#define  status_arrived_areaC_second  16
#define  status_goingto_areaB_second  17
#define  status_arrived_areaB_second  18*/

#define  status_goingto_areaF_second  19
#define  status_arrived_areaF_second  20
// #define  status_goingto_areaF_third   21
// #define  status_arrived_areaF_third   22

#define cv_mode_areaE_1_photo_get_finish 31
#define cv_mode_areaE_1_photo_get_finish_but_no_result 32
#define cv_mode_areaD_photo_get_finish 33
#define cv_mode_areaD_photo_get_finish_but_no_result 34
#define cv_mode_areaC_photo_get_finish 35
#define cv_mode_areaC_photo_get_finish_but_no_result 36
#define cv_mode_areaB_photo_get_finish 37
#define cv_mode_areaB_photo_get_finish_but_no_result 38
#define cv_mode_areaE_2_photo_get_finish 39
#define cv_mode_areaE_2_photo_get_finish_but_no_result 40
#define cv_mode_areaF_photo_get_finish 41
#define cv_mode_areaF_photo_get_finish_but_no_result 42
#define cv_mode_areaC_second_photo_get_finish 43
#define cv_mode_areaC_second_photo_get_finish_but_no_result 44
#define cv_mode_areaB_second_photo_get_finish 45
#define cv_mode_areaB_second_photo_get_finish_but_no_result 46
#define cv_mode_areaF_second_photo_get_finish 47
#define cv_mode_areaF_second_photo_get_finish_but_no_result 48
//#define cv_mode_areaF_third_photo_get_finish 49
//#define cv_mode_areaF_third_photo_get_finish_but_no_result 50

#define area_identification_start  51//表示开始房间识别
#define status_all_finished  52//表示完成房间识别，开始播报
#define status_waitting_for_awake 53

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;
typedef actionlib::SimpleClientGoalState State;
State::StateEnum state;
ros::ServiceClient mapclean_client;
geometry_msgs::Pose pose_init;
move_base_msgs::MoveBaseGoal goal;
std_srvs::Empty srv;
std_msgs::String clearcostmap;
ros::Publisher vel_pub;
ros::Publisher init_pub;
ros::Publisher ucar_state_pub ,ucar_state_for_cv_pub,start_area_cv_pub,end_area_cv_pub, true_area_cv_pub,cv_mode_pub;
ros::Publisher pub_ClearCostmap; //costmap2d里接受

geometry_msgs::Pose pose_areaE;
geometry_msgs::Pose pose_areaE_2;
geometry_msgs::Pose pose_areaD;
geometry_msgs::Pose pose_areaC;
//geometry_msgs::Pose pose_areaC_2;
geometry_msgs::Pose pose_areaB;
//geometry_msgs::Pose pose_areaB_2;

//geometry_msgs::Pose pose_ramp_1;
geometry_msgs::Pose pose_areaF;
geometry_msgs::Pose pose_areaF_2;
//geometry_msgs::Pose pose_areaF_3;
//geometry_msgs::Pose pose_ramp_2;
geometry_msgs::Pose pose_final;
ros::Publisher flag_dp_pub;
std_msgs::Int32 flag_dp_now;
int flag_start_cv_E1 = 0,flag_enter_cv_E1 = 0,flag_end_cv_E1 = 0,flag_tend_cv_E1=0;//tend = ture end
int flag_start_cv_E2 = 0,flag_enter_cv_E2 = 0,flag_end_cv_E2 = 0,flag_tend_cv_E2=0;
int flag_start_cv_D = 0,flag_enter_cv_D = 0,flag_end_cv_D = 0,flag_tend_cv_D=0;
int flag_start_cv_C = 0,flag_enter_cv_C = 0,flag_end_cv_C = 0,flag_tend_cv_C=0;
int flag_start_cv_B = 0,flag_enter_cv_B = 0,flag_end_cv_B = 0,flag_tend_cv_B=0;
int flag_start_cv_F = 0,flag_enter_cv_F = 0,flag_end_cv_F = 0,flag_tend_cv_F=0;
int flag_dp_Edoor = 0,flag_dp_E = 0,flag_dp_aisle = 0,flag_dp_D = 0,flag_dp_C = 0,flag_dp_B = 0,flag_dp_ramp = 1,flag_dp_F = 1,flag_dp_stop = 1,flag_dp_Edoor_2 = 1,flag_dp_Fin = 1;
int flag_F_clear = 0;
int time_flag1 = 0,time_flag2=0;
int flag_bug = 0;

int clear_map_E_1 = 0,clear_map_E_2 = 0,clear_map_D = 0,clear_map_C = 0,clear_map_C2 = 0,clear_map_B = 0,clear_map_B2 = 0,clear_map_F = 0,clear_map_F2 = 0,clear_map_F3 = 0;//,clear_map_ramp_1 = 0,clear_map_ramp_2 = 0;
int areaE_1_arrived = 0;
std_msgs::Int8 ucar_state_now ,get_picture,flag_area_cv ;
std_msgs::Int8 for_cv_mode,cv_mode ,cv_mode_open_cap,area_is_sure,flag_go_E;
char goal_received_situation=0;
char already_awake=0;
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
	m_quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, _angleRan); //这是一个欧拉角转四元素，车子只有Yaw这个自由度
	return m_quaternion;
}
/*****************************************************
*目的地点初始化菜单
*****************************************************/
void init_goalList()
{
	pose_areaE.position = setPoint(goal_areaE_1_x,goal_areaE_1_y,0);
	pose_areaE.orientation = setQuaternion(goal_areaE_1_z);
	pose_areaE_2.position = setPoint(goal_areaE_2_x,goal_areaE_2_y,0);
	pose_areaE_2.orientation = setQuaternion(goal_areaE_2_z);

	pose_areaD.position = setPoint(goal_areaD_1_x,goal_areaD_1_y,0);
	pose_areaD.orientation = setQuaternion(goal_areaD_1_z);

	pose_areaC.position = setPoint(goal_areaC_1_x,goal_areaC_1_y,0);
	pose_areaC.orientation = setQuaternion(goal_areaC_1_z);
	/*pose_areaC_2.position = setPoint(goal_areaC_2_x,goal_areaC_2_y,0);
	pose_areaC_2.orientation = setQuaternion(goal_areaC_2_z);*/

	pose_areaB.position = setPoint(goal_areaB_1_x,goal_areaB_1_y,0);
	pose_areaB.orientation = setQuaternion(goal_areaB_1_z);
	/*pose_areaB_2.position = setPoint(goal_areaB_2_x,goal_areaB_2_y,0);
	pose_areaB_2.orientation = setQuaternion(goal_areaB_2_z);*/

	/*pose_ramp_1.position = setPoint(goal_ramp_1_x,goal_ramp_1_y,0);
	pose_ramp_1.orientation = setQuaternion(goal_ramp_1_z);
	pose_ramp_2.position = setPoint(goal_ramp_2_x,goal_ramp_2_y,0);
	pose_ramp_2.orientation = setQuaternion(goal_ramp_2_z);*/

	pose_areaF.position = setPoint(goal_areaF_1_x,goal_areaF_1_y,0);
	pose_areaF.orientation = setQuaternion(goal_areaF_1_z);
	pose_areaF_2.position = setPoint(goal_areaF_2_x,goal_areaF_2_y,0);
	pose_areaF_2.orientation = setQuaternion(goal_areaF_2_z);
	/*pose_areaF_3.position = setPoint(goal_areaF_3_x,goal_areaF_3_y,0);
	pose_areaF_3.orientation = setQuaternion(goal_areaF_3_z);*/

	pose_final.position = setPoint(goal_final_x,goal_final_y,0);
	pose_final.orientation = setQuaternion(goal_final_z);
}

/*****************************************************
*激光雷达回调函数
*****************************************************/
double rx = 0;
double ry = 0;
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

}
//'''E区域可能还要更改 别忘了'''
/*****************************************************
*区域位置函数
*实时接收小车当前位置
*将位置信息与标志位进行判断，用来给发布小车当前参数或摄像头状态
*flag_dp 	   动态参数标志位
*flag_start_cv 开始识别标志位
*flag_enter_cv 开始有效识别标志位，即进入该区域
*flag_end_cv   结束识别标志位，即出这个区域到进下一个区域间
*flag_tend_cv  确保结束识别标志位，即小车状态为进入下一个区域
*****************************************************/
void poseCB(const nav_msgs::Odometry &msg)
{
	nav_msgs::Odometry odom1;
	odom1 = msg ;
    tf::Pose pose_tf;
    tf::poseMsgToTF(odom1.pose.pose, pose_tf);
    double psi = tf::getYaw(pose_tf.getRotation());
	// if(odom1.pose.pose.position.x>0 && time_flag1 == 0)
	// {
	// 	ROS_INFO("STARTTTTTTTTTTTT");
	// 	time_flag1=1;
	// }
	// if(odom1.pose.pose.position.x>2 && time_flag2 == 0)
	// {
	// 	ROS_INFO("ENDDDDDDDDDDDD");
	// 	time_flag2=1;
	// }
	// if(flag_tend_cv_D == 1  && status_goingto_areaC == ucar_state_now.data && flag_enter_cv_D == 1 )
	// {
	// 	printf("！");
	// 	std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
	// 	std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
	// 	std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	// }

	/*
	预留动态参数调整位置
	*/
	if(flag_dp_Edoor==0 && odom1.pose.pose.position.x>0.65 && odom1.pose.pose.position.x<2.4 && odom1.pose.pose.position.y<0.05 && odom1.pose.pose.position.y>-0.95)
	{
		flag_dp_Edoor = 1,flag_dp_now.data=1;
		ROS_INFO("Change to the Edoor state");
		flag_dp_pub.publish(flag_dp_now);
	}
	else if(flag_dp_E==0 && odom1.pose.pose.position.x>2.4 && odom1.pose.pose.position.x<4.0 && odom1.pose.pose.position.y<0.05 && odom1.pose.pose.position.y>-2.15)
	{
		flag_dp_E = 1,flag_dp_now.data=2;
		flag_dp_ramp=0,flag_dp_F=0;
		ROS_INFO("Change to the Earea state");
		flag_dp_pub.publish(flag_dp_now);
	}
	else if(flag_dp_Edoor_2 == 0  && odom1.pose.pose.position.x>1.15 && odom1.pose.pose.position.x<3.5 && odom1.pose.pose.position.y<0.05 && odom1.pose.pose.position.y>-0.29) 
	{
		flag_dp_Edoor_2 = 1,flag_dp_now.data=1;
		ROS_INFO("Change to the Edoor state again");
		flag_dp_pub.publish(flag_dp_now);
	}
	// else if(flag_dp_E==0 && flag_dp_aisle==1 && flag_dp_B == 1 &&  odom1.pose.pose.position.x<3.15)
	// {
	// 	flag_dp_E = 1,flag_dp_now.data=2;
	// 	ROS_INFO("Change to the Earea state again");
	// 	flag_dp_pub.publish(flag_dp_now);
	// }
	else if(flag_dp_aisle==0 && odom1.pose.pose.position.x>3.05 && odom1.pose.pose.position.x<4.65 && odom1.pose.pose.position.y<-2.0 && odom1.pose.pose.position.y>-3.3)
	{
		flag_dp_aisle = 1;
		flag_dp_Edoor_2=0;
		ROS_INFO("OPEN EDOOR");
		// flag_dp_pub.publish(flag_dp_now);
	}
	// else if(flag_dp_D==0 && odom1.pose.pose.position.x>2.32 && odom1.pose.pose.position.x<3.0 && odom1.pose.pose.position.y<-3.3 && odom1.pose.pose.position.y>-5.5)
	// {
	// 	flag_dp_D = 1,flag_dp_now.data=4,flag_dp_aisle=0;
	// 	ROS_INFO("Change to the Darea state");
	// 	flag_dp_aisle = 0;
	// 	flag_dp_pub.publish(flag_dp_now);
	// }
	// else if(flag_dp_C==0 && odom1.pose.pose.position.x>4.38 && odom1.pose.pose.position.x<5.74 && odom1.pose.pose.position.y<0.05 && odom1.pose.pose.position.y>-2.42)
	// {
	// 	flag_dp_C = 1,flag_dp_now.data=5;
	// 	ROS_INFO("Change to the Carea state");
	// 	flag_dp_aisle = 0;
	// 	flag_dp_pub.publish(flag_dp_now);
	// }
	// else if(flag_dp_B==0 && odom1.pose.pose.position.x>4.3 && odom1.pose.pose.position.x<5.74 && odom1.pose.pose.position.y<-2.9 && odom1.pose.pose.position.y>-5.5)
	// {
	// 	flag_dp_B = 1,flag_dp_now.data=6;
	// 	ROS_INFO("Change to the Barea state");
	// 	flag_dp_aisle = 0,flag_dp_E = 0,flag_dp_Edoor=0,flag_dp_ramp=0,flag_dp_F=0;
	// 	flag_dp_pub.publish(flag_dp_now);
	// }
	else if(flag_dp_ramp==0 && odom1.pose.pose.position.x>0.42 && odom1.pose.pose.position.x<1.22 && odom1.pose.pose.position.y<-0.5 && odom1.pose.pose.position.y>-2.0)
	{
		flag_dp_ramp = 1,flag_dp_now.data=7;
		ROS_INFO("Change to the ramp state");
		flag_dp_pub.publish(flag_dp_now);
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_dp_F==0 && odom1.pose.pose.position.x>0.3 && odom1.pose.pose.position.x<1.9 && odom1.pose.pose.position.y<-2.0 && odom1.pose.pose.position.y>-2.5)
	{
		flag_dp_F = 1,flag_dp_now.data=8;
		flag_dp_Fin = 0;
		ROS_INFO("Change to the Farea state");
		flag_dp_ramp=0,flag_dp_stop=0;
		flag_dp_pub.publish(flag_dp_now);
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_dp_F==1 && flag_dp_Fin == 0 && odom1.pose.pose.position.y<-2.7 && odom1.pose.pose.position.y>-5.5) ///<-2.6   729
	{
		flag_dp_Fin = 1,flag_dp_F = 0,flag_dp_now.data=2;
		ROS_INFO("Change to the Finarea state");
		flag_dp_ramp=0,flag_dp_stop=0;
		flag_dp_pub.publish(flag_dp_now);
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_dp_stop==0 && flag_dp_F==1 && odom1.pose.pose.position.x>0 && odom1.pose.pose.position.x<1.5 && odom1.pose.pose.position.y<0.05 && odom1.pose.pose.position.y>-0.75)
	{
		flag_dp_stop = 1,flag_dp_now.data=9;
		flag_dp_ramp = 1,flag_dp_F = 1;
		ROS_INFO("Change to the stop state");
		flag_dp_pub.publish(flag_dp_now);
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/*****************************************************
	*区域E，第一次
	*过坡道位置，打开摄像头开始识别
	*进门，开始有效识别
	*当看到D区域（依角度控制）/或是到达区域E第一个停车点位且拍照结束，结束识别
	*走向D区域，确保结束识别
	******************************************************/
	if(odom1.pose.pose.position.x>1.0 && flag_start_cv_E1 == 0 && odom1.pose.pose.position.y<0.05 )
	{
		flag_start_cv_E1 = 1 ;
		flag_area_cv.data = 1 ;
		start_area_cv_pub.publish(flag_area_cv);//发给predict,打开摄像头用
		printf("第一次，开始识别区域E！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(odom1.pose.pose.position.x>2 && flag_start_cv_E1 == 1 && flag_enter_cv_E1 == 0 && odom1.pose.pose.position.y<-0.6 )
	{
		flag_enter_cv_E1 = 1 ;
		flag_area_cv.data = 1 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("第一次，开始有效识别区域E！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(ucar_state_now.data == status_arrived_areaE && flag_end_cv_E1 == 0 && flag_enter_cv_E1 == 1)
	{
		flag_end_cv_E1 = 1 ;
		flag_area_cv.data = 1 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("第一次，区域E结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_E1 == 0  && status_goingto_areaD == ucar_state_now.data && flag_enter_cv_E1 == 1)
	{
		flag_tend_cv_E1 = 1 ;
		flag_area_cv.data = 1 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("第一次，区域E确保结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/*****************************************************
	*区域D
	*出区域E且区域E识别结束，打开摄像头开始识别
	*进门，开始有效识别
	*当看到D区域（依角度控制）/也可以说是D区域最后一张定点转向照片拍完，结束识别
	*走向C区域，确保结束识别
	******************************************************/
	else if(odom1.pose.pose.position.x>2.32 && odom1.pose.pose.position.y<-1.95 && flag_start_cv_D == 0 && (flag_end_cv_E1 == 1 || flag_tend_cv_E1 == 1))
	{
		flag_start_cv_D = 1 ;
		flag_area_cv.data = 2 ;
		start_area_cv_pub.publish(flag_area_cv);
		printf("开始识别区域D！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(odom1.pose.pose.position.x>2.32 && flag_enter_cv_D == 0 && odom1.pose.pose.position.y<-3.20 && flag_start_cv_D == 1 )
	{
		flag_enter_cv_D = 1 ;
		flag_area_cv.data = 2 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("开始有效识别区域D！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(for_cv_mode.data == status_arrived_areaD_last && flag_end_cv_D == 0 && flag_enter_cv_D == 1)
	{
		flag_end_cv_D = 1 ;
		flag_area_cv.data = 2 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域D结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_D == 0  && status_goingto_areaC == ucar_state_now.data && flag_enter_cv_D == 1 )
	{
		flag_tend_cv_D = 1 ;
		flag_area_cv.data = 2 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域D确保结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/*****************************************************
	*区域C
	*出区域D且区域D识别结束，且车头转过来，打开摄像头开始识别
	*进门，开始有效识别
	*当看到B区域（依角度控制），结束识别
	*走向B区域，确保结束识别
	******************************************************/
	else if(odom1.pose.pose.position.x>4.4 && flag_start_cv_C == 0 && (flag_end_cv_D == 1 || flag_tend_cv_D == 1) && odom1.pose.pose.position.y>-3.11)// && ( ((float)odom1.pose.pose.position.z>0.523 &&(float)odom1.pose.pose.position.z<3.14)|| (float)odom1.pose.pose.position.z>6.803 || (float)odom1.pose.pose.position.z<-3.663 ) )
	{
		flag_start_cv_C = 1 ;
		flag_area_cv.data = 3 ;
		start_area_cv_pub.publish(flag_area_cv);
		printf("开始识别区域C！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(odom1.pose.pose.position.x>4.20 && flag_enter_cv_C == 0 && odom1.pose.pose.position.y>-2.19 && flag_start_cv_C == 1)
	{
		flag_enter_cv_C = 1 ;
		flag_area_cv.data = 3 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("开始有效识别区域C！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(for_cv_mode.data == status_arrived_areaC_last && flag_end_cv_C == 0 && flag_enter_cv_C == 1 )//这里的条件到时候得改成C2的最后一个角度转圈完成
	{
		flag_end_cv_C = 1 ;
		flag_area_cv.data = 3 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域C结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_C == 0  && status_goingto_areaB == ucar_state_now.data && flag_start_cv_C == 1)
	{
		flag_tend_cv_C = 1 ;
		flag_area_cv.data = 3 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域C确保结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/*****************************************************
	*区域B
	*出区域C且区域C识别结束，且车头转过来，打开摄像头开始识别
	*进门，开始有效识别
	*当看到B区域（依角度控制），结束识别
	*走向B区域，确保结束识别
	******************************************************/
	else if(odom1.pose.pose.position.x>4.24 && flag_start_cv_B == 0 && (flag_end_cv_C == 1 || flag_tend_cv_C == 1) && odom1.pose.pose.position.y<-2.6)
	{
		flag_start_cv_B = 1 ;
		flag_area_cv.data = 4 ;
		start_area_cv_pub.publish(flag_area_cv);
		printf("区域B开始识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(odom1.pose.pose.position.x>4.24 && flag_enter_cv_B == 0 && flag_start_cv_B == 1 && odom1.pose.pose.position.y<-3.32)//-3.25
	{
		flag_enter_cv_B = 1 ;
		flag_area_cv.data = 4 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("开始有效识别区域B！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(for_cv_mode.data == status_arrived_areaB_last && flag_end_cv_B == 0 && flag_enter_cv_B == 1 )//这里的条件到时候得改成B2的最后一个角度转圈完成
	{
		flag_end_cv_B = 1 ;
		flag_area_cv.data = 4 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域B结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_B == 0  && status_goingto_areaE_second == ucar_state_now.data && flag_enter_cv_B == 1) //注意 这里从B出来后是去E2的
	{
		flag_tend_cv_B = 1 ;
		flag_area_cv.data = 4 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域B确保结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_B == 0  && status_goingto_areaF == ucar_state_now.data && flag_enter_cv_B == 1) //注意 这里从B出来后是去F的
	{
		flag_tend_cv_B = 1 ;
		flag_area_cv.data = 4 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域B确保结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/*****************************************************
	*区域E,第二次
	*出区域B且区域B识别结束，且车头转过来朝向E，打开摄像头开始识别
	*进门，开始有效识别
	*当走到E门，结束识别
	*走向F区域，确保结束识别
	******************************************************/
	else if(odom1.pose.pose.position.x<2.90 && flag_start_cv_E2 == 0 && (flag_end_cv_B == 1 ||flag_tend_cv_B == 1) && odom1.pose.pose.position.y>-2.40 && need_two_goal_for_parkingE == true)// && flag_go_E.data == 1) //y:-1.9
	{
		flag_start_cv_E2 = 1 ;
		flag_area_cv.data = 5 ;
		start_area_cv_pub.publish(flag_area_cv);
		printf("第二次，区域E开始识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(odom1.pose.pose.position.x<3.14 && flag_enter_cv_E2 == 0 && flag_start_cv_E2 == 1 && odom1.pose.pose.position.y>-2.35 ) //y:-1.9
	{
		flag_enter_cv_E2 = 1 ;
		flag_area_cv.data = 5 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("第二次，区域E开始有效识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(ucar_state_now.data == status_arrived_areaE_second && flag_end_cv_E2 == 0 && flag_enter_cv_E2 == 1 ) 
	{
		flag_end_cv_E2 = 1 ;
		flag_area_cv.data = 5 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("第二次，区域E结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_E2 == 0  && status_goingto_areaF == ucar_state_now.data && flag_enter_cv_E2 == 1)
	{
		flag_tend_cv_E2 = 1 ;
		flag_area_cv.data = 5 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("第二次，确保区域E结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/******************************************************
	*区域F,没啥用，因为F区域必须完整识别
	*出区域E且区域E识别结束，开始上坡，打开摄像头开始识别
	*下破后，开始有效识别
	*返回坡道上坡，结束识别
	*下坡，确保结束识别
	******************************************************/
	else if(odom1.pose.pose.position.x<1.6 && flag_start_cv_F == 0 && odom1.pose.pose.position.y<-1)
	{
		flag_start_cv_F = 1 ;
		flag_area_cv.data = 6 ;
		start_area_cv_pub.publish(flag_area_cv);
		printf("区域F开始识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if( flag_enter_cv_F == 0 && odom1.pose.pose.position.y<-2.4 && flag_start_cv_F == 1 )
	{
		flag_enter_cv_F = 1 ;
		flag_area_cv.data = 6 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("区域F开始有效识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_enter_cv_F == 0 && flag_start_cv_F == 1 && ucar_state_now.data == status_arrived_areaF)
	{
		flag_enter_cv_F = 1 ;
		flag_area_cv.data = 6 ;
		true_area_cv_pub.publish(flag_area_cv);
		printf("区域F开始有效识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(odom1.pose.pose.position.x<1.8 && flag_end_cv_F == 0 && flag_enter_cv_F == 1 && odom1.pose.pose.position.y<-1.4 && for_cv_mode.data == status_arrived_areaF_last)
	{
		flag_end_cv_F = 1 ;
		flag_area_cv.data = 6 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("区域F结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	else if(flag_tend_cv_F == 0 && ucar_state_now.data == status_goingto_final_place && flag_start_cv_F == 1)
	{
		flag_tend_cv_F = 1 ;
		flag_area_cv.data = 6 ;
		end_area_cv_pub.publish(flag_area_cv);
		printf("确保区域F结束识别！");
		std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
		std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		std::cout<<"z: "<<odom1.pose.pose.position.z<<endl;
	}
	/******************************************************
	*
	*
	*
	*
	*
	*
	******************************************************/
	if(odom1.pose.pose.position.x>0.4 && odom1.pose.pose.position.x<1.66 && odom1.pose.pose.position.y<-1.4 && odom1.pose.pose.position.y>-2.4 && ucar_state_now.data == status_goingto_areaF) 
	{
		clearcostmap.data="true";
		pub_ClearCostmap.publish(clearcostmap);
		ROS_INFO("already clear the costmap ramp");
	}
	else if(odom1.pose.pose.position.x>0.7 && odom1.pose.pose.position.x<1.25 && odom1.pose.pose.position.y<-0.64 && odom1.pose.pose.position.y>-1.10 && ucar_state_now.data == status_goingto_final_place) 
	{
		clearcostmap.data="true";
		pub_ClearCostmap.publish(clearcostmap);
		ROS_INFO("already clear the costmap ramp2");
	}
}
/*****************************************************
*E区域判断函数
*用来接收predict里传来的是否要去E区域停车的信号
*若前面只识别到了两个及以下的区域，则需要去E停车识别
*去为1，不去为0
*****************************************************/
void flag_go_E_CB(const std_msgs::Int8& msg)
{
	flag_go_E.data = msg.data;
}
/*****************************************************
*摄像头回调函数
*用来接收predict里传来的拍照信息——get_finish或but_no_result
*用来抉择是否去第二个停车点
*****************************************************/
void cv_mode_callback(const std_msgs::Int8& msg)
{
	cv_mode.data = msg.data;
}

void get_new_back(const std_msgs::Int8& msg)
{
	area_is_sure.data = msg.data;
}
/*****************************************************
*action通信的激活函数
*触发action通信即刻触发active
*****************************************************/
void activeCb()
{
	goal_received_situation = 1;
	ROS_INFO("Goal Received");
}

/*****************************************************
*action通信的feedback函数
*触发action通信后，收到feedback后调用该函数
*****************************************************/
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr &feedback)
{
	// ROS_INFO("Got base_position of Feedback");
}

/*****************************************************
*action通信的结束函数
*当action通信的目标完成，触发该函数
*目前除了E区域每个区域只去了一个点位，然后通过一个点位的转圈实现融合
*后续可以添加点位
*****************************************************/
void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
	if (state == State::PREEMPTED)
	{
		ROS_INFO("Goal Cancelled");
	}
	else
	{
		if(ucar_state_now.data == status_goingto_areaE)
		{
			ucar_state_now.data = status_arrived_areaE;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaE;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域E第一个位置");
			areaE_1_arrived = 1;//本来是用来在下面判断，是否要跳过E识别的，但现在感觉其实没用，但先留着
		}
		else if(ucar_state_now.data == status_goingto_areaD)
		{
			ucar_state_now.data = status_arrived_areaD;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaD;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域D");
			// std::cout<<"X: "<<odom1.pose.pose.position.x<<endl;
			// std::cout<<"y: "<<odom1.pose.pose.position.y<<endl;
		}
		else if(ucar_state_now.data == status_goingto_areaC)
		{
			ucar_state_now.data = status_arrived_areaC;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaC;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域C");
		}
		/*else if(ucar_state_now.data == status_goingto_areaC_second)
		{
			ucar_state_now.data = status_arrived_areaC_second;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaC_second;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域C第二个停车点");
		}*/
		else if(ucar_state_now.data == status_goingto_areaB)
		{
			ucar_state_now.data = status_arrived_areaB;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaB;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域B");
		}
		/*else if(ucar_state_now.data == status_goingto_areaB_second)
		{
			ucar_state_now.data = status_arrived_areaB_second;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaB_second;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域B第二个停车点");
		}*/
		else if(ucar_state_now.data == status_goingto_areaE_second)
		{
			ucar_state_now.data = status_arrived_areaE_second;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaE_second;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域E第二个位置");
		}
		/*else if(ucar_state_now.data == status_goingto_ramp_1)
		{
			ucar_state_now.data = status_arrived_ramp_1;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_ramp_1;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达坡道入口");
		}*/
		else if(ucar_state_now.data == status_goingto_areaF)
		{
			ucar_state_now.data = status_arrived_areaF;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaF;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域F");
		}
		else if(ucar_state_now.data == status_goingto_areaF_second)
		{
			ucar_state_now.data = status_arrived_areaF_second;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaF_second;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域F第二个停车点");
		}
		/*else if(ucar_state_now.data == status_goingto_areaF_third)
		{
			ucar_state_now.data = status_arrived_areaF_third;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_areaF_third;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达区域F第三个停车点");
		}*/
		/*else if(ucar_state_now.data == status_goingto_ramp_2)
		{
			ucar_state_now.data = status_arrived_ramp_2;
     		ucar_state_pub.publish(ucar_state_now);
			cv_mode_open_cap.data=status_arrived_ramp_2;
			ucar_state_for_cv_pub.publish(cv_mode_open_cap);
			ROS_INFO("到达坡道出口");
		}*/
		else if(ucar_state_now.data == status_goingto_final_place)
		{
			ucar_state_now.data = status_arrived_final_place;
     		ucar_state_pub.publish(ucar_state_now);
			ROS_INFO("到达最终结束点");
		}
	}
}
/*****************************************************
*小车唤醒判断的回调函数
*小车唤醒且在原点赋1
*****************************************************/
void flag_Callback(std_msgs::Int32 msg_flag)
{
	if(!already_awake && msg_flag.data==1)
	{
		ucar_state_now.data=status_origin;
		already_awake=1;
	}
}
/*****************************************************
*初始化位置节点
*****************************************************/
void init_param_goal()
{
	ros::NodeHandle n("~");
    //是否需要设置第二个停车点，正常来说除了E都不需要
	n.param<bool>("need_two_goal_for_parkingE_",need_two_goal_for_parkingE,true);
	n.param<bool>("need_two_goal_for_parkingC_",need_two_goal_for_parkingC,false);
	n.param<bool>("need_two_goal_for_parkingB_",need_two_goal_for_parkingB,false);
	//区域E，第一个停车点
	n.param<double>("goal_areaE_x_",goal_areaE_1_x,2.529);
	n.param<double>("goal_areaE_y_",goal_areaE_1_y,-0.1061);
	n.param<double>("goal_areaE_z_",goal_areaE_1_z,-0.466957);
	//区域E，第二个停车点
	n.param<double>("goal_areaE_2_x_",goal_areaE_2_x,2.898);
	n.param<double>("goal_areaE_2_y_",goal_areaE_2_y,-0.5);
	n.param<double>("goal_areaE_2_z_",goal_areaE_2_z,1.975928);
	//区域D，第一个停车点
	n.param<double>("goal_areaD_x_",goal_areaD_1_x,2.8627);
	n.param<double>("goal_areaD_y_",goal_areaD_1_y,-4.0899);
	n.param<double>("goal_areaD_z_",goal_areaD_1_z,-1.996575);
	//区域C，第一个停车点
	n.param<double>("goal_areaC_x_",goal_areaC_1_x,5);
	n.param<double>("goal_areaC_y_",goal_areaC_1_y,-0.2);
	n.param<double>("goal_areaC_z_",goal_areaC_1_z,-2.628430);
	//区域C，第二个停车点
	/*n.param<double>("goal_areaC_2_x_",goal_areaC_2_x,4.79);
	n.param<double>("goal_areaC_2_y_",goal_areaC_2_y,-0.25);
	n.param<double>("goal_areaC_2_z_",goal_areaC_2_z,-2.628430);*/
	//区域B，第一个停车点
	n.param<double>("goal_areaB_1_x_",goal_areaB_1_x,5);
	n.param<double>("goal_areaB_1_y_",goal_areaB_1_y,-5);
	n.param<double>("goal_areaB_1_z_",goal_areaB_1_z,2.965937);
	//区域B，第二个停车点
	/*n.param<double>("goal_areaB_2_x_",goal_areaB_2_x,4.79);
	n.param<double>("goal_areaB_2_y_",goal_areaB_2_y,-5);
	n.param<double>("goal_areaB_2_z_",goal_areaB_2_z,2.965937);*/
	//坡道入口
	/*n.param<double>("goal_ramp_1_x_",goal_ramp_1_x,0.9675);
	n.param<double>("goal_ramp_1_y_",goal_ramp_1_y,-0.6);
	n.param<double>("goal_ramp_1_z_",goal_ramp_1_z,-1.6);*/
	//区域F，第一个停车点
	n.param<double>("goal_areaF_1_x_",goal_areaF_1_x,0.9675);
	n.param<double>("goal_areaF_1_y_",goal_areaF_1_y,-2.5);
	n.param<double>("goal_areaF_1_z_",goal_areaF_1_z,2.40);
	//区域F，第二个停车点
	n.param<double>("goal_areaF_2_x_",goal_areaF_2_x,0.9675);
	n.param<double>("goal_areaF_2_y_",goal_areaF_2_y,-4.75);
	n.param<double>("goal_areaF_2_z_",goal_areaF_2_z,0.785);
	//区域F，第三个停车点
	/*n.param<double>("goal_areaF_3_x_",goal_areaF_3_x,1.65);
	n.param<double>("goal_areaF_3_y_",goal_areaF_3_y,-4.75);
	n.param<double>("goal_areaF_3_z_",goal_areaF_3_z,2.355);*/
	//坡道出口
	/*n.param<double>("goal_ramp_2_x_",goal_ramp_2_x,0.9675);
	n.param<double>("goal_ramp_2_y_",goal_ramp_2_y,-0.65);
	n.param<double>("goal_ramp_2_z_",goal_ramp_2_z,1.6);*/
	//终点
	n.param<double>("goal_final_x_",goal_final_x,-0.039747);
	n.param<double>("goal_final_y_",goal_final_y,0.023227);
	n.param<double>("goal_final_z_",goal_final_z,0.00242+3.14159);
}

void ucar_state_callback(const std_msgs::Int8& msg)
{
	for_cv_mode.data = msg.data;
}
/*****************************************************
*发送新目标函数
*还不会写
*****************************************************/
/*void SendNewGoal()
{
	if()
}*/
int main(int argc, char **argv)
{
	setlocale(LC_ALL,"");
	ros::init(argc, argv, "control_test");
	ros::NodeHandle n("~");
	init_param_goal();
	ros::Rate loop_rate(100);
	ucar_state_now.data=status_waitting_for_awake;
	cv_mode.data=0;
	for_cv_mode.data=0;
	flag_go_E.data=0;
	init_goalList();
	geometry_msgs::PoseWithCovarianceStamped init_pose;
	geometry_msgs::Twist vel_cmd;

	flag_dp_pub = n.advertise<std_msgs::Int32>("/flag2",100);

	ros::Subscriber pose_sub=n.subscribe("/odom",100,poseCB);

	ucar_state_pub = n.advertise<std_msgs::Int8>("/ucar_state", 1000);
	start_area_cv_pub = n.advertise<std_msgs::Int8>("/start_room_test", 1000);
	end_area_cv_pub = n.advertise<std_msgs::Int8>("/end_room_test", 1000);
	true_area_cv_pub = n.advertise<std_msgs::Int8>("/true_room_test", 1000);
	ucar_state_for_cv_pub=n.advertise<std_msgs::Int8>("/ucar_state_for_cv", 1000);
	vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);  // 未使用
	init_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1000);
	cv_mode_pub = n.advertise<std_msgs::Int8>("/cv_mode", 10);
	mapclean_client = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	ros::Subscriber laser_sub = n.subscribe("/scan", 100, laserCallback);
	ros::Subscriber cv_mode_sub = n.subscribe("/cv_mode", 100, cv_mode_callback);
	ros::Subscriber ucar_state_for_cv_sub = n.subscribe("/ucar_state_for_cv", 100, ucar_state_callback);
	ros::Subscriber flag_sub = n.subscribe("/ucar_navigation/xf_asr_offline_node/awake_flag_topic", 1, flag_Callback);
	ros::Subscriber get_new_sub = n.subscribe("/get_new", 100,  get_new_back);
	ros::Subscriber flag_go_E_sub = n.subscribe("/flag_go_E", 100,  flag_go_E_CB);
	pub_ClearCostmap=n.advertise<std_msgs::String>("/flag_Clear", 1); //发布清除costmap的信号
	Client ac("move_base", true);
	if (!ac.waitForServer(ros::Duration(6)))
	{
		ROS_ERROR("Can't connected to move base server");
		assert(0);
	}
	int goal_received_judge = 0;

	while(ros::ok())
	{
		/*****************************************************
		*这里首先有6个if，表示对不同区域的提前识别并更新目标点
		*通过接收当前区域是否已经得到结果来更新目的地点，E有两次判断，B有两种不同的更新
		*****************************************************/
		if(area_is_sure.data == 5 && cv_mode.data == cv_mode_areaF_photo_get_finish_but_no_result && flag_bug == 0)
		{
			flag_bug = 1;
			cv_mode.data = cv_mode_areaF_photo_get_finish;
		}
		if(area_is_sure.data == 1 && need_two_goal_for_parkingE == true && flag_go_E.data == 0 && areaE_flag == 0)//接收到区域E已经识别完成
		{
			goal_received_situation = 0;
			areaE_flag = 1;//后续可以利用这个标志位，跳过第二遍识别E区域
			need_two_goal_for_parkingE = false;
		}
		if(area_is_sure.data == 2)//接收到区域D已经识别完成
		{
			if(areaD_flag == 0)//D区域还没识别过
			{
				goal_received_situation = 0;
				areaD_flag = 1;
			}
			if(!goal_received_situation)
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose = pose_areaC;
				if(clear_map_D == 0)
					{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapD");
						clear_map_D = 1;
					}
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标后出发active，situation为1，进入else
				ROS_INFO("取消目标去区域C");
			}
			else
			{
				ucar_state_now.data = status_goingto_areaC;
				ucar_state_pub.publish(ucar_state_now);
				goal_received_judge=1;
				clearcostmap.data="true";
				pub_ClearCostmap.publish(clearcostmap);
				ROS_INFO("already clear the costmapD");
				area_is_sure.data = 0;
			}
		}
		if(area_is_sure.data == 3)//接收到区域C已经识别完成
		{
			if(areaC_flag == 0)//C区域还没识别过
			{
				goal_received_situation = 0;
				areaC_flag = 1;
			}
			if(!goal_received_situation)
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose = pose_areaB;
				if(clear_map_C == 0)
					{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapC");
						clear_map_C = 1;
					}
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标后出发active，situation为1，进入else
				ROS_INFO("取消目标去区域B");
			}
			else
			{
				ucar_state_now.data = status_goingto_areaB;
				ucar_state_pub.publish(ucar_state_now);
				goal_received_judge=1;
				clearcostmap.data="true";
				pub_ClearCostmap.publish(clearcostmap);
				ROS_INFO("already clear the costmapC");
				area_is_sure.data = 0;
			}
		}
		if(area_is_sure.data == 4 && areaE_flag == 0 && need_two_goal_for_parkingE == true && flag_go_E.data == 1)//接收到区域B已经识别完成,且区域E并未识别到,前往区域E2
		{
			// printf("B11111111111");
			if(areaB_flag == 0)//B区域还没识别过
			{
				goal_received_situation = 0;
				areaB_flag = 1;
			}
			if(!goal_received_situation)
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose = pose_areaE_2;
				if(clear_map_B == 0)
					{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapB");
						clear_map_B = 1;
					}
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标后出发active，situation为1，进入else
				ROS_INFO("取消目标去区域E,第二个点位");
			}
			else
			{
				ucar_state_now.data = status_goingto_areaE_second;
				ucar_state_pub.publish(ucar_state_now);
				goal_received_judge=1;
				clearcostmap.data="true";
				pub_ClearCostmap.publish(clearcostmap);
				ROS_INFO("already clear the costmapB");
				area_is_sure.data = 0;
			}
		}
		if(area_is_sure.data == 4 && (areaE_flag == 1 || need_two_goal_for_parkingE == false || flag_go_E.data == 0))//接收到区域B已经识别完成,且区域E已经识别到了,前往坡道入口
		{
			// printf("B2222222222222");
			if(areaB_flag == 0)//B区域还没识别过
			{
				goal_received_situation = 0;
				areaB_flag = 1;
			}
			if(!goal_received_situation)
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				//goal.target_pose.pose = pose_ramp_1;
				goal.target_pose.pose = pose_areaF;
				if(clear_map_B == 0)
					{
						// flag_dp_now.data=4;
						// flag_dp_pub.publish(flag_dp_now);
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapB");
						clear_map_B = 1;
					}
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标后出发active，situation为1，进入else
				ROS_INFO("取消目标去区域F");
			}
			else
			{
				// ucar_state_now.data = status_goingto_ramp_1;
				ucar_state_now.data = status_goingto_areaF;
				ucar_state_pub.publish(ucar_state_now);
				goal_received_judge=1;
				clearcostmap.data="true";
				pub_ClearCostmap.publish(clearcostmap);
				ROS_INFO("already clear the costmapB");
				area_is_sure.data = 0;
			}
		}
		if(area_is_sure.data == 1 && need_two_goal_for_parkingE == true && flag_go_E.data == 1 && areaB_flag == 1)//接收到区域E已经识别完成，且之前到达过E的第一个点位，表明现在是在E的第二点位识别到的
		{
			if(areaE_flag == 0 )//E区域还没识别过，如果这里不是0，则表示第一个被识别过了，那这个本身就进不来的
			{
				goal_received_situation = 0;
				areaE_flag = 1;//后续可以利用这个标志位，跳过第二遍识别E区域
			}
			if(!goal_received_situation)
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				// goal.target_pose.pose = pose_ramp_1;
				goal.target_pose.pose = pose_areaF;
				if(clear_map_E_2 == 0)
					{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapE2");
						clear_map_E_2 = 1;
					}
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标后出发active，situation为1，进入else
				// ROS_INFO("取消目标去坡道入口");
				ROS_INFO("取消目标去区域F");
			}
			else
			{
				// ucar_state_now.data = status_goingto_ramp_1;
				ucar_state_now.data = status_goingto_areaF;
				ucar_state_pub.publish(ucar_state_now);
				goal_received_judge=1;
				clearcostmap.data="true";
				pub_ClearCostmap.publish(clearcostmap);
				ROS_INFO("already clear the costmapE2");
				area_is_sure.data = 0;
			}
		}
		if(area_is_sure.data == 5)//接收到区域F已经识别完成
		{
			if(areaF_flag == 0)//C区域还没识别过
			{
				goal_received_situation = 0;
				areaF_flag = 1;
			}
			if(!goal_received_situation)
			{
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.pose = pose_final;
				if(clear_map_F3 == 0)
					{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapF");
						clear_map_F3 = 1;
					}
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);//发送目标后出发active，situation为1，进入else
				ROS_INFO("取消目标去终点");
			}
			else
			{
				ucar_state_now.data = status_goingto_final_place;
				ucar_state_pub.publish(ucar_state_now);
				goal_received_judge=1;
				clearcostmap.data="true";
				pub_ClearCostmap.publish(clearcostmap);
				ROS_INFO("already clear the costmapF");
				area_is_sure.data = 0;
			}
		}
		/*这里可以添加动态参数配置的话题发布*/
		/**********************************/
		if (!ac.isServerConnected())
		{
			ROS_WARN("Move_Base Actionlib Server Disonnected");
			if (!ac.waitForServer(ros::Duration(1)))
			{
				ROS_ERROR("Can't connected to move base server");
				assert(0);
			}
		}
		
		switch(ucar_state_now.data)
		{
			case status_origin://小车在初始点
				if(!goal_received_situation)//初始为0，active触发后为1，当提前确定区域信息后为0,即当前没有收到去目标的信息，没有目的地,且当前在某个arrived的状态
				{
					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();
					goal.target_pose.pose = pose_areaD;
					ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
					ROS_INFO("SEND GOAL(TO AREAD)");
				}
				else
				{
					ucar_state_now.data = status_goingto_areaD;
					ucar_state_pub.publish(ucar_state_now);
					clearcostmap.data="true";
					pub_ClearCostmap.publish(clearcostmap);
					ROS_INFO("already clear the costmap");
					goal_received_judge=1;
				}
				break;
/*			case status_arrived_areaE:
				if(goal_received_judge == 1)//表示在arrived之前确定是过了going的状态，即时收到过goal的
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaE_1_photo_get_finish && cv_mode.data != cv_mode_areaE_1_photo_get_finish_but_no_result)
				{//到这个区域，但摄像头未识别到
					ROS_INFO("WAIT FOR OPENCV AREAE");
					goal_received_situation = 0;
				}
				else//摄像头识别到了
				{
					if(need_two_goal_for_parkingE == false && cv_mode.data == cv_mode_areaE_1_photo_get_finish_but_no_result)//到这个区域，摄像头识别到了，但没有发现是什么东西，并且目前不打算去第二个停车点
					{
						need_two_goal_for_parkingE = true;
					}
					else if(cv_mode.data == cv_mode_areaE_1_photo_get_finish)
					{
						need_two_goal_for_parkingE = false;
						areaE_flag = 1;
					}
					if(!goal_received_situation)//这边就不管有没有识别到信息，先直接去下一个点
					{
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();
						goal.target_pose.pose = pose_areaD;
						if(clear_map_E_1 == 0)
						{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapE");
						clear_map_E_1 = 1;
						}
						ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
						ROS_INFO("SEND GOAL(TO AREAD)");
					}
					else
					{//目标发送成功
						ucar_state_now.data = status_goingto_areaD;
						ucar_state_pub.publish(ucar_state_now);
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapE");
						goal_received_judge=1;
					}
				}
				break;*/
			case status_arrived_areaD:
				if(goal_received_judge == 1)
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaD_photo_get_finish && cv_mode.data != cv_mode_areaD_photo_get_finish_but_no_result)
				{
					ROS_INFO("WAIT FOR OPENCV AREAD");
					goal_received_situation=0;
				}
				else
				{
					if(cv_mode.data == cv_mode_areaD_photo_get_finish_but_no_result)//到这个区域，摄像头识别到了，但没有发现是什么东西，并且目前不打算去第二个停车点
					{
						ROS_INFO("D NO RESULT");
					}
					else if(cv_mode.data == cv_mode_areaD_photo_get_finish)
					{
						ROS_INFO("D GET RESULT");
					}
					if(!goal_received_situation)//这边就不管有没有识别到信息，先直接去下一个点
					{
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();
						goal.target_pose.pose = pose_areaC;
						if(clear_map_D == 0)
						{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapD");
						clear_map_D = 1;
						}
						ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
						ROS_INFO("SEND GOAL(TO AREAC)");
					}
					else
					{//目标发送成功
						ucar_state_now.data = status_goingto_areaC;
						ucar_state_pub.publish(ucar_state_now);
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapD");
						goal_received_judge=1;
					}
				}
				break;
			case status_arrived_areaC:
				if(goal_received_judge == 1)//表示在arrived之前确定是过了going的状态，即时收到过goal的
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaC_photo_get_finish && cv_mode.data != cv_mode_areaC_photo_get_finish_but_no_result)//等待摄像头识别
				{
					ROS_INFO("WAIT FOR OPENCV AREAC");
					goal_received_situation=0;
				}
				else
				{
					if(need_two_goal_for_parkingC == false && cv_mode.data == cv_mode_areaC_photo_get_finish_but_no_result)//到这个区域C，摄像头识别到了，但没有发现是什么东西，并且目前打算去第二个停车点
					{
						ROS_INFO("C NO RESULT");
						if(!goal_received_situation)//这边就不管有没有识别到信息，先直接去下一个点
						{
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose = pose_areaB;
							if(clear_map_C == 0)
							{
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapC");
							clear_map_C = 1;
							}
							ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
							ROS_INFO("SEND GOAL(TO AREAB)");
						}
						else
						{//目标发送成功
							// ucar_state_now.data = status_goingto_areaC_second;
							ucar_state_now.data = status_goingto_areaB;
							ucar_state_pub.publish(ucar_state_now);
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapC");
							goal_received_judge=1;
						}
					}
					else if(cv_mode.data == cv_mode_areaC_photo_get_finish)//在区域C的第一个位置已经识别到了
					{
						ROS_INFO("C GET RESULT");
						if(!goal_received_situation)//这边就不管有没有识别到信息，先直接去下一个点
						{
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose = pose_areaB;
							if(clear_map_C == 0)
							{
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapC");
							clear_map_C = 1;
							}
							ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
							ROS_INFO("SEND GOAL(TO AREAB)");
						}
						else
						{//目标发送成功
							ucar_state_now.data = status_goingto_areaB;
							ucar_state_pub.publish(ucar_state_now);
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapC");
							goal_received_judge=1;
						}
					}
					
				}
				break;
			case status_arrived_areaB:
				if(areaB_flag == 0)
				{
					areaB_flag = 1;
				}
				if(goal_received_judge == 1)
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaB_photo_get_finish && cv_mode.data != cv_mode_areaB_photo_get_finish_but_no_result)
				{
					ROS_INFO("WAIT FOR OPENCV AREAB");
					goal_received_situation=0;
				}
				else
				{
					if(need_two_goal_for_parkingB == false && cv_mode.data == cv_mode_areaB_photo_get_finish_but_no_result)//到这个区域，摄像头识别到了，但没有发现是什么东西，就去E的第二个点来推测B
					{
						ROS_INFO("B NO RESULT");
						if(!goal_received_situation)//
						{
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose = pose_areaE_2;
							if(clear_map_B == 0)
							{
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapB");
							clear_map_B = 1;
							}
							ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
							ROS_INFO("SEND GOAL(TO AREAE SECOND)");
						}
						else
						{//目标发送成功
							ucar_state_now.data = status_goingto_areaE_second;
							ucar_state_pub.publish(ucar_state_now);
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapB");
							goal_received_judge=1;
						}
					}
					else if(cv_mode.data == cv_mode_areaB_photo_get_finish)//我们知道B的信息了
					{
						ROS_INFO("B GET RESULT");
						if(need_two_goal_for_parkingE == true && flag_go_E.data == 1)//但前面有个地方的信息还不确定，所以得去E停车识别E，来推测前面的点
						{
							if(!goal_received_situation)//这边就不管有没有识别到信息，先直接去下一个点
							{
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								goal.target_pose.pose = pose_areaE_2;
								if(clear_map_B == 0)
								{
								clearcostmap.data="true";
								pub_ClearCostmap.publish(clearcostmap);
								ROS_INFO("already clear the costmapB");
								clear_map_B = 1;
								}
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								ROS_INFO("SEND GOAL(TO AREAE SECOND)");
							}
							else
							{//目标发送成功
								ucar_state_now.data = status_goingto_areaE_second;
								ucar_state_pub.publish(ucar_state_now);
								clearcostmap.data="true";
								pub_ClearCostmap.publish(clearcostmap);
								ROS_INFO("already clear the costmapB");
								goal_received_judge=1;
							}
						}
						else//那就说明不用去E，前面都BCD都知道了，可以推测出E了，就直接去F
						{
							if(!goal_received_situation)
							{
								goal.target_pose.header.frame_id = "map";
								goal.target_pose.header.stamp = ros::Time::now();
								// goal.target_pose.pose = pose_ramp_1;
								goal.target_pose.pose = pose_areaF;
								if(clear_map_B == 0)
								{
								clearcostmap.data="true";
								pub_ClearCostmap.publish(clearcostmap);
								ROS_INFO("already clear the costmapB");
								clear_map_B = 1;
								}
								ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
								// ROS_INFO("SEND GOAL(TO RAMP1)");
								ROS_INFO("SEND GOAL(TO AREAF)");
							}
							else
							{//目标发送成功
								// ucar_state_now.data = status_goingto_ramp_1;
								ucar_state_now.data = status_goingto_areaF;
								ucar_state_pub.publish(ucar_state_now);
								clearcostmap.data="true";
								pub_ClearCostmap.publish(clearcostmap);
								ROS_INFO("already clear the costmapB");
								goal_received_judge=1;
							}
						}
					}
				}
				break;
			case status_arrived_areaE_second:
				if(goal_received_judge == 1)
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaE_2_photo_get_finish && cv_mode.data != cv_mode_areaE_2_photo_get_finish_but_no_result)
				{
					ROS_INFO("WAIT FOR OPENCV AREAE2");
					goal_received_situation=0;
				}
				else
				{
					if(cv_mode.data == cv_mode_areaE_2_photo_get_finish_but_no_result)//到这个区域，摄像头识别到了，但没有发现是什么东西，并且目前不打算去第二个停车点
					{
						ROS_INFO("E2 NO RESULT");
					}
					else if(cv_mode.data == cv_mode_areaE_2_photo_get_finish)
					{
						ROS_INFO("E2 GET RESULT");
					}
					if(!goal_received_situation)//这边就不管有没有识别到信息，先直接去坡道入口
					{
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();
						// goal.target_pose.pose = pose_ramp_1;
						goal.target_pose.pose = pose_areaF;
						if(clear_map_E_2 == 0)
						{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapE2");
						clear_map_E_2 = 1;
						}
						ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
						ROS_INFO("SEND GOAL(TO AREAF)");
					}
					else
					{//目标发送成功
						// ucar_state_now.data = status_goingto_ramp_1;
						ucar_state_now.data = status_goingto_areaF;
						ucar_state_pub.publish(ucar_state_now);
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapE2");
						goal_received_judge=1;
					}
				}
				break;
			/*case status_arrived_ramp_1:
				if(goal_received_judge ==1 )
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(!goal_received_situation)//这边就不管有没有识别到信息，就直接跑到F了
				{
					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();
					goal.target_pose.pose = pose_areaF;
					if(clear_map_ramp_1 == 0)
					{
					clearcostmap.data="true";
					pub_ClearCostmap.publish(clearcostmap);
					ROS_INFO("already clear the costramp1");
					clear_map_ramp_1 = 1;
					}
					ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
					ROS_INFO("SEND GOAL(TO AREAF)");
				}
				else
				{//目标发送成功
					ucar_state_now.data = status_goingto_areaF;
					ucar_state_pub.publish(ucar_state_now);
					clearcostmap.data="true";
					pub_ClearCostmap.publish(clearcostmap);
					ROS_INFO("already clear the costramp1");
					goal_received_judge=1;
				}
				break;*/
			case status_arrived_areaF:
				if(goal_received_judge == 1)
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaF_photo_get_finish && cv_mode.data != cv_mode_areaF_photo_get_finish_but_no_result)
				{
					ROS_INFO("WAIT FOR OPENCV AREAF");
					goal_received_situation=0;
				}
				else
				{
					if(cv_mode.data == cv_mode_areaF_photo_get_finish_but_no_result)//到这个区域，摄像头识别到了，但没有发现是什么东西，并且目前不打算去第二个停车点
					{
						ROS_INFO("F NO RESULT");
						if(!goal_received_situation)//这边就不管有没有识别到信息，先直接坡道出口
						{
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.header.stamp = ros::Time::now();
							// goal.target_pose.pose = pose_ramp_2;
							goal.target_pose.pose = pose_areaF_2;
							if(clear_map_F == 0)
							{
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapF");
							clear_map_F = 1;
							}
							ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
							ROS_INFO("SEND GOAL(TO AREA F SECOND)");
						}
						else
						{//目标发送成功
							// ucar_state_now.data = status_goingto_ramp_2;
							ucar_state_now.data = status_goingto_areaF_second;
							ucar_state_pub.publish(ucar_state_now);
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapF");
							goal_received_judge=1;
						}
					}
					else if(cv_mode.data == cv_mode_areaF_photo_get_finish)
					{
						ROS_INFO("F GET RESULT");
						if(!goal_received_situation)//这边就不管有没有识别到信息，先直接坡道出口
						{
							goal.target_pose.header.frame_id = "map";
							goal.target_pose.header.stamp = ros::Time::now();
							// goal.target_pose.pose = pose_ramp_2;
							goal.target_pose.pose = pose_final;
							if(clear_map_F == 0)
							{
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapF");
							clear_map_F = 1;
							}
							ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
							ROS_INFO("SEND GOAL(TO AREA FIANL)");
						}
						else
						{//目标发送成功
							// ucar_state_now.data = status_goingto_ramp_2;
							ucar_state_now.data = status_goingto_final_place;
							ucar_state_pub.publish(ucar_state_now);
							clearcostmap.data="true";
							pub_ClearCostmap.publish(clearcostmap);
							ROS_INFO("already clear the costmapF");
							goal_received_judge=1;
						}
					}
					
				}
				break;
			case status_arrived_areaF_second:
				if(goal_received_judge == 1)
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(cv_mode.data!=cv_mode_areaF_second_photo_get_finish && cv_mode.data != cv_mode_areaF_second_photo_get_finish_but_no_result)
				{
					ROS_INFO("WAIT FOR OPENCV AREAF SECOND");
					goal_received_situation=0;
				}
				else
				{
					if(cv_mode.data == cv_mode_areaF_second_photo_get_finish_but_no_result)//到这个区域，摄像头识别到了，但没有发现是什么东西，并且目前不打算去第二个停车点
					{
						ROS_INFO("F SECOND NO RESULT");
					}
					else if(cv_mode.data == cv_mode_areaF_second_photo_get_finish)
					{
						ROS_INFO("F SECOND GET RESULT");
					}
					if(!goal_received_situation)//这边就不管有没有识别到信息，先直接坡道出口
					{
						goal.target_pose.header.frame_id = "map";
						goal.target_pose.header.stamp = ros::Time::now();
						// goal.target_pose.pose = pose_ramp_2;
						goal.target_pose.pose = pose_final;
						if(clear_map_F2 == 0)
						{
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapF SECOND");
						clear_map_F2 = 1;
						}
						ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
						ROS_INFO("SEND GOAL(TO AREA FIANL)");
					}
					else
					{//目标发送成功
						// ucar_state_now.data = status_goingto_ramp_2;
						ucar_state_now.data = status_goingto_final_place;
						ucar_state_pub.publish(ucar_state_now);
						clearcostmap.data="true";
						pub_ClearCostmap.publish(clearcostmap);
						ROS_INFO("already clear the costmapF SECOND");
						goal_received_judge=1;
					}
				}
				break;
			/*case status_arrived_ramp_2:
				if(goal_received_judge ==1 )
				{
					goal_received_situation=0;
					goal_received_judge=0;
				}
				if(!goal_received_situation)//这边就不管有没有识别到信息，就直接跑到F了
				{
					goal.target_pose.header.frame_id = "map";
					goal.target_pose.header.stamp = ros::Time::now();
					goal.target_pose.pose = pose_final;
					if(clear_map_ramp_2 == 0)
					{
					clearcostmap.data="true";
					pub_ClearCostmap.publish(clearcostmap);
					ROS_INFO("already clear the costmapramp2");
					clear_map_ramp_2 = 1;
					}
					ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
					ROS_INFO("SEND GOAL(TO FINAL)");
				}
				else
				{//目标发送成功
					ucar_state_now.data = status_goingto_final_place;
					ucar_state_pub.publish(ucar_state_now);
					clearcostmap.data="true";
					pub_ClearCostmap.publish(clearcostmap);
					ROS_INFO("already clear the costramp2");
					goal_received_judge=1;
				}
				break;*/
			case status_arrived_final_place:
				if(1==goal_received_judge)
				{
					goal_received_situation=0;
					goal_received_judge=0;
					ucar_state_now.data = area_identification_start;
				}
				ROS_INFO("准备播报完成信息");
				break;
			case area_identification_start:
				ucar_state_pub.publish(ucar_state_now);
				ucar_state_now.data = status_all_finished;
				break;
			
			default: 
				break;
		}
		ros::spinOnce();
		loop_rate.sleep();	
	}
	return 0;

}
#include <ros/ros.h>
#include <std_msgs/Int8.h>
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
#include <Eigen/Core>
#include <Eigen/QR>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
using namespace std;

// #define M_PI 3.14159265358979323846
#define sleep_time 0.3
#define  status_origin  0

#define  status_goingto_areaE 1
#define  status_arrived_areaE 2
#define  status_goingto_areaE_2 3
// #define  status_arrived_areaE_2 4

#define  status_goingto_areaD 5
#define  status_arrived_areaD 6

#define  status_goingto_areaC 7
#define  status_arrived_areaC 8

#define  status_goingto_areaB 9
#define  status_arrived_areaB 10

#define  status_goingto_areaF 11
#define  status_arrived_areaF 12

#define  status_goingto_final_place 13
#define  status_arrived_final_place 14

#define status_goingto_areaC_second 15
#define status_arrived_areaC_second  16

#define status_goingto_areaB_second 17
#define status_arrived_areaB_second  18

#define status_goingto_areaF_second = 19
#define status_arrived_areaF_second = 20
#define status_goingto_areaF_middle = 21
#define status_arrived_areaF_middle = 22

//拍照相关标志位
#define cv_mode_areaE_photo_get_finish  31
#define cv_mode_areaE_photo_get_finish_but_no_result  32
#define cv_mode_areaD_photo_get_finish  33
#define cv_mode_areaD_photo_get_finish_but_no_result 34
#define cv_mode_areaC_photo_get_finish  35
#define cv_mode_areaC_photo_get_finish_but_no_result  36
#define cv_mode_areaB_photo_get_finish  37
#define cv_mode_areaB_photo_get_finish_but_no_result  38
#define cv_mode_areaE_2_photo_get_finish  39
#define cv_mode_areaE_2_photo_get_finish_but_no40
#define cv_mode_areaF_photo_get_finish  41
#define cv_mode_areaF_photo_get_finish_but_no_result  42
#define cv_mode_areaC_second_photo_get_finish 43
#define cv_mode_areaC_second_photo_get_finish_but_no_result 44
#define cv_mode_areaB_second_photo_get_finish 45
#define cv_mode_areaB_second_photo_get_finish_but_no_result 46

#define  status_arrived_areaE_1 -1 //开始第一次转90度
#define  status_arrived_areaE_2 -2 //第一次转90度完成
#define  status_arrived_areaE_3 -3 //开始第二次转90度
#define  status_arrived_areaE_4 -4 //第二次转90度完成
#define  status_arrived_areaE_5 -5 //开始第三次转90度
#define  status_arrived_areaE_6 -6 //第三次转90度完成
#define  status_arrived_areaE_7 -7 //开始第四次转90度
#define  status_arrived_areaE_8 -8 //第四次转90度完成

#define  status_arrived_areaD_1 -10 //开始第一次转90度
#define  status_arrived_areaD_2 -11 //第一次转90度完成
#define  status_arrived_areaD_3 -12 //开始第二次转90度
#define  status_arrived_areaD_4 -13 //第二次转90度完成
#define  status_arrived_areaD_5 -14 //开始第三次转90度
#define  status_arrived_areaD_6 -15 //第三次转90度完成
#define  status_arrived_areaD_7 -16 //开始第四次转90度
#define  status_arrived_areaD_8 -17 //第四次转90度完成

#define  status_arrived_areaC_1 -20 //开始第一次转90度
#define  status_arrived_areaC_2 -21 //第一次转90度完成
#define  status_arrived_areaC_3 -22 //开始第二次转90度
#define  status_arrived_areaC_4 -23 //第二次转90度完成
#define  status_arrived_areaC_5 -24 //开始第三次转90度
#define  status_arrived_areaC_6 -25 //第三次转90度完成
#define  status_arrived_areaC_7 -26 //开始第四次转90度
#define  status_arrived_areaC_8 -27 //第四次转90度完成

#define  status_arrived_areaC_1_second  -50 //开始第一次转90度
#define  status_arrived_areaC_2_second  -51 //第一次转90度完成
#define  status_arrived_areaC_3_second  -52 //开始第二次转90度
#define  status_arrived_areaC_4_second  -53 //第二次转90度完成
#define  status_arrived_areaC_5_second  -54 //开始第三次转90度
#define  status_arrived_areaC_6_second  -55 //第三次转90度完成
#define  status_arrived_areaC_7_second  -56 //开始第四次转90度
#define  status_arrived_areaC_8_second  -57 //第四次转90度完成

#define  status_arrived_areaB_1 -30 //开始第一次转90度
#define  status_arrived_areaB_2 -31 //第一次转90度完成
#define  status_arrived_areaB_3 -32 //开始第二次转90度
#define  status_arrived_areaB_4 -33 //第二次转90度完成
#define  status_arrived_areaB_5 -34 //开始第三次转90度
#define  status_arrived_areaB_6 -35 //第三次转90度完成
#define  status_arrived_areaB_7 -36 //开始第四次转90度
#define  status_arrived_areaB_8 -37 //第四次转90度完成

#define  status_arrived_areaB_1_second  -58 //开始第一次转90度
#define  status_arrived_areaB_2_second  -59 //第一次转90度完成
#define  status_arrived_areaB_3_second  -60 //开始第二次转90度
#define  status_arrived_areaB_4_second  -61 //第二次转90度完成
#define  status_arrived_areaB_5_second  -62 //开始第三次转90度
#define  status_arrived_areaB_6_second  -63 //第三次转90度完成
#define  status_arrived_areaB_7_second  -64 //开始第四次转90度
#define  status_arrived_areaB_8_second  -65 //第四次转90度完成


#define  status_arrived_areaF_1 -40 //开始第一次转90度
#define  status_arrived_areaF_2 -41 //第一次转90度完成
#define  status_arrived_areaF_3 -42 //开始第二次转90度
#define  status_arrived_areaF_4 -43 //第二次转90度完成
#define  status_arrived_areaF_5 -44 //开始第三次转90度
#define  status_arrived_areaF_6 -45 //第三次转90度完成
#define  status_arrived_areaF_7 -46 //开始第四次转90度
#define  status_arrived_areaF_8 -47 //第四次转90度完成
#define  status_arrived_areaF_9 -48 //开始第五次转90度
#define  status_arrived_areaF_10 -49 //第五次转90度完成

#define  status_arrived_areaF_1_second  -66 //开始第一次转90度
#define  status_arrived_areaF_2_second  -67//第一次转90度完成
#define  status_arrived_areaF_3_second  -68 //开始第二次转90度
#define  status_arrived_areaF_4_second  -69 //第二次转90度完成
// #define  status_arrived_areaF_9 -48 //开始第五次转90度

class PurePursuit
{
public:
    PurePursuit(ros::NodeHandle* n);
    double cospsi, sinpsi;
    double px,py;

    //用于转向拍照
    double now_yaw,yaw_max;  //用于获取当前角度，当前角度用于拍照停车转向，范围在-3.14<now_yaw<=3.14
    double areaE_1,areaE_2,areaE_3,areaE_4;
    double areaD_1,areaD_2,areaD_3,areaD_4;
    double areaC_1,areaC_2,areaC_3,areaC_4;//C区域第一个停车点
    double areaC_2_1,areaC_2_2,areaC_2_3,areaC_2_4;//C区域第二个停车点
    double areaB_1,areaB_2,areaB_3,areaB_4;//B区域第一个停车点
    double areaB_2_1,areaB_2_2,areaB_2_3,areaB_2_4;//B区域第二个停车点
    double areaE_2_1,areaE_2_2,areaE_2_3,areaE_2_4;
    double areaF_1,areaF_2,areaF_3,areaF_4,areaF_5,areaF_6,areaF_7,areaF_8,areaF_9;//F区域第一个停车点
    double areaF_2_1;//F区域第二个停车点
    double yaw_xiuzhengE, yaw_xiuzhengD, yaw_xiuzhengC,yaw_xiuzhengB, yaw_xiuzhengC_2,yaw_xiuzhengB_2,yaw_xiuzhengE_2,yaw_xiuzhengF,yaw_xiuzhengF_2;

    double kk = 1, bb = 1;
    double nowDeviation; //偏差
    double kp_inside=1.8;  //1.5
    double turn_v_max=3;  //最大转向速度
    double maxspeed=0.30;  //0.3 
    double pre_length=0.55;
    double Vx = 0, Vy = 0, turn_v = 0 ,turn_v_areaE=3,turn_v_areaD=3,turn_v_areaC=3,turn_v_areaB=3,turn_v_areaC_2=-3,turn_v_areaB_2=3,turn_v_areaE_2=3,turn_v_areaF=3,turn_v_areaF_2=3; //+为逆时针，-为顺时针
    double L=0.20;
    double areaE_1_p=0,areaE_2_p=0,areaE_3_p=0,areaE_4_p=0;//E1停车点设置
    double areaD_1_p=0,areaD_2_p=0,areaD_3_p=0,areaD_4_p=0;//D停车点设置
    double areaC_1_1_p=0,areaC_1_2_p=0,areaC_1_3_p=0,areaC_1_4_p=0;//C区域第一个停车点设置
    double areaB_1_1_p=0,areaB_1_2_p=0,areaB_1_3_p=0,areaB_1_4_p=0;//B区域第一个停车点设置
    double areaC_2_1_p=0,areaC_2_2_p=0,areaC_2_3_p=0,areaC_2_4_p=0;//C区域第二个停车点设置
    double areaB_2_1_p=0,areaB_2_2_p=0,areaB_2_3_p=0,areaB_2_4_p=0;//B区域第二个停车点设置
    double areaE_2_1_p=0,areaE_2_2_p=0,areaE_2_3_p=0,areaE_2_4_p=0;//E2停车点设置
    double areaF_1_p=0,areaF_2_p=0,areaF_3_p=0,areaF_4_p=0,areaF_5_p=0,areaF_6_p=0,areaF_7_p=0,areaF_8_p=0,areaF_9_p=0;//F停车点设置
    double areaF_2_1_p=0;//F区域第二个停车点设置
    std_msgs::Int8 ucar_state_now;
    nav_msgs::Path global_path_can_look;
    geometry_msgs::PoseWithCovarianceStamped amcl_pose_now;
    geometry_msgs::Twist cmd_vel_now, fake_vel_now;
    ros::Timer timer1;
    ros::Timer timer2;
    float goal_angle = M_PI/2;
    float angular_duration;
    int areaE_ticks=0,ticks,areaD_ticks=0,areaC_ticks=0,areaB_ticks=0,areaC_ticks_2=0,areaB_ticks_2=0,areaE_2_ticks=0,areaF_ticks=0,areaF_ticks_2=0;
    std_msgs::Int8 cv_mode;
    ros::Publisher cmd_vel_pub,ucar_state_for_cv_pub;
    double rate; //更新小车运动状态的频率
    nav_msgs::Odometry odom1;
    std_msgs::Float32 imu_data;
private:
    ros::NodeHandle n_;
    ros::Subscriber amcl_pose_sub,ucar_state_sub,global_path_sub,fake_vel_sub,cv_mode_sub ,odom_sub,imu_sub;
    ros::Publisher cv_mode_pub ;
    void amcl_poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_poseMsg);
    void ucar_stateCB(const std_msgs::Int8::ConstPtr &ucar_stateMsg);
    void global_pathCB(const nav_msgs::Path::ConstPtr &global_pathMsg);
    void fake_velCB(const geometry_msgs::Twist::ConstPtr &fake_velMsg);
    void directionControlLoopCB(const ros::TimerEvent &);
    // void directionControlcvLoopCB(const ros::TimerEvent &);
    void cv_mode_callback(const std_msgs::Int8& msg);
    void odomCB(const nav_msgs::Odometry & msg1);
    void imuCB(const std_msgs::Float32 & msg2);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);
};

PurePursuit::PurePursuit(ros::NodeHandle *n)
{
    n_.param<double>("turn_v_areaE_",turn_v_areaE,3);
    n_.param<double>("turn_v_areaD_",turn_v_areaD,3);
    n_.param<double>("turn_v_areaC_",turn_v_areaC,3);
    n_.param<double>("turn_v_areaB_",turn_v_areaB,3);
    n_.param<double>("turn_v_areaC_2_",turn_v_areaC_2,-3);
    n_.param<double>("turn_v_areaB_2_",turn_v_areaB_2,3);
    n_.param<double>("turn_v_areaE_2_",turn_v_areaE_2,3);
    n_.param<double>("turn_v_areaF_",turn_v_areaF,2);
    n_.param<double>("turn_v_areaF_2_",turn_v_areaF_2,2);


    n_.param<double>("areaE_1_p_",areaE_1_p,-0.466957+1.256); // 1.256*5=6.28
    n_.param<double>("areaE_2_p_",areaE_2_p,-0.466957+1.256*2);
    n_.param<double>("areaE_3_p_",areaE_3_p,-0.466957+1.256*3);
    n_.param<double>("areaE_4_p_",areaE_4_p,-0.466957-1.256);


    n_.param<double>("areaD_1_p_",areaD_1_p,-0.78);
    n_.param<double>("areaD_2_p_",areaD_2_p,0.7);
    n_.param<double>("areaD_3_p_",areaD_3_p,2.50);


    n_.param<double>("areaC_1_1_p_",areaC_1_1_p,0.85);
    n_.param<double>("areaC_1_2_p_",areaC_1_2_p,2.8);
    n_.param<double>("areaC_1_3_p_",areaC_1_3_p,2.8);


    n_.param<double>("areaC_2_1_p_",areaC_2_1_p,2.65);
    n_.param<double>("areaC_2_2_p_",areaC_2_2_p,0.52);
    n_.param<double>("areaC_2_3_p_",areaC_2_3_p,-0.68);


    n_.param<double>("areaB_1_1_p_",areaB_1_1_p,-2.05);
    n_.param<double>("areaB_1_2_p_",areaB_1_2_p,-1.0);
    n_.param<double>("areaB_1_3_p_",areaB_1_3_p,-1.0);


    n_.param<double>("areaB_2_1_p_",areaB_2_1_p,-2.4);
    n_.param<double>("areaB_2_2_p_",areaB_2_2_p,-0.65);
    n_.param<double>("areaB_2_3_p_",areaB_2_3_p,0.60);


    n_.param<double>("areaE_2_1_p_",areaE_2_1_p, 1.975928-1.256);
    n_.param<double>("areaE_2_2_p_",areaE_2_2_p, 1.975928-1.256*2);


    n_.param<double>("areaF_1_p_",areaF_1_p, -2.0);
    n_.param<double>("areaF_2_p_",areaF_2_p, -1.75);
    n_.param<double>("areaF_3_p_",areaF_3_p, -1.50);
    n_.param<double>("areaF_4_p_",areaF_4_p, 0.95);

    n_.param<double>("areaF_2_1_p_",areaF_2_1_p, -0.75); 


    rate=20;
    angular_duration = this->goal_angle / this->turn_v_areaE; //转圈持续时间=转圈角度/转圈速度
    ticks = int(this->angular_duration * this->rate);
    n_=*n;
    cospsi=1;
    sinpsi=0;
    now_yaw = 0;
    yaw_max =100;
    ucar_state_now.data=status_origin;// 定义原始状态为status_origin
    cmd_vel_pub = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);      //速度信息
    cv_mode_pub = n_.advertise<std_msgs::Int8>("/cv_mode", 10);
    ucar_state_for_cv_pub =n_.advertise<std_msgs::Int8>("/ucar_state_for_cv", 10);
    amcl_pose_sub = n_.subscribe("/amcl_pose", 1, &PurePursuit::amcl_poseCB, this);
    ucar_state_sub = n_.subscribe("/ucar_state", 1, &PurePursuit::ucar_stateCB, this);
    // global_path_sub = n_.subscribe("/move_base/GlobalPlanner/plan", 1, &PurePursuit::global_pathCB, this);
    global_path_sub = n_.subscribe("/move_base/NavfnROS/plan", 1, &PurePursuit::global_pathCB, this);
    fake_vel_sub = n_.subscribe("/fake_vel", 1, &PurePursuit::fake_velCB, this);
    timer1 = n_.createTimer(ros::Duration(0.002), &PurePursuit::directionControlLoopCB, this); // 500hz
    // timer2 = n_.createTimer(ros::Duration(0.01), &PurePursuit::directionControlcvLoopCB, this); // 500hz
    cv_mode_sub = n_.subscribe("/cv_mode", 100, &PurePursuit::cv_mode_callback,this);
    odom_sub = n_.subscribe("/odom1", 1, &PurePursuit::odomCB, this);
    imu_sub = n_.subscribe("/imu_data_forcv", 1, &PurePursuit::imuCB, this);

    areaE_1=areaE_1_p;
    areaE_2=areaE_2_p;
    areaE_3=areaE_3_p;
    areaE_4=areaE_4_p;

    areaD_1=areaD_1_p;
    areaD_2=areaD_2_p;
    areaD_3=areaD_3_p;
    areaD_4=areaD_4_p;

    areaC_1=areaC_1_1_p;
    areaC_2=areaC_1_2_p;
    areaC_3=areaC_1_3_p;
    areaC_4=areaC_1_4_p;

    areaC_2_1=areaC_2_1_p;
    areaC_2_2=areaC_2_2_p;
    areaC_2_3=areaC_2_3_p;
    areaC_2_4=areaC_2_4_p;

    areaB_1=areaB_1_1_p;
    areaB_2=areaB_1_2_p;
    areaB_3=areaB_1_3_p;
    areaB_4=areaB_1_4_p;

    areaB_2_1=areaB_2_1_p;
    areaB_2_2=areaB_2_2_p;
    areaB_2_3=areaB_2_3_p;
    areaB_2_4=areaB_2_4_p;

    areaE_2_1=areaE_2_1_p;
    areaE_2_2=areaE_2_2_p;
    areaE_2_3=areaE_2_3_p;
    areaE_2_4=areaE_2_4_p;

    areaF_1=areaF_1_p;
    areaF_2=areaF_2_p;
    areaF_3=areaF_3_p;
    areaF_4=areaF_4_p;  

    areaF_2_1=areaF_2_1_p;
    
    //逆时针使用3.14,顺时针使用-3.14

}
Eigen::VectorXd PurePursuit::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) //多项式拟合
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
        A(i, 0) = 1.0;

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        A(j, i + 1) = A(j, i) * xvals(j);
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

void PurePursuit::directionControlLoopCB(const ros::TimerEvent &event) //一般转向控制器
{  
     int a=1;
     if(a==0)
     { 
        //|| (px>2.86 && px<3.41 && py>-0.853 && py<0.424)(px>1.79 && px<2.71 && py>-0.07 && py<0.227)  || (px>3.40 && px<3.83 && py>-0.882 && py<-0.487)||(px>4.34 && px<4.83 && py>-0.2 && py<0.479)    
        if( px>-0.5 && px<1 && py>-3 && py<-0.5)     //px>-0.5 && px<2 && py>-3 && py<-0.5
        {
            //ROS_INFO("222");
            //说明正在导航途中，用自己的控制方法
            int NN = 0;
            int  path_lengh = global_path_can_look.poses.size();
            ROS_INFO("global_path_points_size=%d",path_lengh);
            if(path_lengh>0)
            {
                //ROS_INFO("777");
                //ROS_INFO("global_path_points_size=%d",path_lengh);
                if (path_lengh > 100)     //max_vx 0.3 :15
                {
                    path_lengh= 100;
                }
                Eigen::VectorXd x_veh(path_lengh);
                Eigen::VectorXd y_veh(path_lengh);
                int logo[path_lengh];
                memset(logo, 0, sizeof(logo));
                double x_veh_max = 0;
                double distance2_max=0;
                for (int i = 0; i < path_lengh; i++)
                {
                    const double dx= global_path_can_look.poses[i].pose.position.x - px; //1~N
                    const double dy = global_path_can_look.poses[i].pose.position.y - py;
                    x_veh[i] = dx * cospsi + dy * sinpsi;
                    y_veh[i] = dy * cospsi - dx * sinpsi;
                    double distance2=x_veh[i]*x_veh[i]+y_veh[i]*y_veh[i];
                    if(distance2>=(pre_length*pre_length))
                    {
                        logo[NN] = i; //记录数组下标
                        distance2_max = distance2;
                        //NN++;
                        break;
                    }
                }
                x_veh_max = 0;
                //得 到新的有效数组X_veh、Y_veh，和有效个数N
                Eigen::VectorXd XX_veh(NN);
                Eigen::VectorXd YY_veh(NN);

                //auto coeffs = polyfit(XX_veh, YY_veh, 1); //这里的x被auto推断为double类型c11标准//hhh
                //cout<<"k="<<coeffs[1]<<"                       b="<<coeffs[0]+(L)*coeffs[1]<<endl;
                ////参数：90，15；
                kk = 1900; //1850
                bb = 10;
                //this->nowDeviation = ((coeffs[1]) * kk + (coeffs[0] + (this->L)*coeffs[1]) * bb) / (kk + bb);
                //d_k = coeffs[1];
                //d_b = coeffs[0];
                this->nowDeviation=atan2(2*L*sin(atan2(y_veh[logo[0]],x_veh[logo[0]])),pre_length);
                this->turn_v = nowDeviation * kp_inside;
                if (this->turn_v > turn_v_max)
                {
                    this->turn_v = turn_v_max;
                }
                if (this->turn_v < -1*turn_v_max)
                {
                    this->turn_v = -1*turn_v_max;
                }
                this->Vx  = this->maxspeed; //0.3
                cmd_vel_now.linear.x = this->Vx;
                cmd_vel_now.linear.y = this->Vy;
                cmd_vel_now.angular.z = this->turn_v;
                cmd_vel_pub.publish(this->cmd_vel_now);
                //ROS_INFO("333");
            }
            else
            {
                cmd_vel_pub.publish(this->fake_vel_now);
                //ROS_INFO("444");
            }
        }
        else
        {
            cmd_vel_pub.publish(this->fake_vel_now);
            //ROS_INFO("555");
        }
    }
    else  //其他情况就发送原始速度
    {
        cmd_vel_pub.publish(this->fake_vel_now);
        //ROS_INFO("666");
    }
}


void PurePursuit::amcl_poseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_poseMsg)
{

    this->amcl_pose_now=*amcl_poseMsg;
    tf::Pose pose_tf;
    tf::poseMsgToTF(amcl_pose_now.pose.pose, pose_tf);
    px=amcl_pose_now.pose.pose.position.x;
    py=amcl_pose_now.pose.pose.position.y;
    double psi = tf::getYaw(pose_tf.getRotation());
    // this->now_yaw = psi;
    // if(this->now_yaw>3.14 ){this->now_yaw=this->now_yaw-3.14;}
    // if(this->now_yaw<=-3.14 ){this->now_yaw=this->now_yaw+3.14;}
    this->cospsi = cos(psi);
    this->sinpsi = sin(psi);
}
void PurePursuit::ucar_stateCB(const std_msgs::Int8::ConstPtr &ucar_stateMsg)
{
    this->ucar_state_now=*ucar_stateMsg;
}

void PurePursuit::cv_mode_callback(const std_msgs::Int8& msg)
{
  this->cv_mode.data= msg.data;
}
void PurePursuit::odomCB(const nav_msgs::Odometry & msg)
{
    // this->odom1=msg;
    // tf::Pose pose_tf;
    // tf::poseMsgToTF(odom1.pose.pose, pose_tf);
    // double psi = tf::getYaw(pose_tf.getRotation());
    // this->now_yaw = psi;
    // if(this->now_yaw>3.14 ){this->now_yaw=this->now_yaw-3.14;}
    // if(this->now_yaw<=-3.14 ){this->now_yaw=this->now_yaw+3.14;}
    // cout<<now_yaw<<endl;
}

void PurePursuit::imuCB(const std_msgs::Float32 & msg)
{
    
    this->imu_data=msg;
    this->now_yaw = imu_data.data;
    if(this->now_yaw>3.14 ){this->now_yaw=this->now_yaw-6.28;}
    if(this->now_yaw<=-3.14 ){this->now_yaw=this->now_yaw+6.28;}
}

void PurePursuit::global_pathCB(const nav_msgs::Path::ConstPtr &global_pathMsg)
{
    this->global_path_can_look = *global_pathMsg;
}

void PurePursuit::fake_velCB(const geometry_msgs::Twist::ConstPtr &fake_velMsg)
{
    //  cout<<"fake_velCB is here "<<endl;
    this->fake_vel_now = *fake_velMsg;
}



int main(int argc, char **argv)
{
	setlocale(LC_ALL,"");
	ros::init(argc, argv, "ucar_pure_pursuit_control");
	ros::NodeHandle n("~");
    double rate=20; //需要在PurePursuit同步修改
    ros::Rate loopRate(rate);
    PurePursuit purepursuit(&n);
    int aaa=1;
    ros::Rate loop_rate(1000);
    while(ros::ok) // && purepursuit.cv_mode.data!=NULL
    {
        //******************************************************************************************************
//区域E
        if(status_arrived_areaE_1== purepursuit.cv_mode.data )
        {   
            purepursuit.areaE_2_ticks++;   
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaE_2; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaE_2_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaE_2_1+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaE_2;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaE_2_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaE_3== purepursuit.cv_mode.data )
        {   
            purepursuit.areaE_2_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaE_2; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaE_2_2-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaE_2_2+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaE_4;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaE_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        // else if(status_arrived_areaE_5== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaE_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaE; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaE_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaE_3+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaE_6;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         // purepursuit.areaE_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaD_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaD_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaD; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     cout<<purepursuit.now_yaw<<endl;
        //     if(((purepursuit.areaD_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaD_4+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaD_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
//******************************************************************************************************
//区域D
        if(status_arrived_areaD_1== purepursuit.cv_mode.data )
        {   
            purepursuit.areaD_ticks++;   
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaD; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaD_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaD_1+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaD_2;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaD_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaD_3== purepursuit.cv_mode.data )
        {   
             ROS_INFO("d3333333");
            purepursuit.areaD_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaD; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaD_2-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaD_2+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaD_4;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaD_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaD_5== purepursuit.cv_mode.data )
        {   
            purepursuit.areaD_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaD; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaD_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaD_3+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaD_6;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaD_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        // else if(status_arrived_areaD_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaD_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaD; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     cout<<purepursuit.now_yaw<<endl;
        //     if(((purepursuit.areaD_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaD_4+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaD_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
    //*****************************************************************************************************************
    //区域C
        else if(status_arrived_areaC_1== purepursuit.cv_mode.data )
        {   
            purepursuit.areaC_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaC_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_1+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaC_2;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaC_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaC_3== purepursuit.cv_mode.data )
        {   
            purepursuit.areaC_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaC_2-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_2+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaC_4;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaC_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaC_5== purepursuit.cv_mode.data )
        {   
            purepursuit.areaC_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaC_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_3+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaC_6;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaC_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        // else if(status_arrived_areaC_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaC_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaC_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_4+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaC_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         // purepursuit.areaC_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }

    //*****************************************************************************************************************
    //区域C第二个点
        // else if(status_arrived_areaC_1_second== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaC_ticks_2++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC_2; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaC_2_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_2_1+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaC_2_second;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         purepursuit.areaC_ticks_2=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaC_3_second== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaC_ticks_2++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC_2; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaC_2_2-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_2_2+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaC_4_second;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         purepursuit.areaC_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaC_5_second== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaC_ticks_2++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC_2; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaC_2_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_2_3+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaC_6_second;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         // purepursuit.areaC_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaC_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaC_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaC; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaC_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaC_4+0.10)))
        //     {
                
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaC_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
    //*****************************************************************************************************************
    //区域B
        else if(status_arrived_areaB_1== purepursuit.cv_mode.data )
        {   
            purepursuit.areaB_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaB_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_1+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaB_2;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaB_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaB_3== purepursuit.cv_mode.data )
        {   
            purepursuit.areaB_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaB_2-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_2+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaB_4;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaB_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaB_5== purepursuit.cv_mode.data )
        {   
            purepursuit.areaB_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaB_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_3+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                sleep(sleep_time);
                purepursuit.cv_mode.data= status_arrived_areaB_6;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaB_ticks=0;
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        // else if(status_arrived_areaB_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaB_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaB_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_4+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaB_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         // purepursuit.areaB_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }


    //*****************************************************************************************************************
    //区域B第二个点
        // else if(status_arrived_areaB_1_second== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaB_ticks_2++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB_2; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaB_2_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_2_1+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaB_2_second;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         purepursuit.areaB_ticks_2=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaB_3_second== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaB_ticks_2++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB_2; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaB_2_2-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_2_2+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaB_4_second;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         purepursuit.areaB_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaB_5_second== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaB_ticks_2++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB_2; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaB_2_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_2_3+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaB_6_second;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         // purepursuit.areaB_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaB_5== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaB_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaB_3-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_3+0.10)))
        //     {
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaB_6;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         purepursuit.areaB_ticks=0;
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        // else if(status_arrived_areaB_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaB_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaB; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaB_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaB_4+0.10)))
        //     {
                
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         sleep(sleep_time);
        //         purepursuit.cv_mode.data= status_arrived_areaB_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
    //*****************************************************************************************************************
    //区域F
        else if(status_arrived_areaF_1== purepursuit.cv_mode.data )
        {   
            purepursuit.areaF_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaF; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaF_1-0.08)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaF_1+0.08)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                purepursuit.cv_mode.data= status_arrived_areaF_2;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaF_ticks=0;
                ROS_INFO("F1");
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaF_3== purepursuit.cv_mode.data )
        {   
            purepursuit.areaF_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaF; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaF_2-0.08)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaF_2+0.08)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                purepursuit.cv_mode.data= status_arrived_areaF_4;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaF_ticks=0;
                ROS_INFO("F2");
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        else if(status_arrived_areaF_5== purepursuit.cv_mode.data )
        {   
            purepursuit.areaF_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaF; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaF_3-0.08)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaF_3+0.08)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                purepursuit.cv_mode.data= status_arrived_areaF_6;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaF_ticks=0;
                ROS_INFO("F3");
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
         else if(status_arrived_areaF_7== purepursuit.cv_mode.data )
        {   
            purepursuit.areaF_ticks++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaF; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaF_4-0.08)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaF_4+0.08)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                purepursuit.cv_mode.data= status_arrived_areaF_8;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                purepursuit.areaF_ticks=0;
                ROS_INFO("F4");
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        //*****************************************************************************************************************
        //区域F第二个点
        else if(status_arrived_areaF_1_second== purepursuit.cv_mode.data )
        {   
            purepursuit.areaF_ticks_2++;
            purepursuit.fake_vel_now.linear.x = 0;
            purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaF_2; // 设置角速度，正为左转，负为右转
            purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
            if(((purepursuit.areaF_2_1-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaF_2_1+0.10)))
            {
                purepursuit.fake_vel_now.linear.x = 0;
                purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
                purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
                purepursuit.cv_mode.data= status_arrived_areaF_2_second;
                purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
                // purepursuit.areaF_ticks=0;
                ROS_INFO("F_2_1");
                cout<<purepursuit.now_yaw<<endl;
            }
            ros::spinOnce();
            loop_rate.sleep();	
        }
        // else if(status_arrived_areaF_7== purepursuit.cv_mode.data )
        // {   
        //     purepursuit.areaF_ticks++;
        //     purepursuit.fake_vel_now.linear.x = 0;
        //     purepursuit.fake_vel_now.angular.z = purepursuit.turn_v_areaF; // 设置角速度，正为左转，负为右转
        //     purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //     if(((purepursuit.areaF_4-0.10)<purepursuit.now_yaw) && (purepursuit.now_yaw<(purepursuit.areaF_4+0.10)))
        //     {
                
        //         purepursuit.fake_vel_now.linear.x = 0;
        //         purepursuit.fake_vel_now.angular.z = 0; // 设置角速度，正为左转，负为右转
        //         purepursuit.cmd_vel_pub.publish(purepursuit.fake_vel_now);
        //         purepursuit.cv_mode.data= status_arrived_areaF_8;
        //         purepursuit.ucar_state_for_cv_pub.publish(purepursuit.cv_mode);
        //         ROS_INFO("F4");
        //         cout<<purepursuit.now_yaw<<endl;
        //     }
        //     ros::spinOnce();
            // loop_rate.sleep();	
        // }
        else
        {
            ros::spinOnce();
            loop_rate.sleep();	
        }
    }
	ros::spin();

    return 0;

}

#include "../include/visual_headfile.h"

using namespace cv;
using namespace std;
#define VIDEO_OPEN 0
bool received_flag = false;
double angular_velocity_z = 0.0;
double integrated_angle = 0.0;
float __avoid_dist = 0.5;
int turn_num = 0, turn_1 = 1, turn_2 = 1;
double vel_max = 0.7, par = 4.4;
double vel_mine = 0.05,Dist = 0;
int obstacle = 0,__Avoid = 0,target_dir = 1;
double scan_data[240];
double footprint_width = 0.256;
double footprint_length = 0.42;
nav_msgs::Odometry last_odom_data;

/////////
#if VIDEO_OPEN
std::string filename = "/home/ucar/Desktop/ucar_Video/FL/";
std::time_t now = std::time(0);
tm *ltm = localtime(&now);
String T_path = filename + to_string(int(ltm->tm_mon)) + "_" + to_string(int(ltm->tm_mday)) + "_" + to_string(int(ltm->tm_hour)) + "_" + (to_string(int(ltm->tm_min)) + ".mp4");
cv::VideoWriter Video_Out(T_path, VideoWriter::fourcc('H', '2', '6', '4'), 30, cv::Size(320, 240));
#endif
////////
double max_m(double a,double b,double c)
{
    if(a>=b && a>=c)
        return a;
    else if(b>=a && b>=c)
        return b;
    else return c;
}

void brake(ros::Publisher &pub)
{
    geometry_msgs::Twist vel;
    while(last_odom_data.twist.twist.linear.x > 0)
    {
        geometry_msgs::Twist vel;
        vel.linear.x = -0.5;
        vel.linear.y = 0;
        vel.angular.z = 0;
        pub.publish(vel);
    }
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    pub.publish(vel);
}
double min_m(double a,double b,double c)
{
    if(a<=b && a<=c)
        return a;
    else if(b<=a && b<=c)
        return b;
    else return c;
}
//////
void startCallback(const std_msgs::Int32 &msg)
{
    // Trace_edge.data = msg.data;
}
//////
void LidarCallback(const sensor_msgs::LaserScan &msg)
{
    // if (msg->ranges[] < 0.5)
    //     obstacle = 1;
    float min_x = 100;

    for (int i = 0; i < 240; i++) {
        scan_data[i] = msg.ranges[258 + i];
        if(i>110 && i<130)
        {
            if(min_x > scan_data[i] && scan_data[i] != 0)
                min_x = scan_data[i];
        }
    }
    if(min_x < __avoid_dist)
    {
        __Avoid = 1;
        obstacle = 1;
    }
}

void PoseCallback(const nav_msgs::OdometryConstPtr &msg)
{

    float Dist_integral = 0;
    if(__Avoid == 0)
    {  
        last_odom_data = *msg;
    }
    else 
    {   
        Dist_integral = sqrt(pow((msg->pose.pose.position.x-last_odom_data.pose.pose.position.x),2)+pow((msg->pose.pose.position.y-last_odom_data.pose.pose.position.y),2));
        Dist += Dist_integral;
        last_odom_data = *msg;
    }
}
void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    angular_velocity_z = msg->angular_velocity.z;
}
int avoidance_control(ros::Publisher &pub)
{
    if(__Avoid == 0)
        return 0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    pub.publish(vel);
    // brake(pub);
    ROS_INFO("//__Avoid__\\\\");
    ros::Rate loop_rate(50);
    int i,j,__back =0;
    double dist_min = scan_data[120];
    double target_dist = 0;
    for(i = 1;i<20;i++)
        if(min_m(dist_min,scan_data[120-i],scan_data[120+i]) != 0)
            dist_min = min_m(dist_min,scan_data[120-i],scan_data[120+i]);
    ROS_WARN("dist_min: %f",dist_min);
    double dist_max = dist_min;


    for(i=1;i<60;i++)
    {
        double max = 0;

        if(max_m(max,scan_data[120-i],scan_data[120+i]) > 10)
            continue;
        max = max_m(max,scan_data[120-i],scan_data[120+i]);
        if(max > 1 || max - dist_max > 0.1)
            break;
        dist_max = max;
    }
    ROS_WARN("dist_max: %f",dist_max);
    target_dist = footprint_width/2 + sqrt(pow(dist_max,2)-pow(dist_min,2));

    vel.linear.x = 0;
    vel.linear.y = 0.4;
    //vel.linear.y = target_dir * (vel.linear.x * target_dist/dist_min+0.1);
    pub.publish(vel);
    

    while (ros::ok())
    {
        if( Dist >0.1 + target_dist && __back == 0)
        {
            Dist = 0;
            vel.linear.x = 0.5;
            vel.linear.y = 0;
            pub.publish(vel);
            __back = 1;
        }
        if(Dist > footprint_length && __back == 1)
        {
            Dist = 0;
            vel.linear.x = 0;
            vel.linear.y = -0.4;
            //vel.linear.y = -target_dir * (vel.linear.x * target_dist/dist_min+0.1);
            pub.publish(vel);
            __back = 2;
        }
        if( Dist >0.05 + target_dist && __back == 2)
        {
            Dist = 0;
            vel.linear.x = 0;
            vel.linear.y = 0;
            pub.publish(vel);
            break;
        }
        pub.publish(vel);

        loop_rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("\\\\__Avoid__//");
    __Avoid = 0;
    return 0;
}
void PublishTwist(ros::Publisher &pub, double linear_x, double angular_z)
{
    if(__Avoid == 0)
    {
        geometry_msgs::Twist twist;
        twist.linear.x = linear_x;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = angular_z;
        pub.publish(twist);
    }
}

double my_abs(double x)
{
    if (x > 0)
        return x;
    else
        return -x;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "visual_navigation");
    ros::NodeHandle nh;

    nh.param<double>("vel_max_", vel_max, 0.7);
    // nh.param<double>("par_", par, 4.4);
    nh.param<double>("vel_mine_", vel_mine, 0.05);


    PID_Parameter_Init();
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, ImuCallback);
    //////
    // ros::Subscriber visiual_sub = nh.subscribe("/visual_nav", 1, startCallback);

    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, LidarCallback);
    //////
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher end_pub = nh.advertise<std_msgs::Int32>("/visual_nav_end", 10);
    ros::Subscriber odom_sub = nh.subscribe("/amcl_pose", 1, PoseCallback);
    ros::Rate loop_rate(60);
    //////
    // std_msgs::Int32::ConstPtr Trace_edge = ros::topic::waitForMessage<std_msgs::Int32>("/visual_nav",nh);
    std_msgs::Int32::ConstPtr Trace_edge = NULL;
    //////    
    double time = ros::Time::now().toSec();
    cv::VideoCapture capture(0);
    ROS_ERROR("open camera takes %f",ros::Time::now().toSec()-time);
    if (!capture.isOpened())
    {
        ROS_ERROR("摄像头启动失败！");
        return -1;
    }
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    // capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('H', '2', '6', '4'));


    Mat original_frame;
    // original_frame = Mat::zeros(160,120, CV_8UC3);
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;
    std::cout << "Frame width: " << capture.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "Frame height: " << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    ROS_INFO("--------------------------START-------------------------------");
    capture >> original_frame;
    return 0;
    while (ros::ok())
    {
        if(obstacle == 0)
        {
            capture >> original_frame;
        ////
#if VIDEO_OPEN
            Video_Out.write(original_frame);
#endif
        ////
            Canny_Method(original_frame, 50, 150,Trace_edge->data);

            Error_Calculation();
            Speed_Control(0.05, 0.035, vel_max);

            PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);

        }
        else if(obstacle == 1)
        {
            avoidance_control(pub);
            obstacle = 2;
        }
        else if(obstacle == 2)
        {
            capture >> original_frame;
            Canny_Method(original_frame, 50, 150,3);

            Error_Calculation(nh);
            Speed_Control(0.05, 0.035, vel_max);

            PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);

            if (waitKey(1) >= 0)
            {
                PublishTwist(pub, 0.0, 0.0);
                std_msgs::Int32 end_msg;
                end_msg.data = 1;
                end_pub.publish(end_msg);
                break;
                // ROS_INFO("已退出！");
                // system("play /home/ucar/ucar_test/src/xf_mic_asr_offline/audio/finish.wav");
                // break;
            }

        }
        ros::spinOnce();
        loop_rate.sleep();
    }//while end
    return 0;
}

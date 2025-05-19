// #include "../include/visual_headfile.h"
// // #include <cv_bridge/cv_bridge.h>
// using namespace cv;
// using namespace std;

// bool received_flag = false;
// double angular_velocity_z = 0.0;
// double integrated_angle = 0.0;
// float __avoid_dist = 0.4;
// int turn_num = 0, turn_1 = 1, turn_2 = 1;
// double vel_max = 0.7, par = 4.4;
// double vel_start = 0.05, Dist = 0,acceleration = 0.035;
// int obstacle = 0, __Avoid = 0, target_dir = 1;
// double scan_data[240];
// double footprint_width = 0.34;
// double footprint_length = 0.4;
// double diff_angle = 0.0,delta_theta = 0.0,front_dist = 1.0,dist_min = 0.0;

// Mat image;
// /////////

// ////////
// double max_m(double a, double b, double c)
// {
//     if (a >= b && a >= c)
//         return a;
//     else if (b >= a && b >= c)
//         return b;
//     else
//         return c;
// }

// class PID{
//     public:

//     PID(double Kp, double Ki, double Kd)
//     {
//         this->Kp = Kp;
//         this->Ki = Ki;
//         this->Kd = Kd;
//         this->last_error = 0.0;
//         this->integral = 0.0;
//         this->derivative = 0.0;
//     }

//     double update(double target, double current)
//     {
//         double error = target - current;
//         integral += error;
//         derivative = error - last_error;
//         last_error = error;
//         return Kp * error + Ki * integral + Kd * derivative;
//     }

//     void clear()
//     {
//         this->integral = 0.0;
//         this->derivative = 0.0;
//         this->last_error = 0.0;
//     }
//     private:
//     double Kp, Ki, Kd;
//     double integral = 0.0;
//     double derivative = 0.0;
//     double last_error = 0.0;

// };
// double min_m(double a, double b, double c)
// {
//     if (a <= b && a <= c)
//         return a;
//     else if (b <= a && b <= c)
//         return b;
//     else
//         return c;
// }
// //////
// void startCallback(const std_msgs::Int32 &msg)
// {
//     // Trace_edge.data = msg.data;
// }
// //////
// void LidarCallback(const sensor_msgs::LaserScan &msg)
// {

//     // ROS_INFO("LidarCallback");
//     float min_x = 100;
//     if (msg.ranges[454] != INFINITY)
//         front_dist = msg.ranges[454];
//     int min_index = 0;
//     // for (int i = 0; i < 250; i++)
//     // {
        
//     //     scan_data[i] = msg.ranges[329 + i];
//     //     if (i > 110 && i < 130)
//     //     {
//     //         if (min_x > scan_data[i] && scan_data[i] != 0)
//     //         {
//     //             min_x = scan_data[i];
//     //             min_index = i + 329;
//     //         }
//     //     }
//     // }
//     // if (min_x < __avoid_dist && obstacle == 0)
//     if (front_dist < __avoid_dist && obstacle == 0)
//     {
//         diff_angle = (min_index - 454) * 3.1415926 / 909;

//         dist_min = min_x;
//         __Avoid = 1;
//         obstacle = 1;
//     }
//     // ROS_INFO("LidarCallback end");
// }
// void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
// {
//     static double last_angular_velocity_z = 0.0;

//     double temp = 0.6 * msg->angular_velocity.z + (1 - 0.6) * last_angular_velocity_z;
//     delta_theta += temp / 50;
//     last_angular_velocity_z = msg->angular_velocity.z;

// }
// void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
// {
//     // ROS_INFO("OdomCallback");
//     static nav_msgs::Odometry last_odom_data;
//     float Dist_integral = 0;
//     if (__Avoid == 0)
//     {
//         last_odom_data = *msg;
//     }
//     else
//     {
//         Dist_integral = sqrt(pow((msg->pose.pose.position.x - last_odom_data.pose.pose.position.x), 2) + pow((msg->pose.pose.position.y - last_odom_data.pose.pose.position.y), 2));
//         Dist += Dist_integral;
//         last_odom_data = *msg;
//     }
//     // ROS_INFO("OdomCallback end");
// }
// // void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
// // {
// //     image = cv_bridge::toCvShare(msg, "bgr8")->image;
// //     // ROS_INFO("image size: %d, %d", image.rows, image.cols);
// // }





// void Rorate(double theta,ros::Publisher &pub)
// {
//     delta_theta = 0;
//     ros::Rate loop_rate(50);
//     PID rorate_PID(3, 0.0, 0.3);
//     theta = theta * 3.1415926 / 180;
//     geometry_msgs::Twist vel;
//     while(abs(delta_theta - theta) > 0.05 and abs(angular_velocity_z) > 0.01)
//     {
        
//         vel.angular.z = rorate_PID.update(theta, delta_theta);
//         pub.publish(vel);
//         loop_rate.sleep();
//         ros::spinOnce();
//     }
//     vel.angular.z = 0;
//     pub.publish(vel);

// }





// // void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
// // {
// //     angular_velocity_z = msg->angular_velocity.z;
// // }
// int avoidance_control(ros::Publisher &pub)
// {
//     if (__Avoid == 0)
//         return 0;
//     geometry_msgs::Twist vel;
//     vel.linear.x = 0;
//     vel.linear.y = 0;
//     vel.angular.z = 0;
//     pub.publish(vel);
//     // brake(pub);
//     ros::Rate loop_rate(50);
//     ROS_INFO("//__Avoid__\\\\");

//     Rorate(diff_angle,pub);

//     ros::spinOnce();



//     int i, j, __back = 0;
//     // double dist_min = scan_data[120];
//     double target_dist = 0;
//     // for (i = 1; i < 20; i++)
//     //     if (min_m(dist_min, scan_data[120 - i], scan_data[120 + i]) != 0)
//     //         dist_min = min_m(dist_min, scan_data[120 - i], scan_data[120 + i]);
//     // ROS_WARN("dist_min: %f", dist_min);
//     // double dist_max = dist_min;

//     // for (i = 1; i < 60; i++)
//     // {
//     //     double max = 0;

//     //     if (max_m(max, scan_data[120 - i], scan_data[120 + i]) > 10)
//     //         continue;
//     //     max = max_m(max, scan_data[120 - i], scan_data[120 + i]);
//     //     if (max > 1 || max - dist_max > 0.1)
//     //         break;
//     //     dist_max = max;
//     // }
//     // ROS_WARN("dist_max: %f", dist_max);


//     target_dist = footprint_width / 2 + 0.25;
//     vel.linear.x = 0;
//     vel.linear.y = 0.4;
//     // vel.linear.y = target_dir * (vel.linear.x * target_dist/dist_min+0.1);
//     pub.publish(vel);
//     double length = front_dist;
//     std::cout << "footprint_length: " << footprint_length << std::endl;
//     std::cout << "dist_min: " << dist_min << std::endl;

//     while (ros::ok())
//     {
//         if (Dist > 0.1 + target_dist && __back == 0)
//         {
//             Dist = 0;
//             vel.linear.x = 0.5;
//             vel.linear.y = 0;
//             pub.publish(vel);
//             __back = 1;
//         }
//         if (Dist > footprint_length  && __back == 1)
//         {
//             Dist = 0;
//             vel.linear.x = 0;
//             vel.linear.y = -0.4;
//             // vel.linear.y = -target_dir * (vel.linear.x * target_dist/dist_min+0.1);
//             pub.publish(vel);
//             __back = 2;
//         }
//         if (Dist > target_dist +0.05 && __back == 2)
//         {
//             Dist = 0;
//             vel.linear.x = 0;
//             vel.linear.y = 0;
//             pub.publish(vel);
//             break;
//         }
//         pub.publish(vel);

//         loop_rate.sleep();
//         ros::spinOnce();
//     }
//     ROS_INFO("\\\\__Avoid__//");
//     __Avoid = 0;
//     return 0;
// }
// void PublishTwist(ros::Publisher &pub, double linear_x, double angular_z)
// {
//     if (__Avoid == 0)
//     {
//         geometry_msgs::Twist twist;
//         twist.linear.x = linear_x;
//         twist.linear.y = 0.0;
//         twist.linear.z = 0.0;
//         twist.angular.x = 0.0;
//         twist.angular.y = 0.0;
//         twist.angular.z = angular_z;
//         pub.publish(twist);
//     }
// }

// double my_abs(double x)
// {
//     if (x > 0)
//         return x;
//     else
//         return -x;
// }

// int main(int argc, char **argv)
// {
//     // std::cout << "opencv_version: " << CV_VERSION << std::endl;
//     ros::init(argc, argv, "visual_navigation");
//     ros::NodeHandle nh;
//     double highthreshold = 90;
//     double lowthreshold = 50;
//     bool VIDEO_OPEN = false;
//     std::string video_path = "/home/ucar/Videos/";
    
//     nh.param<bool>("VIDEO_OPEN_", VIDEO_OPEN, false);
//     nh.param<std::string>("video_path", video_path, "/home/ucar/Videos/");
//     nh.param<double>("highthreshold", highthreshold, 90);
//     nh.param<double>("lowthreshold", lowthreshold, 50);
//     nh.param<double>("vel_max_", vel_max, 0.7);
//     nh.param<double>("acceleration_", acceleration, 0.035);
//     // nh.param<double>("par_", par, 4.4);
//     nh.param<double>("vel_start_", vel_start, 0.05);


//     VideoWriter Video_Out;
//     if (VIDEO_OPEN)
//     {
//         time_t now = time(0);
//         tm *ltm = localtime(&now);
//         string T_path = video_path + to_string(int(ltm->tm_mon)) + "_" + to_string(int(ltm->tm_mday)) + "_" + to_string(int(ltm->tm_hour)) + "_" + to_string(int(ltm->tm_min)) + ".mp4";
//         Video_Out.open(T_path, VideoWriter::fourcc('H', '2', '6', '4'), 30, Size(320, 240));
//         if (!Video_Out.isOpened()) {
//             ROS_WARN("无法创建视频文件: %s", T_path.c_str());
//             VIDEO_OPEN = false;
//         } else {
//             ROS_INFO("视频录制已启动: %s", T_path.c_str());
//         }
//     }





//     ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, LidarCallback);
//     ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
//     ros::Publisher end_pub = nh.advertise<std_msgs::Int32>("/visual_nav_end", 10);
//     ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdomCallback);
//     ros::Subscriber imu_sub = nh.subscribe("/imu", 1, ImuCallback);
//     ros::Rate loop_rate(60);
//     //////
//     std_msgs::Int32::ConstPtr Trace_edge = ros::topic::waitForMessage<std_msgs::Int32>("/visual_nav", nh);

//     //////
//     std::cout << "opening cap ..." << std::endl;
//     VideoCapture capture(0);
//     capture.set(CAP_PROP_FRAME_WIDTH, 160);
//     capture.set(CAP_PROP_FRAME_HEIGHT, 120);
//     capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('H', '2', '6', '4'));

//     if (!capture.isOpened())
//     {
//         ROS_ERROR("摄像头启动失败！");
//         return -1;
//     }

//     PID_Parameter_Init();

//     Mat original_frame;
//     capture >> original_frame;
//     ROS_ERROR_STREAM_ONCE("=====!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!======");

//     // ros::spinOnce();
//     // std::cout << "image size: " << image.size() << std::endl;
//     // return 0;
//     // Mat original_frame;

//     while (ros::ok())
//     {
//         capture >> original_frame;

//         if (VIDEO_OPEN)
//                 Video_Out.write(original_frame);


//         if (original_frame.empty())
//         {
//             ROS_ERROR("capture error");
//             continue;
//         }
//         if (obstacle == 0)
//         {

//             ////
            
            
//             ////
//             Canny_Method(original_frame, lowthreshold, highthreshold, Trace_edge->data);

//             Error_Calculation(nh);
//             Speed_Control(vel_start, acceleration, vel_max);

//             PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);
//         }
//         else if (obstacle == 1)
//         {
//             avoidance_control(pub);
//             obstacle = 2;
//         }
//         else if (obstacle == 2)
//         {
//             if (Canny_Method(original_frame, lowthreshold, highthreshold, 9))
//             {
//                 PublishTwist(pub, 0.0, 0.0);
//                 std_msgs::Int32 end_msg;
//                 end_msg.data = 1;
//                 end_pub.publish(end_msg);
//                 ROS_INFO("quit!");
//                 break;
//             }
//             Error_Calculation(nh);
//             Speed_Control(0.05, 0.035, vel_max);

//             PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);
//         }
//         ros::spinOnce();
//         loop_rate.sleep();
//     } // while end


//     if (VIDEO_OPEN)
//         Video_Out.release();
//     capture.release();
//     return 0;
// }








/*------------------------------------------------------------------------------------------------------------------------------------------*/
#include "../include/visual_headfile.h"
// #include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;
#define VIDEO_OPEN 0
bool received_flag = false;
double angular_velocity_z = 0.0;
double integrated_angle = 0.0;
float __avoid_dist = 0.45;
int turn_num = 0, turn_1 = 1, turn_2 = 1;
double vel_max = 0.7, par = 4.4;
double vel_mine = 0.05, Dist = 0;
int obstacle = 0, __Avoid = 0, target_dir = 1;
double scan_data[240];
double footprint_width = 0.34;
double footprint_length = 0.5;

Mat image;
/////////
#if VIDEO_OPEN
std::string filename = "/home/ucar/Desktop/ucar_Video/FL/";
std::time_t now = std::time(0);
tm *ltm = localtime(&now);
String T_path = filename + to_string(int(ltm->tm_mon)) + "_" + to_string(int(ltm->tm_mday)) + "_" + to_string(int(ltm->tm_hour)) + "_" + (to_string(int(ltm->tm_min)) + ".mp4");
cv::VideoWriter Video_Out(T_path, VideoWriter::fourcc('H', '2', '6', '4'), 30, cv::Size(320, 240));
#endif
////////
double max_m(double a, double b, double c)
{
    if (a >= b && a >= c)
        return a;
    else if (b >= a && b >= c)
        return b;
    else
        return c;
}

void brake(ros::Publisher &pub)
{
    // geometry_msgs::Twist vel;
    // while(last_odom_data.twist.twist.linear.x > 0)
    // {
    //     geometry_msgs::Twist vel;
    //     vel.linear.x = -0.5;
    //     vel.linear.y = 0;
    //     vel.angular.z = 0;
    //     pub.publish(vel);
    // }
    // vel.linear.x = 0;
    // vel.linear.y = 0;
    // vel.angular.z = 0;
    // pub.publish(vel);
}
double min_m(double a, double b, double c)
{
    if (a <= b && a <= c)
        return a;
    else if (b <= a && b <= c)
        return b;
    else
        return c;
}
//////
void startCallback(const std_msgs::Int32 &msg)
{
    // Trace_edge.data = msg.data;
}
//////
void LidarCallback(const sensor_msgs::LaserScan &msg)
{

    // ROS_INFO("LidarCallback");
    float min_x = 100;

    for (int i = 0; i < 250; i++)
    {
        scan_data[i] = msg.ranges[329 + i];
        if (i > 110 && i < 130)
        {
            if (min_x > scan_data[i] && scan_data[i] != 0)
                min_x = scan_data[i];
        }
    }
    if (min_x < __avoid_dist)
    {
        __Avoid = 1;
        obstacle = 1;
    }
    // ROS_INFO("LidarCallback end");
}

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("OdomCallback");
    static nav_msgs::Odometry last_odom_data;
    float Dist_integral = 0;
    if (__Avoid == 0)
    {
        last_odom_data = *msg;
    }
    else
    {
        Dist_integral = sqrt(pow((msg->pose.pose.position.x - last_odom_data.pose.pose.position.x), 2) + pow((msg->pose.pose.position.y - last_odom_data.pose.pose.position.y), 2));
        Dist += Dist_integral;
        last_odom_data = *msg;
    }
    // ROS_INFO("OdomCallback end");
}
// void ImageCallback(const sensor_msgs::ImageConstPtr &msg)
// {
//     image = cv_bridge::toCvShare(msg, "bgr8")->image;
//     // ROS_INFO("image size: %d, %d", image.rows, image.cols);
// }
void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    angular_velocity_z = msg->angular_velocity.z;
}
int avoidance_control(ros::Publisher &pub)
{
    if (__Avoid == 0)
        return 0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    pub.publish(vel);
    // brake(pub);
    ROS_INFO("//__Avoid__\\\\");
    ros::Rate loop_rate(50);
    int i, j, __back = 0;
    double dist_min = scan_data[120];
    double target_dist = 0;
    // for (i = 1; i < 20; i++)
    //     if (min_m(dist_min, scan_data[120 - i], scan_data[120 + i]) != 0)
    //         dist_min = min_m(dist_min, scan_data[120 - i], scan_data[120 + i]);
    // ROS_WARN("dist_min: %f", dist_min);
    // double dist_max = dist_min;

    // for (i = 1; i < 60; i++)
    // {
    //     double max = 0;

    //     if (max_m(max, scan_data[120 - i], scan_data[120 + i]) > 10)
    //         continue;
    //     max = max_m(max, scan_data[120 - i], scan_data[120 + i]);
    //     if (max > 1 || max - dist_max > 0.1)
    //         break;
    //     dist_max = max;
    // }
    // ROS_WARN("dist_max: %f", dist_max);
    target_dist = footprint_width / 2 + 0.25;

    vel.linear.x = 0;
    vel.linear.y = 0.4;
    // vel.linear.y = target_dir * (vel.linear.x * target_dist/dist_min+0.1);
    pub.publish(vel);

    while (ros::ok())
    {
        if (Dist > 0.1 + target_dist && __back == 0)
        {
            Dist = 0;
            vel.linear.x = 0.5;
            vel.linear.y = 0;
            pub.publish(vel);
            __back = 1;
        }
        if (Dist > footprint_length && __back == 1)
        {
            Dist = 0;
            vel.linear.x = 0;
            vel.linear.y = -0.4;
            // vel.linear.y = -target_dir * (vel.linear.x * target_dist/dist_min+0.1);
            pub.publish(vel);
            __back = 2;
        }
        if (Dist > target_dist +0.05 && __back == 2)
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
    if (__Avoid == 0)
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
    // std::cout << "opencv_version: " << CV_VERSION << std::endl;
    ros::init(argc, argv, "visual_navigation");
    ros::NodeHandle nh;

    nh.param<double>("vel_max_", vel_max, 0.7);
    // nh.param<double>("par_", par, 4.4);
    nh.param<double>("vel_mine_", vel_mine, 0.05);

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, ImuCallback);
    //////
    // ros::Subscriber visiual_sub = nh.subscribe("/visual_nav", 1, startCallback);

    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, LidarCallback);
    //////
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher end_pub = nh.advertise<std_msgs::Int32>("/visual_nav_end", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdomCallback);
    // ros::Subscriber image_sub = nh.subscribe("/visual_cam", 1, ImageCallback);
    ros::Rate loop_rate(60);
    //////
    std_msgs::Int32::ConstPtr Trace_edge = ros::topic::waitForMessage<std_msgs::Int32>("/visual_nav", nh);

    //////
    std::cout << "opening cap ..." << std::endl;
    VideoCapture capture(0);
    capture.set(CAP_PROP_FRAME_WIDTH, 160);
    capture.set(CAP_PROP_FRAME_HEIGHT, 120);
    capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('H', '2', '6', '4'));

    if (!capture.isOpened())
    {
        ROS_ERROR("摄像头启动失败！");
        return -1;
    }

    PID_Parameter_Init();

    Mat original_frame;
    capture >> original_frame;
    ROS_ERROR_STREAM_ONCE("=====!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!======");

    // ros::spinOnce();
    // std::cout << "image size: " << image.size() << std::endl;
    // return 0;
    // Mat original_frame;
    double highthreshold = 90;
    double lowthreshold = 50;
    while (ros::ok())
    {
        capture >> original_frame;
        if (original_frame.empty())
        {
            ROS_ERROR("capture error");
            continue;
        }
        if (obstacle == 0)
        {

            ////
#if VIDEO_OPEN
            // Video_Out.write(original_frame);
#endif
            ////
            Canny_Method(original_frame, lowthreshold, highthreshold, Trace_edge->data);

            Error_Calculation(nh);
            Speed_Control(0.05, 0.035, vel_max);

            PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);
        }
        else if (obstacle == 1)
        {
            avoidance_control(pub);
            obstacle = 2;
        }
        else if (obstacle == 2)
        {
            if (Canny_Method(original_frame, lowthreshold, highthreshold, 9))
            {
                PublishTwist(pub, 0.0, 0.0);
                std_msgs::Int32 end_msg;
                end_msg.data = 1;
                end_pub.publish(end_msg);
                ROS_INFO("quit!");
                break;
            }
            Error_Calculation(nh);
            Speed_Control(0.05, 0.035, vel_max);

            PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);
        }
        ros::spinOnce();
        loop_rate.sleep();
    } // while end
    capture.release();
    return 0;
}




/*------------------------------------------------------------------------------------------------------------------------------------------*/
#include "../include/visual_headfile.h"
// #include "../include/mask_L.h"
// #include "../include/mask_R.h"
// #include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;
bool received_flag = false;
double angular_velocity_z = 0.0;
double integrated_angle = 0.0;
double __avoid_dist = 0.45;
double Dist = 0,vel_max = 0.7,vel_start = 0.05,acceleration = 0.035;
int obstacle = 0,__Avoid = 0, target_dir = 1;
double scan_data[240] = {0};
double footprint_width = 0.34;
double footprint_length = 0.5;


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

void LidarCallback(const sensor_msgs::LaserScan &msg)
{

    // ROS_INFO("LidarCallback");
    float min_x = 100;

    for (int i = 0; i < 240; i++)
    {
        scan_data[i] = msg.ranges[329 + i];
        if (i > 110 && i < 130)
        {
            if (min_x > scan_data[i] && scan_data[i] != 0)
                min_x = scan_data[i];
        }
    }
    if (min_x < __avoid_dist and obstacle == 0)
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

void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // ROS_INFO("ImuCallback");
    static double accOdom = 0;
    angular_velocity_z = msg->angular_velocity.z;
    if(globalOdom == 1)
    {
        accOdom += angular_velocity_z / 30;
        if(abs(accOdom) > 5.7)
        globalOdom = 2;
    }

    // ROS_INFO("ImuCallback end");
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
        if (Dist > target_dist && __back == 2)
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
    // ROS_INFO("PublishTwist");
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
    // ROS_INFO("PublishTwist end");
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
    ros::NodeHandle nh("~"); 
    int Trace_edge = 0;
    double highthreshold = 90;
    double lowthreshold = 50;
    float KP = 15.0,KI = 0.0,KD = 108.0;
    bool VIDEO_OPEN = false;
    bool Debug = false;
    std::string video_path = "/home/ucar/Videos/";
    

    nh.param<bool>("VIDEO_OPEN_", VIDEO_OPEN, true);
    nh.param<std::string>("video_path", video_path, "/home/ucar/Videos/");
    nh.param<double>("highthreshold", highthreshold, 90);
    nh.param<double>("lowthreshold", lowthreshold, 50);
    nh.param<double>("vel_max_", vel_max, 0.7);
    nh.param<double>("acceleration_", acceleration, 0.035);
    nh.param<double>("vel_start_", vel_start, 0.05); 
    nh.param<double>("Avoid_Dist", __avoid_dist, 0.45);
    nh.param<int>("Trace_edge", Trace_edge, 0);
    nh.param<float>("KP",KP,15.0);
    nh.param<float>("KI",KI,0.0);
    nh.param<float>("KD",KD,108.0);  
    nh.param<bool>("Debug", Debug, false);



    std::cout << "--------------------------------" << std::endl;
    std::cout << "VIDEO_OPEN: " << VIDEO_OPEN << std::endl;
    std::cout << "video_path: " << video_path << std::endl;
    std::cout << "highthreshold: " << highthreshold << std::endl;
    std::cout << "lowthreshold: " << lowthreshold << std::endl;
    std::cout << "vel_max_: " << vel_max << std::endl;
    std::cout << "acceleration_: " << acceleration << std::endl;
    std::cout << "vel_start_: " << vel_start << std::endl;
    std::cout << "Avoid_Dist: " << __avoid_dist << std::endl;
    std::cout << "Trace_edge: " << Trace_edge << std::endl;
    std::cout << "KP: " << KP << std::endl;
    std::cout << "KI: " << KI << std::endl;
    std::cout << "KD: " << KD << std::endl;
    std::cout << "Debug: " << Debug << std::endl;
    std::cout << "--------------------------------" << std::endl;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, ImuCallback);
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1, LidarCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher end_pub = nh.advertise<std_msgs::Int32>("/visual_nav_end", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, OdomCallback);
    ros::Rate loop_rate(30);

    
    if (Trace_edge == 0)
        Trace_edge = ros::topic::waitForMessage<std_msgs::Int32>("/visual_nav", nh)->data;

    
        

    std::cout << "opening cap ..." << std::endl;
    VideoCapture capture(0);
    capture.set(CAP_PROP_FRAME_WIDTH, 320);
    capture.set(CAP_PROP_FRAME_HEIGHT, 240);
    capture.set(CAP_PROP_FPS, 30);

    Mat original_frame;
    capture >> original_frame;
    // int width = original_frame.cols;
    // int height = original_frame.rows;

    VideoWriter Video_Out;
    string T_path;

    if (VIDEO_OPEN)
    {
        time_t now = time(0);
        tm *ltm = localtime(&now);
        T_path = video_path + to_string(int(ltm->tm_mon)+1) + "_" + to_string(int(ltm->tm_mday)) + "_" + to_string(int(ltm->tm_hour)) + "_" + to_string(int(ltm->tm_min)) + ".mp4";
        Video_Out.open(T_path, VideoWriter::fourcc('m', 'p', '4', 'v'), 30, Size(160, 120));
        if (!Video_Out.isOpened()) {
            ROS_WARN("can't open video file: %s", T_path.c_str());
            VIDEO_OPEN = false;
        } else {
            ROS_INFO("video recording start: %s", T_path.c_str());
        }
    }

    if (!capture.isOpened())
    {
        ROS_ERROR("can't open camera");
        return -1;
    }

    PID_Parameter_Init(KP,KI,KD);


    ROS_ERROR_STREAM_ONCE("=====!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!======");

    int flag = 0;
    while (ros::ok())
    {
        if (obstacle != 2)
            capture >> original_frame;
        else if(flag < 5 && obstacle == 2) 
        {
            capture >> original_frame;
            flag++;
            continue;
        }
        else 
            capture >> original_frame;
        if (original_frame.empty())
        {
            ROS_ERROR("capture error");
            continue;
        }

        if (obstacle == 0)
        {

            Canny_Method(original_frame, lowthreshold, highthreshold, Trace_edge,Debug);
            if (VIDEO_OPEN)
            {
                cvtColor(original_frame, original_frame, COLOR_GRAY2RGB);
                Video_Out.write(original_frame);
            }
            Error_Calculation(nh);
            Speed_Control(vel_start, acceleration, vel_max);

            PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);
        }
        else if (obstacle == 1)
        {
            avoidance_control(pub);
            obstacle = 2;
        }
        else if (obstacle == 2)
        {
   
            if (Canny_Method(original_frame, lowthreshold, highthreshold, 9,Debug))
            {
                // imwrite("~/ucar_ws/output.png",original_frame);   
                PublishTwist(pub, 0.0, 0.0);
                std_msgs::Int32 end_msg;
                end_msg.data = 1;
                end_pub.publish(end_msg);
                ROS_INFO("quit!");
                break;
            }
            if (VIDEO_OPEN)
            {
                cvtColor(original_frame, original_frame, COLOR_GRAY2RGB);
                Video_Out.write(original_frame);
            }
            Error_Calculation(nh);
            Speed_Control(vel_start, acceleration, vel_max);

            PublishTwist(pub, vehicle_linear_speed, vehicle_orientations);
        }

        ros::spinOnce();
        loop_rate.sleep();
    } // while end
    capture.release();

    Video_Out.release();
    ROS_INFO("video recorded to %s", T_path.c_str());
    return 0;
}

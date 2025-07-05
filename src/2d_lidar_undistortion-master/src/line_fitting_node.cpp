#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <pudding/Pudding.h>

// ... 省锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷 ...
ros::Publisher line_pub;
ros::Publisher yaw_pub;

std_msgs::Float32 yaw_data;
ros::Publisher pcl_pub_test;

const double MIN_RANGE = 0.3; // 锟斤拷小锟斤拷锟斤拷
const double MAX_RANGE = 2.0; // 锟斤拷锟斤拷锟斤拷
double ANG_RANGE = 35; // 锟角讹拷锟斤拷锟斤拷 锟斤拷40锟斤拷

double cv_yaw_left = -1,lidar_dis = 2.0,cv_yaw_right = -1,cv_yaw = -1;
int flag_todeal = 0,needrviz = 1;
int num_dis_left = 378,num_dis_right = 378,num_dis = 378,point = 0;

bool use = false;

sensor_msgs::LaserScan Scan_data;

sensor_msgs::PointCloud2 filtered_cloud_msg;

double m_fabs(double x)
{
    return x >= 0 ? x : -x  ;
}

void flagCB(const std_msgs::Int32& msg)
{
    if(msg.data == 1)
    {
        use = true;
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2& cloud_msg)
{
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // 去除无效点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // 创建模型系数对象和内点索引对象
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 法向量估计
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    
    if(flag_todeal == 2)            //向右变大
    {   
        point = 0;
        if(cv_yaw <= 0)
        {
            for (int i = num_dis-3-point; i < num_dis+3-point; i++)         //向左取
            {
                if (pcl::isFinite(cloud->points[i])) // 检查点是否有效
                {
                    filtered_cloud->points.push_back(cloud->points[i]);
                }
            }
        }
        else
        {
            for (int i = num_dis+3+point; i > num_dis-3+point; i--)
            {
                if (pcl::isFinite(cloud->points[i])) // 检查点是否有效
                {
                    filtered_cloud->points.push_back(cloud->points[i]);
                }
            }
        }

        // 再次去除无效点
        pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indices);

        // 检查 filtered_cloud 是否为空
        if (!filtered_cloud->points.empty())
        {
            ROS_ERROR("Filtered cloud is %d",filtered_cloud->points.size());
            // return;
        }

        /***************************************use line*****************************************/
        seg.setOptimizeCoefficients(true); // 开启系数优化
        seg.setModelType(pcl::SACMODEL_LINE); // 设置模型类型为直线
        seg.setMethodType(pcl::SAC_RANSAC); // 设置分割方法为RANSAC
        seg.setDistanceThreshold(0.01); // 设置距离阈值
        seg.setInputCloud(filtered_cloud); // 设置输入点云
        seg.segment(*inliers, *coefficients); // 执行分割

        yaw_data.data = std::atan2(-coefficients->values[3] ,coefficients->values[4]);
        if(yaw_data.data > M_PI/2)
        {
            yaw_data.data = yaw_data.data - M_PI; 
        }
        else if(yaw_data.data < -M_PI/2)
        {
            yaw_data.data = yaw_data.data + M_PI;
        }

        ROS_INFO("Normal[%d]: x=%f, y=%f, yaw=%f", num_dis, coefficients->values[3], coefficients->values[4], yaw_data.data);
        yaw_pub.publish(yaw_data);
        
        flag_todeal = 3;
        // use = false;

        /*******************************************显示******************************************/
        // RViz显示
        if(needrviz)
        {
            pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
            filtered_cloud_msg.header.frame_id = "laser_frame";
            pcl_pub_test.publish(filtered_cloud_msg);
            visualization_msgs::Marker line_marker;
            line_marker.header.frame_id = cloud_msg.header.frame_id;
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "line_fitting";
            line_marker.id = 0;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.05; // ��������
            line_marker.color.r = 1.0;
            line_marker.color.g = 0.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;

            geometry_msgs::Point p1, p2;
            p1.x = coefficients->values[0];
            p1.y = coefficients->values[1];
            p1.z = coefficients->values[2];
            // p2.x = normal.normal_x;
            // p2.y = normal.normal_y;
            // p2.z = normal.normal_z;
            p2.x = coefficients->values[0] + coefficients->values[4];
            p2.y = coefficients->values[1] - coefficients->values[3];
            p2.z = coefficients->values[2] + coefficients->values[5];

            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);

            line_pub.publish(line_marker);
        }
        
    }
    
}

/*鑾峰彇闆疯揪鏁版嵁寰楀埌鏂瑰悜涓婄殑璺濈鐢ㄦ潵婊ょ偣*/
void scanCallback(const sensor_msgs::LaserScan msg)
{
    int a = 0;
    if(flag_todeal == 1)
    {
        Scan_data = msg;      //      756
        // num_dis_left = 378 - (int)((3.14*cv_yaw_left/180) / Scan_data.angle_increment);
        // num_dis_right = 378 - (int)((3.14*cv_yaw_right/180) / Scan_data.angle_increment);
        num_dis = 378 - (int)((3.14*cv_yaw/180) / Scan_data.angle_increment);

        // a = std::sqrt(0.04+(Scan_data.ranges[num_dis])*(Scan_data.ranges[num_dis]) + 2*0.2*Scan_data.ranges[num_dis]*m_fabs(cos(3.14*cv_yaw/180)));
        // point = (int)(std::asin(0.2*m_fabs(sin(3.14*cv_yaw/180)/a))/Scan_data.angle_increment);
        // ROS_WARN("left num : %d right num : %d point : %d",num_dis_left,num_dis_right,point);
        // lidar_dis = Scan_data.ranges[num_dis + 17];
        // if(lidar_dis < 3.0)
        flag_todeal = 2;
    }
}

// void pic_positionCB(const pudding::Pudding::ConstPtr& pic)
// {
//     static int last_pic_data = -1;
//     if(pic->left != -1 && pic->right != -1 && flag_todeal == 0)
//     {
//         ROS_INFO("pic->left : %d",pic->left);                
//         ROS_INFO("pic->right : %d",pic->right);                
//         cv_yaw_left = -(0.125 * pic->left - 40);       
//         cv_yaw_right = -(0.125 * pic->right - 40);     
//         cv_yaw = -(0.125 * (pic->right+pic->left)/2 - 40);  
//         flag_todeal = 1;
        
//     }
//     // last_pic_data = pic.data;
// }

void pic_positionCB(const std_msgs::Int32 &pic)
{
    static int last_pic_data = -1;
    if(pic.data != -1 && flag_todeal == 0)
    {                  
        if(pic.data == last_pic_data)
        {
            ROS_INFO("%d",pic.data);                
            cv_yaw = -(0.125 * pic.data - 40);     
            flag_todeal = 1;
        }
        
    }
    last_pic_data = pic.data;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "line_fitting_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);
    ros::Subscriber point_sub = nh.subscribe("/lidar_undistortion/origin", 1, pointCloudCallback);
    ros::Subscriber scan_sub = nh.subscribe("/scan", 10, scanCallback);
    ros::Subscriber cv_position_sub = nh.subscribe("/pic_position",100,pic_positionCB);
    pcl_pub_test = nh.advertise<sensor_msgs::PointCloud2> ("/lidar_undistortion/test", 10);
    yaw_pub=nh.advertise<std_msgs::Float32>("/yaw_fromscan", 1);
    ros::Subscriber flag_sub= nh.subscribe("/flagF", 10, flagCB);

    line_pub = nh.advertise<visualization_msgs::Marker>("line_marker", 1);
    while(ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
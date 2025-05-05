/*
 * @Author: AyefLev 592162794@qq.com
 * @Date: 2024-04-010 18:15:09
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-07-20 15:39:24
 * @FilePath: \srcc:\Users\AXS\AppData\Roaming\MobaXterm\slash\RemoteFiles\76798_9_4\get_pic.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "vision_ncnn.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctime>
#include <boost/thread.hpp>
#define VIDEO_OPEN 0
class ucar_vision : public Picodet_vision
{
public:
    // flag
    int detect_flag;
    float temp = 0;
    float distemp = 0;
    double fps[5] = {0, 0, 0, 0, 0};
    int iiii = 0;
    int jjjj = 0;
    float last_position = -1; // 上次的目标位置
    int once_label_num = 0;   //
    int NULL_num = 0;
    int turn_d = 0;
    int NULL_num_th = 1;
    float pudding_p = 0;
    /////
    ros::NodeHandle n_;
    ros::Publisher cv_label_pub;
    ros::Publisher cv_position_pub;

    ros::Subscriber flag_dp_sub;
    ros::Timer timer_contorller, timer_image;
    std_msgs::Int8 cv_mode_now;
    std_msgs::Int32 cv_pose_now;

    /////
    std_msgs::Float32 cv_line_now;
    /////

    ucar_vision(ros::NodeHandle *n);
    Clocassion CL = Clocassion();

#if VIDEO_OPEN
    std::string filename = "/home/ucar/Desktop/ucar_Video/TD/";
    cv::VideoWriter Video_Out;
#endif
    /////

    void get_image(const ros::TimerEvent &);
    void webcam(const ros::TimerEvent &);
    void flag_dp_CB(const std_msgs::Int32 &msg);
    float to_degree() { return (640 - CL.location) * 0.195 - 62.4; };

    ~ucar_vision()
    {
#if VIDEO_OPEN
        if (Video_Out.isOpened())
        {
            Video_Out.release();
            ROS_INFO("Video has been released...");
        }
#endif
    }

    //......
    void webcam1();
    void start() { video_thread = boost::thread(&ucar_vision::webcam1, this); }

private:
    boost::thread video_thread; // 或有必要开多线程?
    //......
};

ucar_vision::ucar_vision(ros::NodeHandle *n) : Picodet_vision()
{
#if VIDEO_OPEN
    std::time_t now = std::time(0);
    tm *ltm = localtime(&now);
    String T_path = this->filename + to_string(int(ltm->tm_mon)) + "_" + to_string(int(ltm->tm_mday)) + "_" + to_string(int(ltm->tm_hour)) + "_";
    T_path += (to_string(int(ltm->tm_min)) + ".mp4");
    Video_Out.open(T_path, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 12, cv::Size(640, 480));
    ROS_INFO("writing Video...");
#endif // 开启视频录制
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, config["cap_1"]["CAP_PROP_AUTO_EXPOSURE"].as<float>());
    cap.set(cv::CAP_PROP_EXPOSURE, config["cap_1"]["CAP_PROP_EXPOSURE"].as<float>());
    cap.set(cv::CAP_PROP_BRIGHTNESS, config["cap_1"]["CAP_PROP_BRIGHTNESS"].as<float>());
    // cap.set(cv::CAP_PROP_CONTRAST, config["cap_1"]["CAP_PROP_CONTRAST"].as<float>());
    // cap.set(cv::CAP_PROP_SATURATION, config["cap_1"]["CAP_PROP_SATURATION"].as<float>());
    // cap.set(cv::CAP_PROP_HUE, config["cap_1"]["CAP_PROP_HUE"].as<float>());
    pudding_p = config["thresholds"]["pudding_p"].as<float>();
    NULL_num_th = config["thresholds"]["NULL_num"].as<int>();
    n_ = *n;
    // flag_dp_sub = n_.subscribe("/flag2", 100, &ucar_vision::flag_dp_CB, this);
    cv_label_pub = n_.advertise<std_msgs::Int8>("/pic_data", 100);
    cv_position_pub = n_.advertise<std_msgs::Int32>("/pic_position", 100);

    ROS_INFO("vision is ready");

    timer_image = n_.createTimer(ros::Duration(1.0 / 40.0), &ucar_vision::get_image, this);
    timer_contorller = n_.createTimer(ros::Duration(1.0 / 12.0), &ucar_vision::webcam, this); // 开启定时器
};

void ucar_vision::flag_dp_CB(const std_msgs::Int32 &msg)
{
    this->detect_flag = msg.data;
    // std::cout << detect_flag << "  ";
};

//////////////////////////////////////////////////////////////////////////////////
void ucar_vision::get_image(const ros::TimerEvent &)
{
    cap >> image;
    cv::flip(this->image, this->image, 1);
}
void ucar_vision::webcam(const ros::TimerEvent &)
{
    cv::Mat Image = image.clone();
    ///
    double t = (double)cv::getTickCount();
    ///

    // if (14 == detect_flag)
    // {

        this->to_detect();
        if (!this->Results.empty()) // 有识别到任何东西
        {
            this->trans_Box(this->Results, 640, 480);
            // CL = this->get_label_with_filter();
            std::cout << this->class_names[this->Results[0].label] << "  " <<this->Results[0].score << "  " << CL.location << std::endl;
            cv_mode_now.data = CL.label;
            cv_label_pub.publish(cv_mode_now);
            cv_pose_now.data = CL.location;
            // if (CL.label > 3) // 识别到目标道具(与此同时，恐怖分子不发位置)
            // {
            //     // cout << CL.location << endl;
            //     turn_d = (CL.location - last_position) > 0 ? 1 : -1; // 转向趋势
            //     last_position = CL.location;
            //     once_label_num++; // 如果是目标类别，识别次数++
            //     NULL_num = 0;
            //     cv_position_pub.publish(cv_pose_now);
            // }
            // else if (CL.label == 0)
            // {
            //     if (once_label_num > 0)
            //         NULL_num++;
            //     if (NULL_num > NULL_num_th)
            //     {
            //         cv_pose_now.data = last_position + turn_d * pudding_p; // 发上次有值的位置
            //         ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            //     }
            //     cv_position_pub.publish(cv_pose_now);
            // }

            this->image = this->draw_bboxes(this->image, this->Results, 0);
        // }
        // else // 整幅图像无任何识别
        // {
        //     // std::cout << "NULL"<< "  "<< "NULL"<< "  "<< "-1"<< std::endl;
        //     cv_mode_now.data = 0;
        //     cv_pose_now.data = -1;
        //     if (once_label_num > 0)
        //         NULL_num++;
        //     if (NULL_num > NULL_num_th)
        //     {
        //         cv_pose_now.data = last_position + turn_d * pudding_p; // 发上次有值的位置
        //         ROS_INFO("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        //     }
        //     cv_position_pub.publish(cv_pose_now);
        }
        else 
            ROS_INFO("empty");
        // detect_flag = 11111;
    // }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (66 == detect_flag) /// 关闭识别任务
    {
#if VIDEO_OPEN               ///
        Video_Out.release(); ///
        ROS_INFO("Video is Save...");
#endif
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, config["cap_2"]["CAP_PROP_AUTO_EXPOSURE"].as<float>());
        cap.set(cv::CAP_PROP_EXPOSURE, config["cap_2"]["CAP_PROP_EXPOSURE"].as<float>());
        cap.set(cv::CAP_PROP_BRIGHTNESS, config["cap_2"]["CAP_PROP_BRIGHTNESS"].as<float>());
        // cap.set(cv::CAP_PROP_CONTRAST, config["cap_2"]["CAP_PROP_CONTRAST"].as<float>());
        // cap.set(cv::CAP_PROP_SATURATION, config["cap_2"]["CAP_PROP_SATURATION"].as<float>());
        // cap.set(cv::CAP_PROP_HUE, config["cap_2"]["CAP_PROP_HUE"].as<float>());
        cap.release();
        timer_contorller.stop();
        timer_image.stop();
    } ///
#if VIDEO_OPEN
    Video_Out.write(this->image);
    // jjjj++;
#endif
    //////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // // 帧率打印
    // t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << iiii << "  FPS: " << 1.0 / t << endl;
    // fps[iiii] = 1.0 / t;
    // if (iiii == 4)
    // {
    //     cout << "fps: " << 1.0 / t << "   val  FPS: " << (fps[0] + fps[1] + fps[2] + fps[3] + fps[4]) / 5 << endl;
    // }
    // iiii = (++iiii) % 5;
};

/// //////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pic_get");
    ros::NodeHandle n("~");
    // ros::Rate loop_rate(15);
    ROS_INFO("Vision preparing");
    ucar_vision Vision(&n);
    // while(ros::ok())
    // {
    //     Vision.webcam();
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::spin();
    return 0;
}

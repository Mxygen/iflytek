/*
 * @Author: AyefLev 592162794@qq.com
 * @Date: 2024-03-19 21:08:37
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-07-15 22:14:19
 * @FilePath: \24xf_Vision\include\vision_ncnn.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
 #ifndef VISION_NCNN
 #define VISION_NCNN
 
 #include <opencv2/opencv.hpp>
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <iostream>
 #include <net.h>
 #include "picodet.h"
 #include <benchmark.h>
 #include <string.h>
 #include <stack>
 #include <yaml-cpp/yaml.h>
 // #include <ros/ros.h>
 using namespace std;
 using namespace cv;
 // #define NEWC 1
 enum equipments
 {
     pie,    //0
     red,    //1
     nana, //2
     coke, //3
     pep,      //4
     green, //5
     tom, //6
     milk,
     pot,
     apple,
     melon
 };
 
 typedef struct Clocassion
 {
     int label;
     float location;
     float cx1, cx2, cy1, cy2;
     Clocassion(int a, float b) : label(a), location(b) { cx1 = cx2 = cy1 = cy2 = 0; };
     Clocassion()
     {
         label = -1;
         location = -1;
         cx1 = cx2 = cy1 = cy2 = -1;
     };
 } Clocassion;
 
 const int color_list[23][3] =
     {
         {216, 82, 24},
         {236, 176, 31},
         {125, 46, 141},
         {118, 171, 47},
         {76, 189, 237},
         {238, 19, 46},
         {76, 76, 76},
         {153, 153, 153},
         {255, 0, 0},
         {255, 127, 0},
         {190, 190, 0},
         {0, 255, 0},
         {0, 0, 255},
         {170, 0, 255},
         {84, 84, 0},
         {84, 170, 0},
         {84, 255, 0},
         {170, 84, 0},
         {170, 170, 0},
         {170, 255, 0},
         {255, 84, 0},
         {255, 170, 0},
         {255, 255, 0},
 };
 
 class Picodet_vision
 {
 public:
     cv::VideoCapture cap;
     cv::Mat image;
     stack<cv::Mat> IMAGES;
     std::vector<BoxInfo> Results;
 
     YAML::Node config = YAML::LoadFile("/home/ucar/ucar_test/src/navigation_test/cfg/Avision_1.yml");
 
     int class_Num = 11;
     int model_size_w = config["model"]["size_w"].as<int>();
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
     string PARAM = config["model"]["param_path"].as<std::string>();
     string BIN = config["model"]["bin_path"].as<std::string>();
     const char *param = PARAM.c_str();
     const char *bin = BIN.c_str();
 
     PicoDet detector = PicoDet(param, bin, model_size_w, model_size_w, class_Num, true);
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
     std::vector<std::string> CNAME = config["model"]["class_names"].as<std::vector<std::string>>();
     std::vector<const char *> class_names;
 
     // const char *class_names[8] = {
     //     "teargas_1",
     //     "teargas_2",
     //     "body_armor_1",
     //     "body_armor_2",
     //     "baton_1",
     //     "terrorists_2",
     //     "terrorists_1",
     //     "terrorists_3",
     // };
 
     const char *tol_class_names[12] = {"null_class", "pie", "red", "nana", "coke", "pep", "green", "tom", "milk", "pot", "apple", "melon"}; // 大类类名//_去急救包
     std::vector<int> class_order = config["model"]["class_order"].as<std::vector<int>>();
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     std::vector<int> BoxLim = config["Filter"]["BoxLim"].as<std::vector<int>>();
     //const int BoxLim[2] = {540, 400};
     const double Fc = 0;
     const double class_img_w[11] = {};
     const double class_img_h[11] = {};
 
     float score_threshold = config["thresholds"]["score"].as<float>();
     float nms_threshold = config["thresholds"]["nms"].as<float>();
 
     int F_Terrorists = 0;
     int F_Get_first_aid_packet = 1;
     int F_Get_stage = 0;
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     // 常用、主要函数:
     void get_image();
     void to_detect();
     std::vector<Clocassion> get_label();
     Clocassion get_label_with_filter();
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     // 测试函数:
     int image_demo(PicoDet &detector, const char *imagepath);
     int webcam_demo();
     int video_demo(PicoDet &detector, const char *path, int save_frame, int draw);
 
     Picodet_vision();
     Picodet_vision(cv::VideoCapture &CAP);
     //~Picodet_vision();
 
     ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
     // 工具
     std::vector<int> GenerateColorMap(int num_class);                                     // 生成色图 :)
     cv::Mat draw_bboxes(const cv::Mat &Im, const std::vector<BoxInfo> &bboxes, int show); // 画框而已
     void resize_uniform(cv::Mat &src, cv::Mat &dst, cv::Size dst_size);
     std::vector<BoxInfo> trans_Box(std::vector<BoxInfo> &Box, int w, int h);
 
 private:
 };
 extern cv::Mat global_image;
 #endif
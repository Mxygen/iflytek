#ifndef FIND_LINE_H
#define FIND_LINE_H

#include <opencv2/core/core.hpp>
#include <array>
#include <cstring>

// float midline_detect(double angle_p_left, double angle_p_right, double angle_p_mid, double b_p_left, double b_p_right, double b_p_mid, cv::Mat& img);
typedef struct dot_flag{
    float dot;
    int flag;
}dot_flag;

// LineDetectionMaps类用于存储巡线处理中的地图和边界点
class LineDetectionMaps {
public:

    //增加每一行的畸变参数
    double data_line[320];      //增加每一行的畸变参数
    int least_mid_dot = 35;    //调整什么时候轮到中线来巡线

    int base_arrow_mid = 30;
    int base_arrow_left_right = 0;

    ////////////////////////////////////////////

    int flag = 0;  //flag改变代表着出发

    // 成员变量
    cv::Mat allmap; // 最初始的二值化图，1表示白点，0表示黑点
    cv::Mat basemap; // 从allmap中搜索得到的图，白点连通，边界值为2表示实际的黑点
    cv::Mat leftmap; // 从basemap的左边黑点开始搜索得到的黑色连通区域图
    cv::Mat rightmap; // 从basemap的右边黑点开始搜索得到的黑色连通区域图
    cv::Mat insidemap;  //最后画出三个线的图

//    int leftline[190][15];
//    int rightline[190][15];

    int BRIGHT = 255;
    int GRAY = 90;


    // 存储kb的值    注意：第三个是横坐标的修改后的b
    float midline_kb[3];  //存储中线的k和b
    float leftline_kb[3];  //存储左线的k和b
    float rightline_kb[3];  //存储右线的k和b

    //后面加上两个搜索，搜索左右图的线段
    int leftmap_line[300][15];
    int rightmap_line[300][15];
    int midline[300][15];  //搜索中间的线段，用二分之一的办法

    float midfloat=0;           //这个是沿着斜率走的
    float mid_dot=0;            //这个是沿着点偏差走的

    // 记录左线,右线,中线的点的个数
    int leftmap_line_num = 0;
    int rightmap_line_num = 0;
    int midline_line_num = 0;
    // 记录左图，右图点的个数
    int basenum_all=0;
    int lnum_all=0;
    int rnum_all=0;
    // 一个def的定义
    int MIDLINE = 0;
    int LEFTLINE = 1;
    int RIGHTLINE =2;
    int NOLINE =3;

    // // 盲选左线右线
    // int LEFT = -15;
    // int RIGHT = 15;

    int ERROR = 999;

    // main里面的变量
    // 设置图像的宽度和高度
    int width=640/2;
    int height=300;

    int LIMIT_LINE = 300;
    // Width: 640 Height: 300 Channels: 1
    int YM = 300 -1; // y轴最大遍历点
    int XM = width-1; // x轴最大遍历点
    int XX = width/2-1; // x轴中心点

    int top = 300; // 示例：定义 _top 变量并赋值为 0
    int _top = 0;  // 真正用到的中点个数

    int detect_y_top=35;     //一般取最高点 原来是15的
    // int y_top_left = 35;
    // int y_top_right = 35;


    int HORIZON_TIMES=2;


    int mid_dot_n = 1000;
    int left_dot_n = 1000;
    int right_dot_n = 1000;

    int y_top_mid = 135;
    int y_top_left = 105;
    int y_top_right = 105;


    // 构造函数
    LineDetectionMaps();
    void init_time();

    void horizonCompress(const cv::Mat &src);
    void unevenLightCompensate(cv::Mat image, int blockSize ,cv::Mat &outputImage);
    void processImage(const cv::Mat& inputImage, cv::Mat& outputImage);
//    两个实际上是一个函数，只是参数不同
//    searchimg 是 主要的函数
    void searchimg(int x, int y); // searchimg函数声明
    void searchWhiteArea( int x, int y);

    void searchleftmap(int x, int y); // search 左边的黑点
    void searchrightmap(int x, int y); // search 右边的黑点

    void find_leftmapPoints();   // 寻找leftmap边界点
    void find_rightmapPoints();  // 寻找rightmap边界点

    void find_midline();

    void draw_line();  //画出三条线
    void draw_midline();  //画出中间的线

    void show();



    float getMidlinedot_seg();
    float getLeftlinedot_seg();
    float getRightlinedot_seg();


    // main 函数
    dot_flag midline_detect(double piancha_p_left, double piancha_p_right, double piancha_p_mid, double angle_p_left, double angle_p_right, double angle_p_mid, double b_p_left, double b_p_right, double b_p_mid, cv::Mat& img);
    int servo_test();     //选取标准线，进行测试


};



#endif
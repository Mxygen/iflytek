#ifndef LINE_DETECTION_MAPS_H_INCLUDED
#define LINE_DETECTION_MAPS_H_INCLUDED

#include <opencv2/core/core.hpp>
#include <array>
#include <cstring>


// 成员变量
cv::Mat allmap; // 最初始的二值化图，1表示白点，0表示黑点
cv::Mat basemap; // 从allmap中搜索得到的图，白点连通，边界值为2表示实际的黑点


cv::Mat leftmap; // 从basemap的左边黑点开始搜索得到的黑色连通区域图
cv::Mat rightmap; // 从basemap的右边黑点开始搜索得到的黑色连通区域图
cv::Mat insidemap;  //最后画出三个线的图

// 定义一个点的结构体，用于存储巡线相关的坐标点
struct Point {
    int x; // 横坐标
    int y; // 纵坐标
    Point() : x(0), y(0) {} // 默认构造函数
    Point(int x, int y) : x(x), y(y) {} // 带参数的构造函数
};

int leftline[190][15]={0};
int rightline[190][15]={0};
int errorline[190]={0};

int midline[190][15]={0};  //中间的黑点

//后面加上两个搜索，搜索左右图的线段
int leftmap_line[190][15]={0};
int rightmap_line[190][15]={0};

float midfloat=0;

// LineDetectionMaps类用于存储巡线处理中的地图和边界点
class LineDetectionMaps {
public:
    // 构造函数
    LineDetectionMaps();



    // 存储不同区域边界点的数组
    std::array<Point, 5> left_bottom; // 右图，区域最左边边界的底部
    std::array<Point, 5> right_bottom; // 左图，区域最右边边界的底部
    std::array<Point, 5> left_top; // 右图，区域最左边边界的顶部
    std::array<Point, 5> right_top; // 左图，区域最右边边界的顶部

    std::array<Point, 5> bottom_left; // 右图，区域底部边界的最左边
    std::array<Point, 5> bottom_right; // 左图，区域底部边界的最右边
    std::array<Point, 5> top_left; // 右图，区域顶部边界的最左边
    std::array<Point, 5> top_right; // 左图，区域顶部边界的最右边

    // 成员函数声明（具体实现在cpp文件中）
    // void initializeMaps(int width, int height); // 初始化地图的函数
   
    void processImage(const cv::Mat& inputImage, cv::Mat& outputImage);
//    两个实际上是一个函数，只是参数不同
//    searchimg 是 主要的函数
    void searchimg(int x, int y); // searchimg函数声明
    void searchWhiteArea( int x, int y);
    
    void searchleftmap(int x, int y); // search 左边的黑点
    void searchrightmap(int x, int y); // search 右边的黑点
    
    // 寻找basemap边界点
    void find_leftBoundaryPoints();
    void find_rightBoundaryPoints();


    void find_leftmapPoints();   // 寻找leftmap边界点
    void find_rightmapPoints();  // 寻找rightmap边界点
    


    void find_midline();

    void draw_line();  //画出三条线
    void draw_midline();  //画出中间的线

    // void findBoundarythe_Points(); // 查找边界点的函数
    // void finddots(const cv::Mat& inputImage, cv::Mat& outputImage); // 查找黑点的函数
    // void detectEdges(const cv::Mat& inputImage, cv::Mat& outputImage); // 检测边缘的函数
    // void detectLines(const cv::Mat& inputImage, std::vector<cv::Vec4i>& lines); // 检测直线的函数



};

float midline_detect();

#endif
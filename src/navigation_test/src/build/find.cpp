// basemap 是0开始的   2024/4/15  //16:11

// searchWhiteArea( x, y + 1); 这个要不要注释掉

//    for(int y=height-1; y>detect_y_top; y--)  
//计算k的时候也只允许这些点参与计算中，不能超过detect_y_top才行


// 一些突发状况的处理
// hsv光照的不稳定
// 寻找点的时候会出现basemap是对的，但是图像不对


// 时间于17:56 
// 之后开始大改

// 找点的一个问题
// 没有看到一些边缘的点，他们能够出现截断的情况()

// 并且修改了这个东西
// if (x < XX + 15 && leftmap.at<uchar>(y, x + 1) == 0) // 向右

// 左右图的点做出了初始化
// 左图默认为0，右图需要XM作为最大值

// 增加摄像头水平翻转

// 正在查询为什么中线会直接缩短一大截
        // if(leftmap_pointnum > 8)
        // {
        //     if(y<height/2)
        //     {cout<<y<<endl;
        //     for(int i=0;i<leftmap_pointnum;i++)
        //     cout<<leftmap_line[y][i]<<" ";
        //     cout<<endl;
        //     detect_y_top=y;
        //     }
        // }
// 暂时还没有得出结果

#include "iostream"
#include<opencv2/opencv.hpp>
#include "cstring"
#include "algorithm"
#include "find.h"


using namespace std;
// using namespace cv;

// 设置图像的宽度和高度
#define width 640
#define height 190

int LIMIT_LINE = 190;
// Width: 640 Height: 190 Channels: 1

int YM = 189; // y轴最大遍历点
int XM = 639; // x轴最大遍历点
int XX = 319; // x轴中心点

int top = 190; // 示例：定义 _top 变量并赋值为 0
int _top = 0;
int repeat_counter = 0; // 示例：定义 repeat_counter 变量并赋值为 0
int black_point_count = 0; // 示例：定义 black_point_count 变量并赋值为 0



// 白点的总数
int totalWhitePoints=0;


int detect_y_top=6;     //一般取最高点        
//在寻找左右点时候发现线段出现横线的情况，这个时候的总体mid方向只允许到top为止


// 注释
// allmap：最初始的二值化图，二值化为1就是1，二值化为0值就是0
// basemap：allmap中搜来的，白点一定连通，所以从一个白点开始搜到边界。
// 	边界值为2，在实际上是黑点
// leftmap：basemap中搜来的，从左边的黑点开始搜，搜黑色连通区域。
// 	边界在basemap边界外1格，值为2。在basemap中值为0，在allmap中值为0，实际上是白点。

// Point left_bottom[5];			//右图，区域最左边边界的底部
// Point right_bottom[5];			//左图，区域最右边边界的底部
// Point left_top[5];				//右图，区域最左边边界的顶部
// Point right_top[5];				//左图，区域最右边边界的顶部

// Point bottom_left[5];			//右图，区域底部边界的最左边
// Point bottom_right[5];			//左图，区域底部边界的最右边
// Point top_left[5];				//右图，区域顶部边界的最左边
// Point top_right[5];				//左图，区域顶部边界的最右边
// 这些点在对应的图中值为1，为边界内部黑点，就是basemap最外边的黑点
 

// void initialize_Maps()
// {
//       // 初始化
//     allmap = cv::Mat::zeros(height, width, CV_8UC1);
//     basemap = cv::Mat::zeros(height, width, CV_8UC1);
//     leftmap = cv::Mat::zeros(height, width, CV_8UC1);
//     rightmap = cv::Mat::zeros(height, width, CV_8UC1);
//     insidemap = cv::Mat::ones(height, width, CV_8UC1)*255; // 与allmap相同大小的全白图
// }


LineDetectionMaps::LineDetectionMaps() 
{   

    for (int i = 0; i < 5; ++i) {
        left_bottom[i] = Point(-1, -1);
        right_bottom[i] = Point(-1, -1);
        left_top[i] = Point(-1, -1);
        right_top[i] = Point(-1, -1);
        bottom_left[i] = Point(-1, -1);
        bottom_right[i] = Point(-1, -1);
        top_left[i] = Point(-1, -1);
        top_right[i] = Point(-1, -1);
    }
}

void LineDetectionMaps::processImage(const cv::Mat& inputImage, cv::Mat& outputImage) {

    cv::Mat hsv;
    cv::Mat crop = inputImage(cv::Range(290, 480), cv::Range(0, 640)); // Slicing to crop the image
    // cv::imshow("originmap", crop);
    // bool isSuccess = cv::imwrite("saved_frame.jpg", crop);


    // cv::Mat crop = cv::img(cv::Range(290, 480), cv::Range(0, 640)); // Slicing to crop the image
    cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar white_low = cv::Scalar(0, 0, 181); //hsv
    cv::Scalar white_high = cv::Scalar(99, 56, 255);
    cv::inRange(hsv, white_low, white_high, hsv); //hsv operation
    // cv::imshow("originmap2", hsv);

    cv::Mat struct1, struct2;  //dilate and erode
//          struct1 = getStructuringElement(0, cv::Size(3, 3));  //矩形结构元素
    struct2 = getStructuringElement(1, cv::Size(3, 3));  //十字结构元素
    cv::medianBlur(hsv, hsv, 3);  //中值滤波
    // cv::erode(hsv, hsv, struct2);//腐蚀
    cv::dilate(hsv, hsv, struct2); //膨胀

    // // 填充
    // the_Point p1(0, 0);
    // the_Point p2(150, 0);
    // the_Point p3(0, 100);
    // std::vector<the_Point> the_Points;
    // the_Points.push_back(p1);
    // the_Points.push_back(p2);
    // the_Points.push_back(p3);
    // fillConvexPoly(hsv, the_Points, Scalar(0, 0, 0));

    allmap = hsv.clone();
    // cv::bitwise_not(allmap, allmap);

    // cv::imshow("allmap", allmap);

    // cout << "Width: " << allmap.cols << " Height: " << allmap.rows << " Channels: " << allmap.channels() << endl;
}


void LineDetectionMaps::searchimg(int x, int y) {
    // int totalWhitethe_Points = 0; // 用来记录找到的白点数量
    searchWhiteArea(x, y);
    
    // cv::imshow("basemap", basemap);
}

// 递归搜索白色区域
void LineDetectionMaps::searchWhiteArea( int x, int y) {
    // 检查坐标(x, y)是否在图像范围内
    
    if (x < 0 || x >= allmap.cols || y < 0 || y >= allmap.rows) {
        // cout << "Width: " << allmap.cols << " Height: " << allmap.rows << " Channels: " << allmap.channels() << endl;
        // cout << "x" << x << "y" << y <<endl;
        return;
    }

    // 如果当前点是白点并且未被访问过
    if (allmap.at<uchar>(y, x) == 0 && basemap.at<uchar>(y, x) == 0) {
        // 标记为已访问
        basemap.at<uchar>(y, x) = 128;
        

        // 增加总白点数
        totalWhitePoints++;

        // 递归搜索相邻点
        searchWhiteArea( x + 1, y); // 右
        searchWhiteArea( x - 1, y); // 左
        // searchWhiteArea( x, y + 1); // 下
        searchWhiteArea( x, y - 1); // 上
    } else if (allmap.at<uchar>(y, x) == 255) {
        // 如果是黑点，标记边界
        basemap.at<uchar>(y, x) = 255;
    }
}



void LineDetectionMaps::searchleftmap(int x, int y) 
{
    if (y >= YM || x >= XM) return; // 检查图像边界

    if (basemap.at<uchar>(y, x) == 0) { // 如果basemap中该点非0（非赛道内部）
        // cout<<"x"<<x<<"y"<<y<<endl;
        // 对关键点处理
        leftmap.at<uchar>(y, x) = 128;    // leftmap中该点设为1
        insidemap.at<uchar>(y, x) = 0;  // insidemap中该点设为0

        if (y > _top) _top = y; // 把最大非白点y坐标赋给_top

        // if (rightmap.at<uchar>(y, x) != 0) repeat_counter++; // 若rightmap中值也非0，则重复计数器自加
        // black_point_count++; // 该区域黑点数量自加

        // 搜图条件
        // if (x < XX - 1 && leftmap.at<uchar>(y, x + 1) == 0) // 向右
        if (x < XX + 60 && leftmap.at<uchar>(y, x + 1) == 0) // 向右
            searchleftmap(x + 1, y);

        if (x > 0 && leftmap.at<uchar>(y, x - 1) == 0) // 向左
            searchleftmap(x - 1, y);

        if (y > 0 && leftmap.at<uchar>(y - 1, x) == 0) // 向上
            searchleftmap(x, y - 1);
        
        
        // 向下的条件
        if (y < YM - 1 && y < LIMIT_LINE && leftmap.at<uchar>(y + 1, x) == 0) 
        {
            if (y < top && y> 50) { // 若y比top小就向下搜
                searchleftmap(x, y + 1);
            // } else if (basemap.at<uchar>(y + 1, x) != 0 && y + 1 == top + 1) { // 若这点之下的点为赛道外部点而且搜到top之上1格
            //     // 取消下寻，记录该点，之后有用
            //     // 从赛道最高点之上搜下来就不搜了
            // } else if (basemap.at<uchar>(y + 1, x) == 0) { // 若这点之下的点为赛道内部点
            //     leftmap.at<uchar>(y + 1, x) = 255; // 则把这点之下的点设为2
            }
        }
        
    } else { // 如果basemap中该点为0（赛道边界）
        leftmap.at<uchar>(y, x) = 255; // leftmap中该点设为2，退出递归
    }
    // cv::imshow("leftmap", leftmap);
}



void LineDetectionMaps::searchrightmap(int x, int y) {
    if (y >= YM || x >= XM) return; // Check image boundary

    if (basemap.at<uchar>(y, x) == 0) { // If the point in basemap is not 0 (not inside the track)
        // Handle key points
        // cout<<"x"<<x<<"y"<<y<<endl;

        rightmap.at<uchar>(y, x) = 128; // Set the point in rightmap to 1
        insidemap.at<uchar>(y, x) = 0; // Set the point in insidemap to 0

        if (y > _top) _top = y; // Assign the maximum non-white point y-coordinate to _top

        // Search conditions

        if (x > 0 && rightmap.at<uchar>(y, x - 1) == 0) // Left
            searchrightmap(x - 1, y);
        // if (x > XX - 1 && rightmap.at<uchar>(y, x + 1) == 0) // Right
        if (x > XX - 60 && rightmap.at<uchar>(y, x + 1) == 0) // Right
            searchrightmap(x + 1, y);
        if (y > 0 && rightmap.at<uchar>(y - 1, x) == 0) // Up
            searchrightmap(x, y - 1);

        // Downward condition
        if (y < YM - 1 && y < LIMIT_LINE && rightmap.at<uchar>(y + 1, x) == 0) {
            if (y < top && y> 50) { // If y is less than top, search downwards
            //特地用50标注表示---如果在上面是不允许下落的
                searchrightmap(x, y + 1);
            // } else if (basemap.at<uchar>(y + 1, x) != 0 && y + 1 == top + 1) {
            //     // Stop searching downwards if the point below is outside the track and one grid above top
            // } else if (basemap.at<uchar>(y + 1, x) == 0) { // If the point below is inside the track
            //     rightmap.at<uchar>(y + 1, x) = 255; // Set the point below to 2
            }
        }
        
    } else { // If the point in basemap is 0 (track boundary)
        rightmap.at<uchar>(y, x) = 255; // Set the point in rightmap to 2 and exit recursion
    }
    // cv::imshow("rightmap", rightmap);
}

/*
记录的方式：
一行只有一个黑点，记录这个黑点的x坐标
如果出现一行有两个黑点，记录这两个黑点的x坐标，errorline++
之后专门写一个函数让那一行只有一个黑点
*/

void LineDetectionMaps::find_leftBoundaryPoints() {
    for(int y=height-1; y>0; y--) 
    {
        int leftpointnum = 0;   //记录每一行的黑点数量
        for(int x=0; x<width/2; x++) 
        {
            
            if(leftmap.at<uchar>(y, x) == 255) {
                leftline[y][leftpointnum] = x;
                leftpointnum++;            
            }
        }
        if(leftpointnum >= 2) 
        {
            errorline[y]++;     //应该只有一行黑点
            continue;
        }
        // 计数点记错了！
        else
        if(leftpointnum==0 && y>height/2)
        {
            leftline[y][leftpointnum] = 0;
        }
    }
}

void LineDetectionMaps::find_rightBoundaryPoints() {
    for(int y=height-1; y>0; y--)
    {
        int rightpointnum = 0;    //记录每一行的黑点数量
        for(int x=width/2; x<width; x++) 
        {
            
            if(rightmap.at<uchar>(y, x) == 255) {
                rightline[y][rightpointnum] = x;
                rightpointnum++;
                // rightBoundaryPoints.push_back(cv::Point(x, y));
            }
        }

        if(rightpointnum >= 2) 
        {
            errorline[y]++;     //应该只有一行黑点
            continue;
        }
        else 
        if(rightpointnum==0 && y>height/2)
        {
            rightline[y][rightpointnum] = width;
        }
    }
}

void LineDetectionMaps::find_leftmapPoints() {
    for(int y = 0; y < height; y++) 
    {
        int leftmap_pointnum=0;
        for(int x = 0; x < width/2; x++) 
        {
            
            if(leftmap.at<uchar>(y, x) == 255) {
                leftmap_line[y][leftmap_pointnum] = x;
                leftmap_pointnum++;            
            }
        }

        // 左图如果没有我们就为这个数字
        if(leftmap_pointnum == 0)
        {
            leftmap_line[y][leftmap_pointnum] = 0;
        }

        if(leftmap_pointnum >=2 && leftmap_pointnum <8)    //考虑到如果出现一条横线的情况
        {
        //找到最右边的点，并且清除其他的点
            int max = 0;
            for(int i=0; i<leftmap_pointnum; i++) {
                if(leftmap_line[y][i] > max) {
                    max = leftmap_line[y][i];
                }
            }
            for(int i=0; i<leftmap_pointnum; i++) {
                if(leftmap_line[y][i] != max) {
                    leftmap_line[y][i] = 0;
                }
            }
            leftmap_line[y][0] = max;
        }
        
        if(leftmap_pointnum > 8)
        {
            if(y<height/2)
            {
            // cout<<y<<endl;
            for(int i=0;i<leftmap_pointnum;i++)
            // cout<<leftmap_line[y][i]<<" ";
            // cout<<endl;
            detect_y_top=y;
            }
        }
    
        // break;
    }
}


void LineDetectionMaps::find_rightmapPoints() {
    for(int y = 0; y < height; y++) 
    {
        int rightmap_pointnum=0;

        for(int x = width/2; x < width; x++) 
        {
            
            if(rightmap.at<uchar>(y, x) == 255) {
                rightmap_line[y][rightmap_pointnum] = x;
                rightmap_pointnum++;            
            }
        }

        // 右图如果没有我们就为这个数字
        if(rightmap_pointnum == 0)
        {
            rightmap_line[y][rightmap_pointnum] = XM-1;
        }


        if(rightmap_pointnum >= 2 && rightmap_pointnum <8) 
        {
        //找到最左边的点，并且清除其他的点
            int min = 640;
            for(int i=0; i<rightmap_pointnum; i++) {
                if(rightmap_line[y][i] < min) {
                    min = rightmap_line[y][i];
                }
            }
            for(int i=0; i<rightmap_pointnum; i++) {
                if(rightmap_line[y][i] != min) {
                    rightmap_line[y][i] = 0;
                }
            }
            rightmap_line[y][0] = min;
        }
        if(rightmap_pointnum > 8)
        {
            if(y<height/2)
            detect_y_top=y;
        }
        
    
        // break;
    }
}



void LineDetectionMaps::find_midline() 
{
    for(int y=height-1; y>detect_y_top; y--) 
    {
        int mid=(leftmap_line[y][0] + rightmap_line[y][0]) / 2;
        if(errorline[y] == 0) {
            midline[y][0] = (leftmap_line[y][0] + rightmap_line[y][0]) / 2;
        }
        if(errorline[y] > 0 ) {
            midline[y][0] = (leftmap_line[y][0] + rightmap_line[y][0]) / 2;
        }

        midfloat = midfloat + (mid-320);

    }
}





/*
画图的方式
有errorline的判断机制，但是midline实际上并没有完全处理好
需要写一个处理errorline的函数
让一些特殊情况归一
*/
void LineDetectionMaps::draw_line() {
    for(int y=0; y<height; y++) {
        if(errorline[y] == 0) {
            insidemap.at<uchar>(y, leftmap_line[y][0]) = 255;
        }
        if(errorline[y] == 1) {
            insidemap.at<uchar>(y, leftmap_line[y][0]) = 128;
        }
    }

    for(int y=0; y<height; y++) {
        if(errorline[y] == 0) {
            insidemap.at<uchar>(y, rightmap_line[y][0]) = 255;
        }
        if(errorline[y] == 1) {
            insidemap.at<uchar>(y, rightmap_line[y][0]) = 128;
        }
    }

    // 应该不会发生这种情况吧
    for(int y=0; y<height; y++) {
        if(errorline[y] == 2) {
            insidemap.at<uchar>(y, leftmap_line[y][0]) = 128;
            insidemap.at<uchar>(y, rightmap_line[y][0]) = 128;
        }
    }
}


void LineDetectionMaps::draw_midline() {
    for(int y=height-1; y>detect_y_top; y--) {
        insidemap.at<uchar>(y, midline[y][0]) = 255;
    }
}



int main() {
    while(true)
    {

    
    midfloat=0;      //偏差点
    // // 读取图片
    // cv::Mat img = cv::imread("way.jpg");
    // if (img.empty()) {
    //     std::cout << "Error loading the image" << std::endl;
    //     return -1;
    // }
    
    // 读取摄像头
    cv::VideoCapture cap(0); // 0代表第一个摄像头
    // 检查摄像头是否成功打开
    if (!cap.isOpened()) {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    
        // 帧数
        double fps = 0.0;
        cv::TickMeter tm;
        tm.start();

        cv::Mat frame;
        // 读取新的帧
        if (!cap.read(frame)) 
        {
            std::cout << "No frame" << std::endl;
        }


        cv::flip(frame, frame, 1); // 第三个参数为1表示水平翻转，0表示垂直翻转，-1表示水平和垂直同时翻转
        // 显示帧


        // 初始化
        allmap = cv::Mat::zeros(height, width, CV_8UC1);
        basemap = cv::Mat::zeros(height, width, CV_8UC1);
        leftmap = cv::Mat::zeros(height, width, CV_8UC1);
        rightmap = cv::Mat::zeros(height, width, CV_8UC1);
        insidemap = cv::Mat::zeros(height, width, CV_8UC1); // 与allmap相同大小的全白图


        // 创建LineDetectionMaps类的对象
        LineDetectionMaps lineDetectionMaps;

        // 使用processImage函数处理图像
        cv::Mat processedImage;
        lineDetectionMaps.processImage(frame, processedImage); // 将图像复制到processedImage中

        processedImage.clone(); // 将处理后的图像传递到allmap中

        // 假设您想在特定位置开始搜索图像
        int startX = 319; // 起始搜索位置X
        int startY = 189; // 起始搜索位置Y
        lineDetectionMaps.searchimg(startX, startY); // 搜索图像


        for (int j = 0; j < 190; ++j)
        {
            if (basemap.at<uchar>(j, 0) == 0  && rightmap.at<uchar>(j, 0) == 0)
            {
                lineDetectionMaps.searchleftmap(0,j);
            }
            if (basemap.at<uchar>(j, XM) == 0 && leftmap.at<uchar>(j, XM) == 0 )
            {
                lineDetectionMaps.searchrightmap(XM-1,j);
            }
        }

        // lineDetectionMaps.searchleftmap(0,130);

        // lineDetectionMaps.searchrightmap(638,130);




        
        cv::imshow("leftmap", leftmap);

        
        cv::imshow("rightmap", rightmap);


        // lineDetectionMaps.find_leftBoundaryPoints();
        // lineDetectionMaps.find_rightBoundaryPoints();

        lineDetectionMaps.find_rightmapPoints();
        lineDetectionMaps.find_leftmapPoints();


        lineDetectionMaps.find_midline();

        // lineDetectionMaps.draw_line();
        lineDetectionMaps.draw_midline();

        std::vector<cv::Vec2f> lines;
        cv::HoughLines(insidemap, lines, 1, CV_PI / 180, 100);

        for (const auto& line : lines) 
        {
        float rho = line[0];
        float theta = line[1];

        double a = std::cos(theta);
        double b = std::sin(theta);
        double x0 = a * rho;
        double y0 = b * rho;

        cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
        cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));

        cv::line(insidemap, pt1, pt2, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }

        // 显示原图和二值化后的图像
        // cv::imshow("Original Image", img);
        // cv::imshow("Binary Image", processedImage); // 应该使用processedImage而不是binaryImg
        // cv::imshow("insidemap", insidemap);     //画出真正的巡线轨迹


        // 等待任意键按下
        // cv::imshow("basemap", basemap);


        // // 这里主要是纠错用得上
        // // 输出errorline的情况
        // for(int y=0; y<height; y++) {
        //     if(errorline[y] > 0) {
        //         cout<<"errorline"<<y<<":"<<errorline[y]<<endl;
        //         for(int i=0; i<15; i++) {
        //             if(leftline[y][i] == 0) {
        //                 break;
        //             }
        //             cout<<leftline[y][i]<<" ";
        //         }
        //         cout<<endl;
        //         for(int i=0; i<15; i++) {
        //             if(rightline[y][i] == 0) {
        //                 break;
        //             }
        //             cout<<rightline[y][i]<<" ";
        //         }
        //         cout<<endl;
        //         cout<<"--------------------"<<endl;

        //     }
        // }



        // // 输出midline的情况

        midfloat = midfloat/(190-detect_y_top);
        // cout<<"mid: "<<midfloat<<endl;

        int kp = 0.1;
        // pid控制
        midfloat = midfloat*kp;
        // cout<<"mid: "<<midfloat<<endl;
        if (midfloat>0)
        {
            if(midfloat>0.5)
            {
                midfloat=0.5;
            }
        }
        if (midfloat<0)
        {
            if(midfloat<-0.5)
            {
                midfloat=-0.5;
            }
        }

        cout<<"mid :     " << midfloat <<endl;





        // 清空
        memset(leftline, 0, sizeof(leftline));
        memset(rightline, 0, sizeof(rightline));
        memset(errorline, 0, sizeof(errorline));
        memset(midline, 0, sizeof(midline));
        memset(leftmap_line, 0, sizeof(leftmap_line));
        memset(rightmap_line, 0, sizeof(rightmap_line));


        // 这部分也是最开始变量的初始化
        LIMIT_LINE = 190;
        YM = 189; // y轴最大遍历点
        XM = 639; // x轴最大遍历点
        XX = 319; // x轴中心点

        top = 190; // 示例：定义 _top 变量并赋值为 0
        _top = 0;
        repeat_counter = 0; // 示例：定义 repeat_counter 变量并赋值为 0
        black_point_count = 0; // 示例：定义 black_point_count 变量并赋值为 0

        // 白点的总数
        totalWhitePoints=0;
        detect_y_top=6;     //一般取最高点       





        // 按下ESC键退出
        if (cv::waitKey(10) == 27) {
        }


        tm.stop();

        // fps =  tm.getTimeMilli();
        // fps = 1000.0 / fps;
        // cout<<fps<<endl;
        tm.reset();

    // 释放摄像头资源
    cap.release();

    // 关闭所有OpenCV窗口
    cv::destroyAllWindows();

    return midfloat;
    }
}

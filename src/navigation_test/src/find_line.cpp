/*/
--4-24=--
midfloat  偏差变化太大
现在解决办法是设置一个before，如果这个before偏差超过一定值，那就说明不合理，这个时候车子必须在最开始就在线上面
另一个办法是去除干扰的点:修改find_midline


同时对左右图也用这样的操作
通过lasty >=36进行可调整范围，防止最开始出现的去掉线操作
中线lasty >=50

关于 ytop不起作用（明明是找点时候不可能找得到的）


--4-25--

1. 在原始图像上面增加了光照，更确切说以后必须要从最开始的源头入手
先硬件再原图再图像处理

2. 对right和left的kb进行修正，但是这个修正并不能作为真正的修正
因为这个修正是在最开始的时候就要做的事情
在boom代码里面的standard里面的代码

我的办法是记录kb然后用b修正k的角度，调参

/*/



#include "iostream"
#include <opencv2/opencv.hpp>
#include "cstring"
#include "algorithm"
#include "find_line.h"

using namespace std;

float midfloat_before=0;
static float previous_error = 0;


LineDetectionMaps::LineDetectionMaps()
{

    // 偏差的权重进行赋值
    // 注意：底部从零开始
    std::fill(std::begin(data_line), std::end(data_line), 1.0);
    ////////////////////// 这个是之前的图像巡线加上的办法
    ///                    由于它非要是逆向的
   for (int i = height; i < height-120; i--){ data_line[i] = 1.0; }       //0-120
   for (int i = height-120; i < height-140; i--){ data_line[i] = 0.95; }   //120-140
   for (int i = height-140; i < height-160; i--){ data_line[i] = 0.9; }
   for (int i = height-160; i < height-180; i--){ data_line[i] = 0.85; }
   for (int i = height-180; i < height-200; i--){ data_line[i] = 0.8; }
   for (int i = height-200; i < height-220; i--){ data_line[i] = 0.75; }
   for (int i = height-220; i < height-240; i--){ data_line[i] = 0.7; }



    midfloat = 0; // 偏差点
    mid_dot = 0;


    // 清空
//    memset(leftline, 0, sizeof(leftline));
//    memset(rightline, 0, sizeof(rightline));
    memset(midline, 0, sizeof(midline));
    memset(leftmap_line, 0, sizeof(leftmap_line));
    memset(rightmap_line, 0, sizeof(rightmap_line));

    allmap = cv::Mat::zeros(height, width, CV_8UC1);
    basemap = cv::Mat::zeros(height, width, CV_8UC1);
    leftmap = cv::Mat::zeros(height, width, CV_8UC1);
    rightmap = cv::Mat::zeros(height, width, CV_8UC1);
    insidemap = cv::Mat::zeros(height, width, CV_8UC1); // 与allmap相同大小的全白图

    // 记录左线,右线,中线的点的个数
    leftmap_line_num = 0;
    rightmap_line_num = 0;
    midline_line_num = 0;
    // 记录左图，右图点的个数
    lnum_all=0;
    rnum_all=0;
    basenum_all=0;
    // 这部分也是最开始变量的初始化


}

void LineDetectionMaps::init_time()
{
    allmap = cv::Mat::zeros(height, width, CV_8UC1);
    basemap = cv::Mat::zeros(height, width, CV_8UC1);
    leftmap = cv::Mat::zeros(height, width, CV_8UC1);
    rightmap = cv::Mat::zeros(height, width, CV_8UC1);
    insidemap = cv::Mat::zeros(height, width, CV_8UC1); // 与allmap相同大小的全白图

    midfloat = 0; // 偏差点
    mid_dot = 0;


    // 清空
//    memset(leftline, 0, sizeof(leftline));
//    memset(rightline, 0, sizeof(rightline));
    memset(midline, 0, sizeof(midline));
    memset(leftmap_line, 0, sizeof(leftmap_line));
    memset(rightmap_line, 0, sizeof(rightmap_line));

    allmap = cv::Mat::zeros(height, width, CV_8UC1);
    basemap = cv::Mat::zeros(height, width, CV_8UC1);
    leftmap = cv::Mat::zeros(height, width, CV_8UC1);
    rightmap = cv::Mat::zeros(height, width, CV_8UC1);
    insidemap = cv::Mat::zeros(height, width, CV_8UC1); // 与allmap相同大小的全白图

    // 记录左线,右线,中线的点的个数
    leftmap_line_num = 0;
    rightmap_line_num = 0;
    midline_line_num = 0;
    // 记录左图，右图点的个数
    lnum_all=0;
    rnum_all=0;
    basenum_all=0;


}


void LineDetectionMaps::horizonCompress(const cv::Mat &src) {
    int dealMax = height * width / HORIZON_TIMES;

    cv::Mat dst = cv::Mat::zeros(height, 640 / HORIZON_TIMES, CV_8UC1);


    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < 640 / HORIZON_TIMES; ++j) {
            int blackNumCnt = 0;
            for (int index = 0; index < HORIZON_TIMES; ++index) {
                if (src.at<uint8_t>(i, j * HORIZON_TIMES + index) < 255) {
                    blackNumCnt++;
                }
            }
            if (blackNumCnt >= HORIZON_TIMES) {
                dst.at<uint8_t>(i, j) = 0;
            } else {
                dst.at<uint8_t>(i, j) = BRIGHT;
            }
        }
    }


//    cv::imshow("dst",dst);
    allmap = dst.clone();
}



void LineDetectionMaps::unevenLightCompensate(cv::Mat image, int blockSize ,cv::Mat &outputImage)
{
    if (image.channels() == 3) cvtColor(image, image, 7);
    double average = mean(image)[0];
    int rows_new = ceil(double(image.rows) / double(blockSize));
    int cols_new = ceil(double(image.cols) / double(blockSize));
    cv::Mat blockImage;
    blockImage = cv::Mat::zeros(rows_new, cols_new, CV_32FC1);
    for (int i = 0; i < rows_new; i++)
    {
        for (int j = 0; j < cols_new; j++)
        {
            int rowmin = i*blockSize;
            int rowmax = (i + 1)*blockSize;
            if (rowmax > image.rows) rowmax = image.rows;
            int colmin = j*blockSize;
            int colmax = (j + 1)*blockSize;
            if (colmax > image.cols) colmax = image.cols;
            cv::Mat imageROI = image(cv::Range(rowmin, rowmax), cv::Range(colmin, colmax));
            double temaver = mean(imageROI)[0];
            blockImage.at<float>(i, j) = temaver;
        }
    }
    blockImage = blockImage - average;
    cv::Mat blockImage2;
    resize(blockImage, blockImage2, image.size(), (0, 0), (0, 0), cv::INTER_CUBIC);
    cv::Mat image2;
    image.convertTo(image2, CV_32FC1);
    cv::Mat dst = image2 - blockImage2;
    dst.convertTo(image, CV_8UC1);

    outputImage = image.clone();
}



void LineDetectionMaps::processImage(const cv::Mat &inputImage, cv::Mat &outputImage)
{
//////    // 添加去除光照
    cv::Mat lightImage;
    unevenLightCompensate(inputImage, 48 ,lightImage);
    cv::Mat crop = lightImage(cv::Range(480 - height, 480), cv::Range(0, 640)); // Slicing to crop the image


//        //取消去除光照
//        cv::Mat crop = inputImage(cv::Range(330, 480), cv::Range(0, 640)); // Slicing to crop the image
//        cv::cvtColor(crop, crop, cv::COLOR_BGR2GRAY);

    // 直接使用灰度图，不需要转换到HSV
    cv::Mat gray = crop; // 假设输入已经是灰度图

    // 设置灰度阈值

//    // 125
//    int gray_low = 144; // 灰度下限
//    int gray_high = 255; // 灰度上限
//    cv::inRange(gray, cv::Scalar(gray_low), cv::Scalar(gray_high), gray); // 灰度阈值操作


    // // 如果adaptive就会出现这种情况
    // cv::Mat struct1, struct2; // dilate and erode
    // // struct1 = getStructuringElement(0, cv::Size(3, 3));  // 矩形结构元素
    // struct2 = getStructuringElement(1, cv::Size(3, 3)); // 十字结构元素
    // cv::medianBlur(gray, gray, 5);                        // 中值滤波
    // cv::erode(gray, gray, struct2); // 腐蚀
    // // cv::dilate(gray, gray, struct2); // 膨胀


////    //方案二：自适应阈值
//    cv::adaptiveThreshold(crop,crop,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,3,6);
//    cv::bitwise_not(crop, crop);
//    gray = crop; // 假设输入已经是灰度图


//      //方案三 大津法
    double otsu_thresh_val = cv::threshold(
            gray, // 输入图像
            gray, // 输出（结果）图像
            0,    // 阈值（0表示自动计算）
            255,  // 最大值
            cv::THRESH_BINARY | cv::THRESH_OTSU // 阈值类型
    );
    cv::inRange(gray, cv::Scalar(0), cv::Scalar(otsu_thresh_val), gray); // 灰度阈值操作
    cv::bitwise_not(gray, gray);

    // 如果adaptive就会出现这种情况
    cv::Mat struct1, struct2; // dilate and erode
    // struct1 = getStructuringElement(0, cv::Size(3, 3));  // 矩形结构元素
    struct2 = getStructuringElement(1, cv::Size(3, 3)); // 十字结构元素
    cv::medianBlur(gray, gray, 5);                        // 中值滤波
    cv::erode(gray, gray, struct2); // 腐蚀
    // cv::dilate(gray, gray, struct2); // 膨胀


    allmap = gray.clone();

    horizonCompress(allmap);


}


void LineDetectionMaps::searchimg(int x, int y)
{
    // int totalWhitethe_Points = 0; // 用来记录找到的白点数量
    searchWhiteArea(x, y);
}

// 递归搜索白色区域
void LineDetectionMaps::searchWhiteArea(int x, int y)
{
    // 检查坐标(x, y)是否在图像范围内

    if (x < 0 || x >= allmap.cols || y < 0 || y >= allmap.rows)
    {
        // cout << "Width: " << allmap.cols << " Height: " << allmap.rows << " Channels: " << allmap.channels() << endl;
        // cout << "x" << x << "y" << y <<endl;
        return;
    }

    // 如果当前点是白点并且未被访问过
    if (allmap.at<uchar>(y, x) == 0 && basemap.at<uchar>(y, x) == 0)
    {
        // 标记为已访问
        basemap.at<uchar>(y, x) = GRAY;
        basenum_all++;

        // 递归搜索相邻点
        searchWhiteArea(x + 1, y); // 右
        searchWhiteArea(x - 1, y); // 左
        searchWhiteArea(x, y - 1); // 上

        if (y < top && y> height )
        {
            searchWhiteArea( x, y + 1); // 下
        }
    }
    else if (allmap.at<uchar>(y, x) == 255)
    {
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
        leftmap.at<uchar>(y, x) = GRAY;    // leftmap中该点设为1
        insidemap.at<uchar>(y, x) = 0;  // insidemap中该点设为0
        lnum_all++;            //统计左图的点的个数

        if (y > _top)   _top = y; // 把最大非白点y坐标赋给_top

        // 搜图条件
        // if (x < XX - 1 && leftmap.at<uchar>(y, x + 1) == 0) // 向右
        if (x < XM-10 && leftmap.at<uchar>(y, x + 1) == 0 && basemap.at<uchar>(y, x+1) != GRAY &&
            rightmap.at<uchar>(y, x+1) != GRAY) // 向右
            searchleftmap(x + 1, y);

        if (x > 0 && leftmap.at<uchar>(y, x - 1) == 0 && basemap.at<uchar>(y, x-1) != GRAY &&
            rightmap.at<uchar>(y, x+1) != GRAY) // 向左
            searchleftmap(x - 1, y);

        if (y > 0 && leftmap.at<uchar>(y - 1, x) == 0 && basemap.at<uchar>(y-1, x) != GRAY &&
            rightmap.at<uchar>(y, x+1) != GRAY) // 向上
            searchleftmap(x, y - 1);


        // 向下的条件
        if (y < YM - 1 && y < LIMIT_LINE && leftmap.at<uchar>(y + 1, x) == 0 && basemap.at<uchar>(y+1, x) != GRAY &&
            rightmap.at<uchar>(y, x+1) != GRAY)
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
        lnum_all++;    //统计左图的点的个数
    }
    // cv::imshow("leftmap", leftmap);
}



void LineDetectionMaps::searchrightmap(int x, int y) {
    if (y >= YM || x >= XM) return; // Check image boundary

    if (basemap.at<uchar>(y, x) == 0) { // If the point in basemap is not 0 (not inside the track)
        // Handle key points
        // cout<<"x"<<x<<"y"<<y<<endl;

        rightmap.at<uchar>(y, x) = GRAY; // Set the point in rightmap to 1
        insidemap.at<uchar>(y, x) = 0; // Set the point in insidemap to 0
        rnum_all++;  // Count the number of points in the right map

        if (y > _top) _top = y; // Assign the maximum non-white point y-coordinate to _top

        // Search conditions

        if (x > 0 && rightmap.at<uchar>(y, x - 1) == 0 && basemap.at<uchar>(y, x-1) != GRAY ) // Left
            searchrightmap(x - 1, y);
        // if (x > XX - 1 && rightmap.at<uchar>(y, x + 1) == 0) // Right
        if (x >  10 && rightmap.at<uchar>(y, x + 1) == 0 && basemap.at<uchar>(y, x+1) != GRAY) // Right
            searchrightmap(x + 1, y);
        if (y > 0 && rightmap.at<uchar>(y - 1, x) == 0 && basemap.at<uchar>(y-1, x) != GRAY) // Up
            searchrightmap(x, y - 1);

        // Downward condition
        if (y < YM - 1 && y < LIMIT_LINE && rightmap.at<uchar>(y + 1, x) == 0 && basemap.at<uchar>(y+1, x) != GRAY) {
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
        rnum_all++;  //统计右图的点的个数
    }
    // cv::imshow("rightmap", rightmap);
}


void LineDetectionMaps::find_leftmapPoints()
{
    int lasty = height - 1;
    for (int y = height - 1; y > detect_y_top; y--)
    {
        int leftmap_pointnum = 0;

        for (int x = width; x >0; x--)
        {
            if (leftmap.at<uchar>(y, x) == 255)
            {
                leftmap_line[y][leftmap_pointnum] = x;
                leftmap_pointnum++;
            }
        }

        if (leftmap_pointnum == 0)
        {
            leftmap_line[y][0] = ERROR;
        }
        else if (leftmap_pointnum >= 2 && leftmap_pointnum < 8)
        {
            int max = 0;
            for (int i = 0; i < leftmap_pointnum; i++)
            {
                if (leftmap_line[y][i] > max)
                {
                    max = leftmap_line[y][i];
                }
            }
            leftmap_line[y][0] = max;
            // 清除其他点
            for (int i = 1; i < leftmap_pointnum; i++)
            {
                leftmap_line[y][i] = 0;
            }
        }

        if (leftmap_pointnum > 0)
        {
            int left_tmp = leftmap_line[y][0];
            if (y <= 20) // 对 y 值小于 30 的行进行特殊处理
            {
                if (abs(leftmap_line[lasty][0] - left_tmp) >= 20)
                {
                    leftmap_line[y][0] = ERROR;
                    continue; // 跳过当前行
                }
            }
            if (lasty >= 20)
            {
                leftmap_line[y][0] = left_tmp;
            }
            else
            {
                if (abs(leftmap_line[lasty][0] - left_tmp) < 20)
                {
                    leftmap_line[y][0] = left_tmp;
                }
                else
                {
                    leftmap_line[y][0] = ERROR;
                    continue;
                }
            }
            lasty = y; // 更新 lasty
            if (leftmap_pointnum > 0 && leftmap_line[y][0] != ERROR)
            {
                leftmap_line_num++; // 增加有效行数计数
            }
        }
    }
}


void LineDetectionMaps::find_rightmapPoints()
{
    int lasty = height - 1;
    for (int y = height - 1; y > detect_y_top; y--)
    {
        int rightmap_pointnum = 0;

        for (int x = 0; x < width; x++)
        {
            if (rightmap.at<uchar>(y, x) == 255)
            {
                rightmap_line[y][rightmap_pointnum] = x;
                rightmap_pointnum++;
            }
        }

        if (rightmap_pointnum == 0)
        {
            rightmap_line[y][0] = ERROR;
        }
        else if (rightmap_pointnum >= 2 && rightmap_pointnum < 8)
        {
            int min = width;
            for (int i = 0; i < rightmap_pointnum; i++)
            {
                if (rightmap_line[y][i] < min)
                {
                    min = rightmap_line[y][i];
                }
            }
            rightmap_line[y][0] = min;
            // 清除其他点
            for (int i = 1; i < rightmap_pointnum; i++)
            {
                rightmap_line[y][i] = 0;
            }
        }

        if (rightmap_pointnum > 0)
        {
            int right_tmp = rightmap_line[y][0];
            if (y < 20) // 对 y 值小于 30 的行进行特殊处理
            {
                if (abs(rightmap_line[lasty][0] - right_tmp) >= 20)
                {
                    rightmap_line[y][0] = ERROR;
                    continue; // 跳过当前行
                }
            }
            if (lasty >= 20)
            {
                rightmap_line[y][0] = right_tmp;
            }
            else
            {
                if (abs(rightmap_line[lasty][0] - right_tmp) < 20)
                {
                    rightmap_line[y][0] = right_tmp;
                }
                else
                {
                    rightmap_line[y][0] = ERROR;
                    continue;
                }
            }
        }

        lasty = y;
        if (rightmap_pointnum > 0 && rightmap_line[y][0] != ERROR)
        {
            rightmap_line_num++;
        }
    }
}

void LineDetectionMaps::find_midline()
{
    int lasty = height - 1;
    for(int y = height - 1; y>detect_y_top; y--)
    {
        if (leftmap_line[y][0]!=0 && leftmap_line[y][0]!=width && rightmap_line[y][0] <= width-10
            && rightmap_line[y][0]!=ERROR && leftmap_line[y][0]!=ERROR)
        {
            //判断和之前的相比能不能好用
            if(y < 160 && midline_line_num>8)
            {
                int mid_tmp = (leftmap_line[y][0] + rightmap_line[y][0]) / 2;
                if(abs( midline[lasty][0]- mid_tmp ) < 35)
                {
                    midline[y][0] = mid_tmp;
                }
                else
                {
                    midline[y][0] = ERROR;
                    continue;
                }
            }


            if(lasty>=160 || midline_line_num<8)
            {
                midline[y][0] = (leftmap_line[y][0] + rightmap_line[y][0]) / 2;
            }
            else
            {
                int mid_tmp = (leftmap_line[y][0] + rightmap_line[y][0]) / 2;
                if(abs( midline[lasty][0]- mid_tmp ) < 35)
                {
                    midline[y][0] = mid_tmp;
                }
                else
                {
                    midline[y][0] = ERROR;
//                    cout<<"continue"<<y<<endl;
                    continue;
                }
            }

            lasty = y;
            midline_line_num++;
            // cout<<midline[y][0]<<endl;
            // cout<<"mid:"<<midline[y][0]<<" "<<"left:"<<leftmap_line[y][0]<<" "<<"right:"<<rightmap_line[y][0]<<endl;

        }
        else
        {
            midline[y][0] = ERROR;   //如果这个东西没有，就设置ERROR标志位
        }
    }

}


// 画图
void LineDetectionMaps::show()
{
    cv::imshow("allmap", allmap);
    cv::imshow("basemap", basemap);
    cv::imshow("leftmap", leftmap);
    cv::imshow("rightmap", rightmap);
    cv::imshow("insidemap", insidemap);
    // cout<<"detect_y_top: "<<detect_y_top<<endl;
}



/*
画图的方式
有errorline的判断机制，但是midline实际上并没有完全处理好
需要写一个处理errorline的函数
让一些特殊情况归一
*/
void LineDetectionMaps::draw_line() {
    for(int y=height-base_arrow_left_right; y>detect_y_top; y--) {
        if(leftmap_line[y][0] != ERROR) {
//            cout<<leftmap_line[y][0]<<endl;
            insidemap.at<uchar>(y, leftmap_line[y][0]) = 255;
        }
    }

    for(int y=height-base_arrow_left_right; y>detect_y_top; y--) {
        if(rightmap_line[y][0] != ERROR){
            insidemap.at<uchar>(y, rightmap_line[y][0]) = 255;
        }
    }
}


void LineDetectionMaps::draw_midline() {
    for(int y=height-base_arrow_mid; y>detect_y_top; y--) {
        if(midline[y][0]!=ERROR)
        {
            insidemap.at<uchar>(y, midline[y][0]) = 255;
        }
    }
}



float LineDetectionMaps::getMidlinedot_seg() {
    double n = 0;
    int piancha =0;


    // 补线
    if ( midline_line_num < 200 && midline_line_num > least_mid_dot ) {
        // 遍历 0-190 的点
        for (int i = height - base_arrow_mid; i >= detect_y_top ; i--)
        {

            if (midline[i][0] == 0 || midline[i][0] == ERROR) {
                if (leftmap_line[i][0] != ERROR && leftmap_line[i][0] != 0) {
                    midline[i][0] = ( leftmap_line[i][0] + width ) /2;
                    insidemap.at<uchar>(i, midline[i][0]) = 255;
                } else if (rightmap_line[i][0] != ERROR && rightmap_line[i][0] != 0) {
                    midline[i][0] = rightmap_line[i][0] / 2;
                    insidemap.at<uchar>(i, midline[i][0]) = 255;
                }else
                {
                    midline[i][0] = width/2;
                    insidemap.at<uchar>(i, midline[i][0]) = 255;
                }
            }
        }

    }


    for (int i = height - base_arrow_mid;  i > y_top_mid && n < mid_dot_n; i--) {
        if (midline[i][0] != 0 && midline[i][0] != ERROR)
        {
            n = n + data_line[i];
            piancha += (midline[i][0] * 2 - width) * data_line[i];
        }
    }

//    cout<<"mid_dot_n"<<n<<endl;
    if (n > 0) {
        piancha = piancha*1.0/n;
    }

    return piancha;
}

float LineDetectionMaps::getLeftlinedot_seg() {
    double n = 0;
    int piancha =0;

    for (int i = height - base_arrow_left_right; i > y_top_left && n < left_dot_n; i--) {
        if (leftmap_line[i][0] != 0 && leftmap_line[i][0] != ERROR) {
            n = n + data_line[i];
            piancha += (0 - leftmap_line[i][0] )  * data_line[i];    //左线的偏差是负的
        }
    }
//    cout<<"left_dot_n"<<n<<endl;

    if (n > 0) {
        piancha = piancha/n;
    }

    return piancha;
}

float LineDetectionMaps::getRightlinedot_seg() {
    double n = 0;
    int piancha =0;

    for (int i = height - base_arrow_left_right; i > y_top_right && n < right_dot_n; i--) {
        if (rightmap_line[i][0] != 0 && rightmap_line[i][0] != ERROR) {
            n = n + data_line[i];
            piancha += (width - rightmap_line[i][0]) * data_line[i]; //右线的偏差是正的
        }
    }
//    cout<<"right_dot_n"<<n<<endl;
    if (n > 0) {
        piancha = piancha/n;
    }

    return piancha;
}




// 1. 如果左右线都有，那么就用中线
// 2. 如果只有一边有线，那么就用这一边的线
// 3. 如果两边都没有线，那么就用上一次的线
int LineDetectionMaps::servo_test()
{
    int controlLine=0;

    // 记录左线，右线点的个数
    int leftnum = leftmap_line_num;
    int rightnum = rightmap_line_num;
    // 记录左图右图的点的个数
    int lnum = lnum_all;
    int rnum = rnum_all;


//    if (midline_line_num >60)
    if (midline_line_num > least_mid_dot )
    {
        return  MIDLINE;
    }


    if (leftnum < 50)
    {
        lnum = 0;
    }
    if (rightnum < 50)
    {
        rnum = 0;
    }
    if (leftnum < 50 && rightnum < 50)
    {
        lnum = rnum = 0;
    }

    //设置一个特殊情况：insidemap什么也没有
    if (leftnum <10 && rightnum<10)
    {
        controlLine = NOLINE;
        return  NOLINE;
    }



//--------------------------------------------控制线设定------------------------------------------------

    //注意：这个地方的判断没有修改
    //之前的图像是压缩过的，所以这些点的数值是不一样的


    if (lnum > rnum + 10000)
    {
        controlLine = LEFTLINE;
    }
    else if (rnum > lnum + 10000)
    {
        controlLine = RIGHTLINE;
    }
    else
    {
        if (lnum >= rnum && rnum <= 10000 && lnum >= 10000)
        {
            controlLine = LEFTLINE;
        }
        else if (rnum >= lnum && lnum <= 10000 &&
                 rnum >= 10000)
        {
            controlLine = RIGHTLINE;
        }
        else if (rnum >= 10000 && lnum >= 10000)
        {
            controlLine = MIDLINE;
        }
        else
        {
//            controlLine = MIDLINE;
            // if(midline_line_num > 10)
            //     return MIDLINE;
            if(leftmap_line_num > rightmap_line_num)
            {
                controlLine = LEFTLINE;
                return LEFTLINE;
            }
            if (rightmap_line_num > leftmap_line_num)
            {
                controlLine = RIGHTLINE;
                return RIGHTLINE;
            }
        }
    }
    // 如果什么都没有找到，记得用之前的值


    return controlLine;
}





dot_flag LineDetectionMaps::midline_detect(double piancha_p_left, double piancha_p_right, double piancha_p_mid, double angle_p_left, double angle_p_right, double angle_p_mid, double b_p_left, double b_p_right, double b_p_mid, cv::Mat& img)
{
    init_time();

    cv::Mat frame;


    cv::flip(img, frame, 1); // 第三个参数为1表示水平翻转，0表示垂直翻转，-1表示水平和垂直同时翻转


    // 使用processImage函数处理图像
    cv::Mat processedImage;
    processImage(frame, processedImage); // 将图像复制到processedImage中

    processedImage.clone();


    // 假设您想在特定位置开始搜索图像
    int startX = width/2 -1;          // 起始搜索位置X
    int startY = height-1;          // 起始搜索位置Y
    searchimg(startX, startY); // 搜索图像

    //解决办法1：如果出现被覆盖的情况，我们就换一个地方进行生长
    if(basenum_all<5)
    {
        searchimg(startX-10,startY);
    }
    if(basenum_all<5)
    {
        searchimg(startX+10,startY);
    }



    for (int j = height-70; j > 70; j--)
    {
        if (basemap.at<uchar>(j, XM) == 0 && leftmap.at<uchar>(j, XM) == 0)
        {
            searchrightmap(XM - 1, j);
        }
    }

    for (int j = height-70; j > 70; j--)
    {
        if (basemap.at<uchar>(j, 0) == 0 && rightmap.at<uchar>(j, 0) == 0)
        {
            searchleftmap(0, j);
        }
    }


    find_rightmapPoints();
    find_leftmapPoints();
    find_midline();

    draw_line();
    draw_midline();
    // cv::imshow("allmap", allmap);

    // cout<<"left_line_point"<<leftmap_line_num<<endl;
    // cout<<"right_line_point"<<rightmap_line_num<<endl;
    // cout<<"mid_line_point"<<midline_line_num<<endl;

    // cout<<"leftmap"<<lnum_all<<endl;
    // cout<<"rightmap"<<rnum_all<<endl;



    // mid_dot = getMidlinedot_seg();
    int judge = servo_test();
    if(judge == LEFTLINE)
    {
        cout<<"LEFTLINE"<<"-------"<<endl;
        mid_dot = getLeftlinedot_seg();
        mid_dot = mid_dot*piancha_p_left;
    }
    else if(judge == RIGHTLINE)
    {
        cout<<"RIGHTLINE"<<"-------"<<endl;
        mid_dot = getRightlinedot_seg();
        mid_dot = mid_dot*piancha_p_right;
    }
    else if(judge == MIDLINE)
    {
        cout<<"MIDLINE"<<"-------"<<endl;
        mid_dot = getMidlinedot_seg();
        mid_dot = mid_dot*piancha_p_mid;
    }
    else if(judge == NOLINE)
    {
        cout<<"NOLINE"<<"-------"<<"Hofman open"<<endl;
        mid_dot = getMidlinedot_seg();
        mid_dot = mid_dot*piancha_p_mid;
    }

    dot_flag AAAA;
    AAAA.dot = mid_dot;
    AAAA.flag = judge;
    cout<<"mid_dot"<<mid_dot<<endl;
    // return mid_dot;
    return AAAA;
}


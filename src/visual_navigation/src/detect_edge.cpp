#include "../include/detect_edge.h"

using namespace cv;
using namespace std;
#define road_width 120
uint8_t current_border_L[74] = {0};
uint8_t current_border_R[74] = {0};
uint8_t last_border_L[74] = {0};
uint8_t last_border_R[74] = {0};
uint8_t llast_border_L[74] = {0};
uint8_t llast_border_R[74] = {0};
uint8_t G_paused = 0;

bool Canny_Method(Mat &original_frame, double lowthreshold, double highthreshold, int Trace_edge, bool Debug)
{
    /*图像深度预处理*/
    resize(original_frame, original_frame, Size(160, 120), 0, 0, INTER_LINEAR); // 压缩分辨率
    flip(original_frame, original_frame, 1);                                    // 水平反转
    cvtColor(original_frame, original_frame, COLOR_BGR2GRAY);                   // 原始图像转换灰度图像
    // GaussianBlur(original_frame, original_frame, Size(5, 5), 1.5);              // 高斯滤波
    Canny(original_frame, original_frame, lowthreshold, highthreshold);         // Canny算子边缘检测
    bool RightAngle = false;
    int Dir = 0;
    // cv::imshow("original_frame",original_frame);
    // cv::waitKey(1);
    /*中心提取*/

    if (!original_frame.empty())
    {
        /*将图片转换为二维数组*/
        std::vector<std::vector<uint8_t>> image(original_frame.rows, std::vector<uint8_t>(original_frame.cols));
        for (int i = 0; i < original_frame.rows; ++i)
        {
            for (int j = 0; j < original_frame.cols; ++j)
            {
                image[i][j] = original_frame.at<uint8_t>(i, j);
            }
        }
        if (Trace_edge == 9)
        {

            if (Canny_Crawl_Top(image, original_frame.cols * 0.2, original_frame.cols * 0.8))
            {
                ROS_INFO("Quit");
                return true;
            }
            else
            {
                Trace_edge = 3;
            }
        }
        /*爬取边线*/
        Canny_Crawl_L_R(image, original_frame.cols, original_frame.rows, original_frame.rows * 0.6);
        if(RightDetect(image, original_frame.cols * 0.2, original_frame.cols * 0.8, Dir))
        {
            ROS_INFO("Right Detect");
            RightAngle = true;
        }
        /*计算中线*/
        for (int i = original_frame.rows * 0.6; i < original_frame.rows - 1; i++)
        {

            if (Trace_edge == 1)
            {
                G_line_M[i] = G_border_R[i] - road_width / 2;
            }
            else if (Trace_edge == 2)
            {
                G_line_M[i] = G_border_L[i] + 2 + road_width / 2;
            }
            else
            {
                G_line_M[i] = (G_border_L[i] + G_border_R[i]) / 2; // 简单取均值
                if (RightAngle)
                {
                    if (Dir == 1)
                        G_line_M[i] -= road_width / 4;
                    else if (Dir == 2)
                        G_line_M[i] += road_width / 4;   
                }
            }

            if (Debug)
            {
                original_frame.at<uint8_t>(i, G_line_M[i]) = 255;
            }

        }
    }
    return false;
}

void Canny_Crawl_L_R(const std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height, uint8_t highest)
{
    uint8_t i = 0, j = 0, found_L = 0, found_R = 0;
    static uint8_t count = 0;
    if (count == 0)
    {
        count = 1;
        for (i = height - 1; i > highest; i--)
            G_line_M[i] = 80;
        
    }
    for (i = height - 1; i > highest; i--)
    {
        current_border_L[i] = 0;
        current_border_R[i] = width - 1;

    }


    for (i = height - 1; i > highest; i--)
    {
        for (j = G_line_M[i]; j > 0; j--)
        {
            if (image[i][j])
            {
                current_border_L[i] = j;
                break;
            }
        }
        G_border_L[i] = current_border_L[i];
    }

    for (i = height - 1; i > highest; i--)
    {

        for (j = G_line_M[i]; j < width - 1; j++)
        {
            if (image[i][j])
            {
                current_border_R[i] = j;
                break;
            }
        }
        G_border_R[i] = current_border_R[i];
    }

}
bool Canny_Crawl_Top(const std::vector<std::vector<uint8_t>> &image, uint8_t left, uint8_t right)
{
    uint8_t count = 0;
    uint8_t tmp[right - left] = {0};
    uint8_t height = 35;
    uint8_t thres = 18;
    for (int i = left; i < right; i++)
    {
        for (int j = 119; j > 119 - height; j--)
        {
            if (image[j][i])
            {
                count++;
                tmp[i - left] = j;
                break;
            }
            
        }
    }
    int average = 0, average_count = 0;
    for (int i = 1; i < right - left - 2; i++)
    {
        if (abs(tmp[i] - tmp[i - 1]) < 5)
        {
            average += tmp[i];
            average_count++;
        }
    }
    average /= average_count;
    // ROS_INFO("average_count: %d", average_count);

    if (average_count > 0.8 * (right - left) && average > 119 - thres)
    {
        ROS_INFO("average Count: %d  average : %d ", average_count, average);
        return true;
    }
    else
    {
        return false;
    }
}

bool RightDetect(const std::vector<std::vector<uint8_t>> &image, uint8_t left, uint8_t right,int &Dir)
{

    uint8_t count = 0;
    int direction = 0;
    for(int i = 72; i < 110; i++)
    {
        if(G_border_R[i] == 159 ^ G_border_L[i] == 0)
        {
            count ++;
            if (G_border_L[i] == 0)
                direction ++;
            else 
                direction --;
        }
    }
    if (count < 15)
        return false;
    else if (direction >= 0)
        Dir = 1;
    else if (direction < 0)
        Dir = 2;



    count = 0;
    uint8_t tmp[right - left] = {0};
    uint8_t height = 72;
    uint8_t thres = 18;
    for (int i = left; i < right; i++)
    {
        for (int j = 119; j > 72; j--)
        {
            if (image[j][i])
            {
                count++;
                tmp[i - left] = j;
                break;
            }
            
        }
    }
    int average = 0, average_count = 0;
    //方差
    double variance = 0.0;
    for (int i = 1; i < right - left - 2; i++)
    {
        if (abs(tmp[i] - tmp[i - 1]) < 2)
        {
            average += tmp[i];
            average_count++;
        }
    }
    average /= average_count;
    for (int i = 1; i < right - left - 2; i++)
    {
        variance += (tmp[i] - average) * (tmp[i] - average);
    }
    variance /= average_count;
    ROS_WARN("variance: %f", variance);

    // ROS_INFO("average_count: %d", average_count);

    if (average_count > 0.6 * (right - left) and average < 84 and variance < 10)
    {
        return true;
    }
    else
        return false;
}
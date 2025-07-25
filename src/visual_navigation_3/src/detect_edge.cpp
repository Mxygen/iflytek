#include "../include/detect_edge.h"
#include "../include/mask_L.h"
#include "../include/mask_R.h"
using namespace cv;
using namespace std;
#define road_width 120
uint8_t current_border_L[120] = {0};
uint8_t current_border_R[120] = {0};
int globalOdom = 0;


bool Canny_Method(Mat &original_frame, double lowthreshold, double highthreshold, int Trace_edge, bool Debug)
{
    resize(original_frame, original_frame, Size(160, 120), 0, 0, INTER_LINEAR); // 压缩分辨率
    flip(original_frame, original_frame, 1);                                    // 水平反转
    cvtColor(original_frame, original_frame, COLOR_BGR2GRAY);                   // 原始图像转换灰度图像
    // GaussianBlur(original_frame, original_frame, Size(5, 5), 1.5);              // 高斯滤波
    Canny(original_frame, original_frame, lowthreshold, highthreshold);         // Canny算子边缘检测
    int Dir = 0;
    static int maskInsert = 0;

    if (!original_frame.empty())
    {
        std::vector<std::vector<uint8_t>> image(original_frame.rows, std::vector<uint8_t>(original_frame.cols));
        for (int i = 0; i < original_frame.rows; ++i)
            for (int j = 0; j < original_frame.cols; ++j)
                image[i][j] = original_frame.at<uint8_t>(i, j);



        if (Trace_edge == 9)
        {
            if (Canny_Crawl_Top(image, original_frame.cols * 0.2, original_frame.cols * 0.8))
            {
                ROS_INFO("Quit");
                return true;
            }
        }
        
        if(maskInsert && globalOdom == 1)
        {
            imageMask(image,Trace_edge,&original_frame);
        }
        
        Canny_Crawl_L_R(image, original_frame.cols, original_frame.rows, original_frame.rows * 0.6);
        if(EdgeDetect(image,Trace_edge) && maskInsert == 0)
        {
            maskInsert = 1;
            globalOdom = 1;
        }


        for (int i = original_frame.rows * 0.6; i < original_frame.rows - 1; i++)
        {
            G_line_M[i] = (G_border_L[i] + G_border_R[i]) / 2; // 简单取均值
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
    static uint8_t first = 0;
    if (first == 0)
    {
        first = 1;
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
        for (j = G_line_M[i]; j < width - 1; j++)
        {
            if (image[i][j])
            {
                current_border_R[i] = j;
                break;
            }
        }
        G_border_R[i] = current_border_R[i];
        G_border_L[i] = current_border_L[i];
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
        if (abs(tmp[i] - tmp[i - 1]) < 2)
        {
            average += tmp[i];
            average_count++;
        }
    }
    average /= average_count;
    if (average_count > 0.8 * (right - left) && average > 119 - thres)
    {
        ROS_WARN("average Count: %d  average : %d ", average_count, average);
        return true;
    }
    else
    {
        return false;
    }
}

void imageMask(std::vector<std::vector<uint8_t>> &image , int Dir, Mat *original_frame)
{
    // Dir == 1 mask_L  turn Left
    if(Dir == 1)
        for(int i = maskLen_L;i>0;i--)
        {  
            image[119 - (maskLen_L - i)][mask_L[i]] = 255;
            if(original_frame)
                original_frame->at<uint8_t>(119 - (maskLen_L - i),mask_L[i]) = 255;
        }

    // Dir == 2 mask_R  turn Right
    else if(Dir == 2)
        for(int i = maskLen_R;i>0;i--)
        {
            image[119 - (maskLen_R - i)][mask_R[i]] = 255;
            if(original_frame)
                original_frame->at<uint8_t>(119 - (maskLen_R - i),mask_R[i]) = 255;
        }
}

bool EdgeDetect(std::vector<std::vector<uint8_t>> &image,int Trace_edge)
{
    // int Count = 0;
    int status = 0;
    int L_OffMid = 0,R_OffMid = 0;
    for(int i = 118; i > 72; i--)
    {
        if(G_border_L[i] - G_border_L[i+1] < 0)    
            L_OffMid++;
        if(G_border_R[i] - G_border_R[i+1] > 0)    
            R_OffMid++;
    }
    if(Trace_edge == 2 && L_OffMid > 10)
        status = 1;
    else if(Trace_edge == 1 && R_OffMid > 10)
        status = 1;




    // for(int i = 118;i>72;i--)
    // {
    //     if(Trace_edge == 1)
    //     {
    //         if(abs(G_border_R[i]-G_border_R[i+1]) > 20)
    //             status = 1;
            
    //     }
    //     else if(Trace_edge == 2)
    //     {
    //         if(abs(G_border_L[i]-G_border_L[i+1]) > 20)
    //             status = 1;
    //     }
    // }



    // static int status = 0;
    // if(status == 0)
    // {
    //     if(Trace_edge == 1)
    //     {
    //         for(int i = 119;i>70;i--)
    //             if(G_border_L[i] == 0)
    //                 Count++;
            
    //         if(Count > 40)
    //             status ++;
    //     }
    //     else if(Trace_edge == 2)
    //     {
    //         for(int i = 119;i>70;i--)
    //             if(G_border_R[i] == 0)
    //                 Count++;
            
    //         if(Count > 40)
    //             status ++;
    //     }
    // }
    // else if(status == 1)
    // {   
    //     if(Trace_edge == 1)
    //     {
    //         for(int i = 119;i>70;i--)
    //             if(G_border_L[i] > 0)
    //                 Count++;
            
    //         if(Count > 40)
    //             status ++;
    //     }
    //     else if(Trace_edge == 2)
    //     {
    //         for(int i = 119;i>70;i--)
    //             if(G_border_R[i] == 0)
    //                 Count++;
            
    //         if(Count > 40)
    //             status ++;
    //     }

    // }
    // else if(status == 2)
    // {
    //     if(Trace_edge == 1)
    //     {
    //         for(int i = 119;i>70;i--)
    //             if(G_border_L[i] == 0)
    //                 Count++;
            
    //         if(Count > 40)
    //             status ++;
    //     }
    //     else if(Trace_edge == 2)
    //     {
    //         for(int i = 119;i>70;i--)
    //             if(G_border_R[i] == 0)
    //                 Count++;
            
    //         if(Count > 40)
    //             status ++;
    //     }
    // }

    if(status)
    {
        cout<<"radius insert"<<endl;
        return true;
    }
    else 
        return false;
}

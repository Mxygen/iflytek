#include "../include/detect_edge.h"

using namespace cv;
#define road_width 130
uint8_t current_border_L[74] = {0};
uint8_t current_border_R[74] = {0};
uint8_t last_border_L[74] = {0};
uint8_t last_border_R[74] = {0};
uint8_t llast_border_L[74] = {0};
uint8_t llast_border_R[74] = {0};
uint8_t G_paused = 0;


uint8_t G_border_L[120] = {0}; // 左线数组
uint8_t G_border_R[120] = {0}; // 右线数组
uint8_t G_line_M[120] = {0};   // 中线数组


uint8_t G_ucar_mode = 0;
float vehicle_deviation = 0.0;
float vehicle_orientations = 0.0;
float vehicle_linear_speed = 0.0;

const uint8_t weights_rows[45] = // 权重
    {
        1 ,1 ,1 ,1 ,1 ,
        3 ,3 ,3 ,3 ,3 ,
        6 ,6 ,6 ,6 ,6 ,
        35,35,35,35,35,
        70,70,70,70,70,
        35,35,35,35,35,
        10,10,10,10,10,
        5 ,5 ,5 ,5 ,5 ,
        2 ,2 ,2 ,2 ,2 ,
};

PID_typedef directional_pid;

void PID_Parameter_Init(void)
{
    directional_pid.KP = 15.0;
    directional_pid.KI = 0.0;
    directional_pid.KD = 108.0;

    directional_pid.d_value = 0.0;
    directional_pid.last_d_value = 0.0;

    directional_pid.current_error = 0.0;
    directional_pid.last_error = 0.0;
    directional_pid.previous_error = 0.0;
    directional_pid.error_sum = 0.0;
    directional_pid.error_sum_max = 0.0;
    directional_pid.error_sum_min = 0.0;
}

float Positional_PID(PID_typedef *pid_data, float expect_value, float current_error)
{
    float output;
    pid_data->current_error = expect_value - current_error;

    output = pid_data->KP * pid_data->current_error + pid_data->KD * (pid_data->current_error - pid_data->last_error);

    pid_data->last_error = pid_data->current_error;

    return output;
}

float Positional_Incomplete_Differential_PID(PID_typedef *pid_data, float expect_value, float current_error)
{
    float output;
    pid_data->current_error = expect_value - current_error;

    pid_data->d_value = 0.7 * pid_data->KD * (pid_data->current_error - pid_data->last_error) + 0.3 * pid_data->last_d_value;
    output = pid_data->KP * pid_data->current_error + pid_data->d_value;

    pid_data->last_error = pid_data->current_error;

    return output;
}

extern "C" {
    void Canny_Method(unsigned char* data, double lowthreshold, double highthreshold,int Trace_edge,float linear_speed,float orientations)
    {
        Mat original_frame(120, 160, CV_8UC3, data);
        /*图像深度预处理*/
        resize(original_frame, original_frame, Size(160, 120), 0, 0, INTER_LINEAR); // 压缩分辨率
        flip(original_frame, original_frame, 1);                                    // 水平反转
        cvtColor(original_frame, original_frame, COLOR_BGR2GRAY);                   // 原始图像转换灰度图像
        GaussianBlur(original_frame, original_frame, Size(5, 5), 1.5);              // 高斯滤波
        Canny(original_frame, original_frame, lowthreshold, highthreshold);         // Canny算子边缘检测

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

            /*爬取边线*/
            Canny_Crawl_L_R(image, original_frame.cols, original_frame.rows, original_frame.rows * 0.6);

            /*计算中线*/
            for (int i = original_frame.rows * 0.6; i < original_frame.rows - 1; i++)
            {
                
                // if (i >= 90 && G_border_L[i] > 59 && G_border_R[i] < 99)
                // {
                //     continue;
                // }
                // else 
                // {
                    if(Trace_edge==1)
                    {
                        G_line_M[i] = G_border_R[i]-road_width/2;
                    }
                    else if(Trace_edge==2)
                    {
                        G_line_M[i] = G_border_L[i]+2+road_width/2;
                    }
                    else
                    {
                        G_line_M[i] = (G_border_L[i] + G_border_R[i]) / 2; // 简单取均值
                    }
                // }
                // if(i >= 100)
                // {
                //     if(G_border_L[i] > 59 || G_border_L[i-1] > 59 || G_border_L[i-2] > 59)
                //     {
                //         if(G_border_R[i] < 99 || G_border_R[i-1] < 99 || G_border_R[i-2] < 99)
                //         {
                //             G_paused = 1;
                //         }
                //     }
                // }
                
            }
        }
        Error_Calculation();
        Speed_Control(0.05, 0.035, 0.7);
        linear_speed = vehicle_linear_speed;
        orientations = vehicle_orientations;
    }
}

void Canny_Crawl_L_R(const std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height, uint8_t highest)
{
    uint8_t i = 0, j = 0, found_L = 0, found_R = 0;
    for (int i = height - 1; i > highest; i--)
    {
        current_border_L[i] = 0;
        current_border_R[i] = width + 5;
    }

    /*提取最低行边线*/
    for (i = width / 2; i > 0; i--) // 左
    {
        if (image[0][i])
        {
            current_border_L[height - 1] = i;
            found_L = 1;
            break;
        }
    }
    for (i = width / 2; i < width - 1; i++) // 右
    {
        if (image[0][i])
        {
            current_border_R[height - 1] = i;
            found_R = 1;
            break;
        }
    }

    /*提取其他行边线*/
    for (i = height - 2; i > highest; i--)
    {
        for (j = width / 2; j > 0; j--)
        {
            if (image[i][j])
            {
                current_border_L[i] = j;
                break;
            }
        }
        // G_border_L[i] = Calculate_Median(current_border_L[i] , last_border_L[i] , llast_border_L[i]);
        // llast_border_L[i] = last_border_L[i];
        // last_border_L[i] = current_border_L[i];
        G_border_L[i] = current_border_L[i];
    }

    for (i = height - 2; i > highest; i--)
    {

        for (j = width / 2; j < width - 1; j++)
        {
            if (image[i][j])
            {
                current_border_R[i] = j;
                break;
            }
        }
        // G_border_R[i] = Calculate_Median(current_border_R[i] , last_border_R[i] , llast_border_R[i]);
        // llast_border_R[i] = last_border_R[i];
        // last_border_R[i] = current_border_R[i];
        G_border_R[i] = current_border_R[i];
    }
}



void Error_Calculation()
{
    float error = 0;
    float calculate_count = 0;
    for (int i = 0; i < 45; i++)
    {
        error += (G_line_M[72 + i] - 79) * weights_rows[i]; // 权重误差累加
        calculate_count += weights_rows[i];                 // 权重累加
    }
    vehicle_deviation = (error / calculate_count) * 0.00607;                                               // 输出值过大，故乘0.01
    vehicle_orientations = (Positional_Incomplete_Differential_PID(&directional_pid, 0, vehicle_deviation)); // 计算PID
}

void Speed_Control(float starting_speed, float acceleration, float target_speed)
{
  if (vehicle_linear_speed == 0.0)
  {
    vehicle_linear_speed = starting_speed;
  }
  if (vehicle_linear_speed < target_speed)
  {
    vehicle_linear_speed = vehicle_linear_speed + acceleration;
  }
}





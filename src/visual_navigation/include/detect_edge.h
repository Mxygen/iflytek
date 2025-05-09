#ifndef _DETECT_EDGE_H_
#define _DETECT_EDGE_H_
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
#include <string>
#include <sstream>
// #include "visual_headfile.h"

// extern uint8_t G_paused;
void Detect_Edge(cv::Mat &original_frame, cv::Mat &detect_edge_frame, double lowthreshold, double highthreshold);
void Canny_Crawl_L_R(const std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height, uint8_t highest);

extern "C" {
    void Canny_Method(unsigned char* data, double lowthreshold, double highthreshold,int Trace_edge,float linear_speed,float orientations);
}


extern float vehicle_orientations;
extern float vehicle_linear_speed;
typedef struct
{
    float KP;
    float KI;
    float KD;

    float d_value;
    float last_d_value;

    float current_error;
    float last_error;
    float previous_error;
    float error_sum;
    float error_sum_max;
    float error_sum_min;

} PID_typedef;

extern PID_typedef directional_pid;

void PID_Parameter_Init(void);

float Positional_PID(PID_typedef *pid_data, float expect_value, float current_error);

float Positional_Incomplete_Differential_PID(PID_typedef *pid_data, float expect_value, float current_error);

void Error_Calculation(); 


void Speed_Control(float starting_speed, float acceleration, float target_speed);
#endif
#ifndef _PID_H_
#define _PID_H_

#include "visual_headfile.h"

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

void PID_Parameter_Init(float KP,float KI,float KD);

float Positional_PID(PID_typedef *pid_data, float expect_value, float current_error);

float Positional_Incomplete_Differential_PID(PID_typedef *pid_data, float expect_value, float current_error);

#endif
#include "../include/pid.h"

PID_typedef directional_pid;

void PID_Parameter_Init(float KP,float KI,float KD,float Circle_KP,float Circle_KI,float Circle_KD)
{
    directional_pid.KP = KP;
    directional_pid.KI = KI;
    directional_pid.KD = KD;

    directional_pid.Circle_KP = Circle_KP;
    directional_pid.Circle_KI = Circle_KI;
    directional_pid.Circle_KD = Circle_KD;


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

    if(globalOdom == 1)
    {
        pid_data->d_value = 0.7 * pid_data->Circle_KD * (pid_data->current_error - pid_data->last_error) + 0.3 * pid_data->last_d_value;
        output = pid_data->Circle_KP * pid_data->current_error + pid_data->d_value;
    }
    else
    {
        pid_data->d_value = 0.7 * pid_data->KD * (pid_data->current_error - pid_data->last_error) + 0.3 * pid_data->last_d_value;
        output = pid_data->KP * pid_data->current_error + pid_data->d_value;
    }

    pid_data->last_error = pid_data->current_error;

    return output;
}
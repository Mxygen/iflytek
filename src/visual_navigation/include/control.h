#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "visual_headfile.h"

extern float vehicle_orientations;
extern float vehicle_linear_speed;

extern "C" {
    void Error_Calculation(); 
}

void Speed_Control(float starting_speed, float acceleration, float target_speed);

#endif
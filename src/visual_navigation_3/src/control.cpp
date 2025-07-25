#include "../include/control.h"

uint8_t G_ucar_mode = 0;
float vehicle_deviation = 0.0;
float vehicle_orientations = 0.0;
float vehicle_linear_speed = 0.0;

// const uint8_t weights_rows[45] = // 权重
// {
//       1 ,1 ,1 ,1 ,1 ,
//       3 ,3 ,3 ,3 ,3 ,
//       6 ,6 ,6 ,6 ,6 ,
//       35,35,35,35,35,
//       70,70,70,70,70,
//       35,35,35,35,35,
//       10,10,10,10,10,
//       5 ,5 ,5 ,5 ,5 ,
//       2 ,2 ,2 ,2 ,2 ,
// };

const uint8_t weights_rows[45] = // 权重
{
      1 ,1 ,1 ,1 ,1 ,
      3 ,3 ,3 ,3 ,3 ,
      5 ,5 ,5 ,5 ,5 ,
      7 ,7 ,7 ,7 ,7 ,
      9 ,9 ,9 ,9 ,9 ,
      11,11,11,11,11,
      35,35,35,35,35,
      13,13,13,13,13,
      10,10,10,10,10,
};

void Error_Calculation(ros::NodeHandle nh)
{
  float error = 0;
  float calculate_count = 0;
  for (int i = 0; i < 45; i++)
  {
    if(G_line_M[72 + i] == 0)
      continue; 
    error += (G_line_M[72 + i] - 79) * weights_rows[i]; // 权重误差累加
    calculate_count += weights_rows[i];                 // 权重累加
  }

  vehicle_deviation = (error / calculate_count) * 0.00607;           
                                    // 输出值过大，故乘0.01
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
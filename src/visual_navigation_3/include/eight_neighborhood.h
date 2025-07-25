#ifndef _EIGHT_NEIGHBORHOOD_H_
#define _EIGHT_NEIGHBORHOOD_H_

#include "visual_headfile.h"

constexpr int num_VPR = 300;

extern uint8_t G_initial_L[2]; // 左边起点的x，y值
extern uint8_t G_initial_R[2]; // 右边起点的x，y值

extern uint8_t G_border_L[120];
extern uint8_t G_border_R[120];
extern uint8_t G_line_M[120];

extern uint16_t G_points_L[num_VPR][2];
extern uint16_t G_points_R[num_VPR][2];
extern uint16_t G_dir_L[num_VPR];
extern uint16_t G_dir_R[num_VPR];

void Eight_Neighborhood_Crawl_L_R(uint16_t times, const std::vector<std::vector<uint8_t>> &image, uint8_t highest);

void Extract_L(uint16_t total_L);

void Extract_R(uint16_t total_R);

void Eight_Neighborhood_Method(cv::Mat &original_frame);

uint8_t Get_Initial_Point(std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t initial_row);

#endif
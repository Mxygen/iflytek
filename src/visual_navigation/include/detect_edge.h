#ifndef _DETECT_EDGE_H_
#define _DETECT_EDGE_H_

#include "visual_headfile.h"

extern uint8_t G_paused;
void Detect_Edge(cv::Mat &original_frame, cv::Mat &detect_edge_frame, double lowthreshold, double highthreshold);
void Canny_Crawl_L_R(const std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height, uint8_t highest);
bool Canny_Crawl_Top(const std::vector<std::vector<uint8_t>> &image, uint8_t left, uint8_t right);
bool Canny_Method(cv::Mat &original_frame, double lowthreshold, double highthreshold,int Trace_edge, bool Debug);
#endif
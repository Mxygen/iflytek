#ifndef _DETECT_EDGE_H_
#define _DETECT_EDGE_H_

#include "visual_headfile.h"

extern int globalOdom;
void Detect_Edge(cv::Mat &original_frame, cv::Mat &detect_edge_frame, double lowthreshold, double highthreshold);
void Canny_Crawl_L_R(const std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height, uint8_t highest);
bool Canny_Crawl_Top(const std::vector<std::vector<uint8_t>> &image, uint8_t left, uint8_t right);
bool Canny_Method(cv::Mat &original_frame, double lowthreshold, double highthreshold,int Trace_edge, bool Debug);
bool RightDetect(const std::vector<std::vector<uint8_t>> &image, uint8_t left, uint8_t right,int &Dir);
void imageMask(std::vector<std::vector<uint8_t>> &image , int Dir,cv::Mat *original_frame = nullptr);
bool EdgeDetect(std::vector<std::vector<uint8_t>> &image,int Trace_edge);
#endif
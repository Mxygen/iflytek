#ifndef _BINARIZATION_H_
#define _BINARIZATION_H_

#include "visual_headfile.h"
#include <cstdint>
#include <vector>

// void binarization(const cv::Mat& original_frame , cv::Mat& binari_frame);

void Binari_Image(std::vector<std::vector<uint8_t>> &image, uint8_t threshold);

void Add_Border(std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height, uint8_t color);

void Image_Filter(std::vector<std::vector<uint8_t>> &image, uint8_t width, uint8_t height);

uint8_t Otsu_Threshold(const std::vector<std::vector<uint8_t>> &image, int width, int height);
#endif
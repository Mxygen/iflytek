#ifndef _COMMON_FUNCTION_H_
#define _COMMON_FUNCTION_H_

#include "visual_headfile.h"

uint8_t Calculate_Median(uint8_t a, uint8_t b, uint8_t c);

cv::Mat array2Mat(uchar **array, int rows, int cols);

cv::Mat Array2Mat(std::vector<std::vector<uint8_t>> &array);

uchar **Mat2Vec(cv::Mat mat);

uchar **initArray(int rows, int cols);

void freeArray(uchar **array, int rows);

void Mat2Array(cv::Mat mat, uchar **array);

void printArray(uchar **array, int rows, int cols);

#endif
#include "../include/common_function.h"

using namespace cv;
uint8_t Calculate_Median(uint8_t a , uint8_t b , uint8_t c)
{
    if((b <= a && a <= c) || (c <= a && a <= b))return a;
    if((a <= b && b <= c) || (c <= b && b <= a))return b;        
    return c;
}

Mat array2Mat(uchar** array, int rows, int cols) {
    Mat mat(rows, cols, CV_8UC1); // 假设输入的是单通道8位图像
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat.at<uchar>(i, j) = array[i][j];
        }
    }
    return mat;
}

Mat Array2Mat(std::vector<std::vector<uint8_t>>& array) 
{
    int rows = array.size();
    int cols = array[0].size();
    Mat mat(rows, cols, CV_8UC1); // 假设输入的是单通道8位图像
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat.at<uchar>(i, j) = array[i][j];
        }
    }
    return mat;
}

uchar** Mat2Vec(Mat mat) {
    // 动态分配行的数组
    uchar** array = new uchar * [mat.rows];

    // 为每一行分配列的数组
    for (int i = 0; i < mat.rows; ++i) {
        array[i] = new uchar[mat.cols];
    }

    // 将Mat对象的值复制到二维数组中
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            array[i][j] = mat.at<uchar>(i, j);
        }
    }

    return array;
}

uchar** initArray(int rows, int cols) {
    uchar** array = new uchar * [rows];
    for (int i = 0; i < rows; ++i) {
        array[i] = new uchar[cols];
    }
    return array;
}

void freeArray(uchar** array, int rows) {
    for (int i = 0; i < rows; ++i) {
        delete[] array[i];
    }
    delete[] array;
}

void Mat2Array(Mat mat, uchar** array) {
    for (int i = 0; i < mat.rows; ++i) {
        for (int j = 0; j < mat.cols; ++j) {
            array[i][j] = mat.at<uchar>(i, j);
        }
    }
}

void printArray(uchar** array, int rows, int cols) {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::cout << static_cast<int>(array[i][j]) << " ";
        }
        std::cout << std::endl;
    }
}




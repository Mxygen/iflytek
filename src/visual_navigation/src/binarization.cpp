#include "../include/binarization.h"

using namespace cv;

uint8_t Otsu_Threshold(const std::vector<std::vector<uint8_t>>& image, int width, int height)
{
    
    const int GrayScale = 256;
    std::vector<int32_t> HistGram(GrayScale, 0);

    uint32_t amount = 0;
    uint32_t pixel_back = 0;
    uint32_t pixel_integral_back = 0;
    uint32_t pixel_integral = 0;
    int32_t pixel_integral_fore = 0;
    int32_t pixel_fore = 0;
    double omega_back = 0, omega_fore = 0, micro_back = 0, micro_fore = 0, sigmaB = 0, sigma = 0;
    uint8_t value_min = 0, value_max = 0;
    uint8_t threshold = 0;

    /*计算直方图*/
    for (int Y = 0; Y < height; Y++)
    {
        for (int X = 0; X < width; X++) 
        {
            uint8_t pixelValue = image[Y][X];
            HistGram[pixelValue]++;
        }
    }

    /*确定最小和最大灰度值*/
    for (value_min = 0; value_min < GrayScale && HistGram[value_min] == 0; value_min++);
    for (value_max = GrayScale - 1; value_max > value_min && HistGram[value_max] == 0; value_max--);

    if (value_max == value_min) 
    {
        return value_max; // 图像中只有一个颜色
    }
    if (value_min + 1 == value_max) 
    {
        return value_min; // 图像中只有两个颜色
    }

    /* 计算像素总数和灰度值总和*/
    for (int Y = value_min; Y <= value_max; Y++) 
    {
        amount += HistGram[Y];
    }

    for (int Y = value_min; Y <= value_max; Y++) 
    {
        pixel_integral += HistGram[Y] * Y;
    }

    sigmaB = -1;
    for (int Y = value_min; Y < value_max; Y++) 
    {
        pixel_back += HistGram[Y];
        pixel_fore = amount - pixel_back;
        omega_back = static_cast<double>(pixel_back) / amount;
        omega_fore = static_cast<double>(pixel_fore) / amount;
        pixel_integral_back += HistGram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        micro_back = static_cast<double>(pixel_integral_back) / pixel_back;
        micro_fore = static_cast<double>(pixel_integral_fore) / pixel_fore;
        sigma = omega_back * omega_fore * (micro_back - micro_fore) * (micro_back - micro_fore);
        if (sigma > sigmaB)
         {
            sigmaB = sigma;
            threshold = static_cast<uint8_t>(Y);
        }
    }
    return threshold;
}

void Binari_Image(std::vector<std::vector<uint8_t>>& image , uint8_t threshold)
{
    for (size_t i = 0; i < image.size(); ++i) 
    {
        for (size_t j = 0; j < image[i].size(); ++j) 
        {
            image[i][j] = (image[i][j] >= threshold) ? 255 : 0;//比较灰度是否搞过阈值
        }
    }
}

void Add_Border(std::vector<std::vector<uint8_t>>& image , uint8_t width, uint8_t height, uint8_t color)
{
    /*左右边框*/
    for (uint8_t i = height * 0.6; i < height; i++) 
    {
        image[i][0] = color;
        image[i][1] = color;
        image[i][width - 1] = color;
        image[i][width - 2] = color;
    }
    /*限高边框*/
    for (uint8_t i = 0; i < width; i++) 
    {
        image[height * 0.6][i] = color;
        image[height * 0.6+1][i] = color;
    }
}

#define threshold_max	255*5////定义膨胀和腐蚀的阈值区间,此参数可根据自己的需求调节
#define threshold_min	255*2

#define BLACK 0
#define WHITE 255

void Image_Filter(std::vector<std::vector<uint8_t>>& image , uint8_t width, uint8_t height)//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint32_t num = 0;
    /*开始滤波*/
	for (uint16_t i = 1; i < height - 1; i++)
	{
		for (uint16_t j = 1; j < (width - 1); j++)//统计八个方向的像素值
		{
			num =
				image[i - 1][j - 1] + image[i - 1][j] + image[i - 1][j + 1]
				+ image[i][j - 1] + image[i][j + 1]
				+ image[i + 1][j - 1] + image[i + 1][j] + image[i + 1][j + 1];
            /*膨胀腐蚀*/
			if (num >= threshold_max && image[i][j] == WHITE)
			{
				image[i][j] = WHITE;
			}
			if (num <= threshold_min && image[i][j] == BLACK)
			{
				image[i][j] = BLACK;
			}
		}
	}
}



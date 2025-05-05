#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
int main(int argc, char **argv)
{

	for(int i=1; i< argc; i++)
	{
		printf("argument %d is %s \n",i,argv[i]);
		//打印命令的参数
	}
	Mat img = imread("OIP-C.jpg");
	printf("img height %d, img width %d \n",img.rows, img.cols);
	Mat dst;
	resize(img, dst, Size(img.size().width / 2, img.size[0] / 2));
        imwrite("resize.jpg", dst);

	return 0;
}

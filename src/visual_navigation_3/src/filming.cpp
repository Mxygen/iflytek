#include "../include/filming.h"

using namespace cv;
using namespace std;

int image_index = 0;
void Save_Image(const Mat &image)
{

     string filename;
     stringstream ss;
     ss << "photo_" << image_index << ".jpg";
     filename = ss.str();
     image_index++;
     imwrite(filename, image);
     cout << "图片已保存至" << filename << endl;
}

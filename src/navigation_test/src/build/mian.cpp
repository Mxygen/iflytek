#include "iostream"
#include<opencv2/opencv.hpp>
#include "algorithm"
#include "unistd.h"
using namespace std;
using namespace cv;

//比较哪个区域的上下宽度更短，以此来决定最大点数
void compareRegion(const vector<Point>&a, const vector<Point>&b, int &min_point, int &max_point)
{
    int a_min=1000;
    int a_max=0;
    int b_min=1000;
    int b_max=0;
    for(const auto & i : a)
    {
        if(i.y<a_min)
        {
            a_min=i.y;
        }
        if(i.y>a_max)
        {
            a_max=i.y;
        }
    }
    for(const auto & i : b)
    {
        if(i.y<b_min)
        {
            b_min=i.y;
        }
        if(i.y>b_max)
        {
            b_max=i.y;
        }
    }
    min_point=max(a_min,b_min);
    max_point=min(a_max,b_max);
}

bool descending(const vector<Point>&a,const vector<Point>&b)
{
    return a.size()>b.size();
}

void add_point(const vector<Point>&map_point1,const vector<Point>&map_point2,float &map1_average,float &map2_average)
{

    //so the max point number is decided by which top and bottom point is smaller
    int min_point=1000; //find the min point of y
    int max_point=0; // find the max point of y
    int map2_num=0; //the point number of map2
    int sum1_num=0;
    int map1_num=0;
    int sum2_num=0;
    compareRegion(map_point1, map_point2, min_point, max_point);
    if(map_point2[0].y>=min_point&&map_point2[0].y<=max_point)
    {
        sum2_num += map_point2[0].x;
    }
    for(int i=1;i<map_point2.size();i++)
    {
        if(map_point2[i].y>=min_point&&map_point2[i].y<=max_point)
        {
            if(map_point2[i].y!=map_point2[i-1].y)
            {
                sum2_num+=map_point2[i].x;
                map2_num++;
            }
        }
    }
    if(map_point1[0].y>=min_point&&map_point1[0].y<=max_point)
    {
        sum1_num+=map_point1[0].x;
    }
    for(int i=1;i<map_point1.size();i++)
    {
        if(map_point1[i].y>=min_point&&map_point1[i].y<=max_point)
        {
           if(map_point1[i].y!=map_point1[i-1].y)
           {
               sum1_num+=map_point1[i].x;
               map1_num++;
           }
        }
    }
    map1_average=(float)sum1_num/(float)map1_num;
    map1_average-=(640.f/2);//center distance
    map2_average=(float)sum2_num/(float)map2_num;
    map2_average-=(640.f/2);//center distance
}
int main() {
    char ckey = waitKey(0);
    string path1 = "line//line_";
    string path2 = ".jpg";
    int i = 10000;
    while(true) {
//        if(ckey == 27)
//        {
//            return 0;
//        }
//        if(ckey == 13 || ckey == 32) {
            i++;
            string path = path1 + to_string(i) + path2;
            cout<<path<<endl;
            Mat img = imread(path);
            if (img.empty()) {
                return -1;
            }
            imshow("RGB", img);
            Mat hsv;
            Mat crop = img(Range(290, 480), Range(0, 640)); // Slicing to crop the image
            cvtColor(crop, hsv, COLOR_BGR2HSV);
            cv::Scalar white_low = cv::Scalar(0, 0, 131); //hsv
            cv::Scalar white_high = cv::Scalar(181, 24, 255);
            cv::inRange(hsv, white_low, white_high, hsv); //hsv operation
            Mat struct1, struct2;  //dilate and erode
//          struct1 = getStructuringElement(0, Size(3, 3));  //矩形结构元素
            struct2 = getStructuringElement(1, Size(3, 3));  //十字结构元素
            cv::medianBlur(hsv, hsv, 5);  //中值滤波
            cv::erode(hsv, hsv, struct2);//腐蚀
            cv::dilate(hsv, hsv, struct2); //膨胀
            Point p1(0, 0);
            Point p2(150, 0);
            Point p3(0, 100);
            std::vector<Point> points;
            points.push_back(p1);
            points.push_back(p2);
            points.push_back(p3);
            fillConvexPoly(hsv, points, Scalar(0, 0, 0));
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            cv::findContours(hsv, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
            //draw
            //排序 找到第一大和第二大的轮廓
            if(contours.size()<2)
            {
                cout<<"contours size is less than 2"<<endl;
                continue;
            }
            sort(contours.begin(), contours.end(), descending);
            Scalar color(255, 255, 255);
            Mat result_after(hsv.size(), CV_8UC3, Scalar(0, 0, 0));
            drawContours(result_after, contours, static_cast<int>(0), color, 1, LINE_8, hierarchy, 0);
            drawContours(result_after, contours, static_cast<int>(1), color, 1, LINE_8, hierarchy, 0);
            float map1_average, map2_average;
            add_point(contours[1], contours[0], map1_average, map2_average);
            cout << map1_average << endl;
            cout << map2_average << endl;
            float differential = map2_average + map1_average;//得到的图像偏差
            cout << "differential:" << differential << endl;
            imshow("result_after", result_after);
            imshow("HSV", hsv);
//        }
//        ckey = waitKey(0);
        waitKey(200);
//        sleep(10);
//        cv::destroyAllWindows();
    }
    waitKey();//可以去掉
    return 0;
}



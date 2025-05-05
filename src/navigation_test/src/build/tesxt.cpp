// #include "find_copy.h"

// using namespace std;

// int main(){
//     cv::Mat img;
//     int ksy;
//     cv::VideoCapture cap;
//     while(1)
// {
//     cap>>img;
//     LineDetectionMaps LineDetectionMaps_1 = LineDetectionMaps();
//     LineDetectionMaps_1.midline_detect(img);

//     LineDetectionMaps_1.show();

//     ksy=cv::waitKey(1);
    
// }
// }

#include "find_copy.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() 
{
    Mat img;
    int key;
    VideoCapture cap(0);  // Assuming you're capturing from camera index 0

    // Check if the VideoCapture object was associated with a camera
    if (!cap.isOpened()) 
    {
        cerr << "Error opening video stream" << endl;
        return -1;
    }

    while (1) 
    {
        // Capture a new frame from the camera
        cap >> img;

        // Check if the frame is empty
        if (img.empty()) {
            cerr << "Received empty frame" << endl;
            break;
        }

        LineDetectionMaps LineDetectionMaps_1 = LineDetectionMaps();
        LineDetectionMaps_1.midline_detect(img);

        // Assuming `show` is a valid method of `LineDetectionMaps`
        LineDetectionMaps_1.show();

        // Wait for a key press for 1ms
        key = waitKey(1);

        // If the 'q' key is pressed, break from the loop
        if (key == 'q' || key == 27) 
        {  // 27 is the ESC key
            break;
        }
    }
    return 0;
}
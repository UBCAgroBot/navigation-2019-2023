#include "opencv2/opencv.hpp"
#include <iostream>
#include <opencv2/highgui/highgui_c.h>

using namespace std;
using namespace cv;

const String window_detection_name = "HSV Image";
const int max_value_H = 360/2;
const int max_value = 255;
int thresholdq = 10;
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
int dilation_size = 2;
static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}
/*static void hough(int, void *){
    thresholdq = 10, 100);
    
    setTrackbarPos("THRES", "canny", thresholdq);
}*/
int main() {
    namedWindow(window_detection_name, WINDOW_AUTOSIZE);
    namedWindow("canny", WINDOW_AUTOSIZE);
    VideoCapture cap( "cropVid.mp4" );
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    createTrackbar("THRES", "canny", &thresholdq, 500); 
    while (1){

                Mat frame;
                cap >> frame;
                if (frame.empty())
                        break;
        imshow ( "Frame", frame );
        
        char c=(char)waitKey (25);
        if (c==27)
                break;
        Mat color_hsv;
        cvtColor(frame, color_hsv, CV_BGR2HSV);
    
        inRange(color_hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), color_hsv);
        
        imshow(window_detection_name, color_hsv);
        //Vector<Vec4i> lines;
        Mat edges;
        Canny(color_hsv, edges, 50, 200);
        Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 2*dilation_size + 1, 2*dilation_size +1 ),
                                       Point( dilation_size, dilation_size ) );
        dilate(edges, edges, element);

        vector<Vec4i> lines;
        HoughLinesP(edges, lines, 1, CV_PI/180, thresholdq, 10, 250);
        for(size_t i=0; i<lines.size(); i++){
            Vec4i l = lines[i];
            line(edges, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255, 0, 0), 3, LINE_AA);

        }
        imshow("canny", edges);

    }

        cap.release();
        destroyAllWindows();

        return 0;
}

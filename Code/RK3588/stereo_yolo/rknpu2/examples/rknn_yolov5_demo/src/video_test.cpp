#include <opencv2/opencv.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/videostab.hpp>
#include <opencv2/highgui.hpp>
int main(){
    cv::namedWindow("Camera", cv::WINDOW_AUTOSIZE);   // set windows auto
    cv::VideoCapture capture(11);                      // default 0 is the first of camera in computer
    cv::Mat bgr_frame;
    capture.set(cv::CAP_PROP_FPS, 30);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    while (true){
        capture >> bgr_frame;                         // >> is a overload opeator, in C++ Grammar
        if(bgr_frame.empty())
            break;
        cv::imshow("Camera", bgr_frame);
        char c = cv::waitKey(1);                           
        if( c== 27 )
            break;
    }
    capture.release();
}
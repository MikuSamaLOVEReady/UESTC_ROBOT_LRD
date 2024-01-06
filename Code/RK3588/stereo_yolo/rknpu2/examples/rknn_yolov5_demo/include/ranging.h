#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include <cstdlib>
#include <vector>
#include "detection.h"
#include <unistd.h>


using namespace cv;

class Ranging
{
private:
    // rknn_handle hdx;
    cv::VideoCapture vcapture;
    Detection yolov5s;
   
    //new
    Mat cam_matrix_left = (Mat_<double>(3, 3) << 
        4.809895643547006e+02, 0, 0,
        0,4.807599168204821e+02, 0,
        3.362108165786334e+02, 2.298502481932070e+02, 1);
    Mat cam_matrix_right = (Mat_<double>(3, 3) << 
        4.903260126250775e+02, 0, 0,
        0,4.900310486342847e+02, 0,
        3.230124997386542e+02, 2.346297967642670e+02, 1);
    Mat distortion_l = (Mat_<double>(1, 5) <<0.113688825569154,-0.106166584327678, 0,
        0, 0);

    Mat distortion_r = (Mat_<double>(1, 5) <<0.121425307936153,-0.141892782717707, 0,
        0, 0);
    Mat rotate = (Mat_<double>(3, 3) << 
        0.999996295879846, 8.723884080433472e-04, 0.002578209660240,
        -8.682590894537506e-04,0.999998339366207, -0.001602308016931,
        -0.002579603213718,0.001600063527818,0.999995392711370);
    Mat trans = (Mat_<double>(3, 1) << 
         -60.348359844102470,-0.030699794141365, 0.495248628081046);


    cv::Mat mapX1, mapX2, mapY1, mapY2, q, Z;

    int imgw, imgh;
    detect_result_group_t detect_result_group;

public:
    Ranging(int index, int imgw, int imgh);
    void rectifyImage(cv::Mat &oriImgL, cv::Mat &oriImgR);
    void getInfo(cv::Mat &imgl, cv::Mat &imgr, cv::Mat &detBoxes, cv::Mat &info);
    std::vector<float> pic2cam(int u, int v);
    std::vector<int> muban(cv::Mat &left_image, cv::Mat &right_image, const int *coordinates);
    std::vector<cv::Mat> get_range();
    void horizon_estimate(cv::Mat& img, cv::Mat& bboxs,int k);
};

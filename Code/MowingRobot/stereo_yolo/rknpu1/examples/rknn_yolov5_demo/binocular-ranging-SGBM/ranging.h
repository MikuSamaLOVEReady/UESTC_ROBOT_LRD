#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
// #include <opencv2/dnn/dnn.hpp>
#include <cstdlib>
#include <vector>
// #include "detection.h"


using namespace cv;

class StereoCamera
{
private:

    //������-��
    Mat cam_matrix_left = (Mat_<double>(3, 3) <<
        3.591292160052484e+02, 0, 0,
        0, 3.594570152846120e+02, 0,
        3.138604286768604e+02, 2.350696697751391e+02, 1);

    Mat cam_matrix_right = (Mat_<double>(3, 3) <<
        3.583894758876852e+02, 0, 0,
        0, 3.589025244294418e+02, 0,
        2.954799359425563e+02, 2.320517157073290e+02, 1);
    Mat distortion_l = (Mat_<double>(1, 5) << 0.123580251768962, -0.161617548937365, 0,
        0, 0);

    Mat distortion_r = (Mat_<double>(1, 5) << 0.117782368340401, -0.148267462310723, 0,
        0, 0);

    Mat rotate = (Mat_<double>(3, 3) <<
        0.999906059536091, 4.255633699835616e-04, 0.013700036453422,
        -4.624762334654719e-04, 0.999996271484043, 0.002691307070252,
        -0.013698840050911, -0.002697390188875, 0.999902528183337);
    Mat trans = (Mat_<double>(3, 1) <<
        -60.393773136539686, -0.094063741327313, -1.270472451455994);


    Mat q;// = (Mat_<double>(4, 4) << 1, 0, 0, -24.66080, 0, 1, 0, -178.70217, 0, 0, 0, 671.72227, 0, 0, 0.01657, -0.00000);


    // float focal_length = 7.465766094960454e+02;
    // float baseline = 60.134851044411256;
    Mat mapX1, mapX2, mapY1, mapY2;

    //�������
    std::vector<std::string> className = { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
        "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
        "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
        "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
        "hair drier", "toothbrush" };

public:
    StereoCamera() {};
    StereoCamera(int imgh, int imgw);
    void rectifyImage(Mat& oriImgL, Mat& oriImgR, std::vector<Mat>& imgs);
    void stereoProcess(Mat& imgl, Mat& imgr, Mat& detecionBoxes);
    std::vector<float> getDHW(Mat& imgr, Mat points3D, Mat& box);
    std::vector<float> pic2cam(int u, int v);
};

class Ranging
{
private:
    VideoCapture vcapture;
    StereoCamera myCamera;
    // Detection yolov5;
public:
    Ranging() {};
    Ranging(int index);

    void rectifyImage();
    // void detect();
    void get_range();

    std::vector<Mat> imgs;
};

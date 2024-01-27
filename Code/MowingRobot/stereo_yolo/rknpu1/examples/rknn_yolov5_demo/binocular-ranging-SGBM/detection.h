#pragma once

#include <opencv2/opencv.hpp>
#include <math.h>
#include <opencv2/dnn/dnn.hpp>


class Detection
{
private:
    float sigmoid(float x) {
        return static_cast<float> (1.f / (1.f + exp(-x)));
    }
    //anchors
    const float netAnchors[3][6] = { { 10.0, 13.0, 16.0, 30.0, 33.0, 23.0 },{ 30.0, 61.0, 62.0, 45.0, 59.0, 119.0 },{ 116.0, 90.0, 156.0, 198.0, 373.0, 326.0 } };
    //stride
    const float netStride[3] = { 8.0, 16.0, 32.0 };
    int outputNum = 25200;
    const int netWidth = 640; //网络模型输入大小
    const int netHeight = 640;
    float nmsThreshold = 0.45;
    float confThreshold = 0.35;

    cv::dnn::Net model;

public:
    Detection(/* args */) {};
    Detection(std::string netPath);

    cv::Mat Detect(cv::Mat& SrcImg);
    cv::Mat outputParse(cv::Mat netInputImg, std::vector<cv::Mat>& netOutputImg);
};


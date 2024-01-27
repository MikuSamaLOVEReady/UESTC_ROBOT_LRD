#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/core_detect.hpp>
#include <vector>
#include <set>
#include <ctime>
#include<iostream>
#include "detection.h"
#include <mutex>
#include <time.h>

using namespace cv;



Detection::Detection(std::string netPath) {
    try {
        model = dnn::readNet(netPath);
        std::cout << "Load detection model completed !";
    }
    catch (const std::exception&) {
        std::cout << "Load detection model failed !";
        exit(EXIT_FAILURE);
    }
}

Mat imgPreprocess(Mat srcImg) {
    Size shape = srcImg.size();
    Size newShape(640, 640);
    float scaleRatio = std::min( float(newShape.width / shape.width), float(newShape.height / shape.height));
    Size scaleShape(int(scaleRatio * shape.width), int(scaleRatio * shape.height));
    Mat newImg;
    resize(srcImg, newImg, newShape);

    return newImg;

}


Mat Detection::Detect(Mat& srcImg) {

    Mat blob;
    dnn::blobFromImage(srcImg, blob, 1 / 255.0, Size(netWidth, netHeight), cv::Scalar(0, 0, 0), true, false);
    model.setInput(blob);
    std::vector<Mat> netOutputImg;
    model.forward(netOutputImg, model.getUnconnectedOutLayersNames());
    Mat detBox = outputParse(srcImg, netOutputImg);
    return detBox;
}

Mat xywh2xyxy(Mat box) {
    Mat newBox = box.clone();
    newBox.col(0) = box.col(0) - box.col(2) / 2;
    newBox.col(1) = box.col(1) - box.col(3) / 2;
    newBox.col(2) = box.col(0) + box.col(2) / 2;
    newBox.col(3) = box.col(1) + box.col(3) / 2;
    return newBox;
}

Mat Detection::outputParse(Mat srcImg, std::vector<Mat>& netOutputImg) {
    
    Mat preBox = netOutputImg[0], temp(outputNum, 85, CV_32F);
    preBox.copySize(temp);

    Mat clsScore = repeat(preBox.col(4), 1, 80);
    clsScore = clsScore.mul(preBox.colRange(5, 85));

    Mat filtedBox, filtedConf, filtedCls, mask = Mat(preBox.col(4) > confThreshold);

    // 考虑并行加速
    //std::mutex myLock;
    //auto f = [&filtedBox, &filtedConf, &filtedCls, &preBox, &clsScore, &myLock](uchar& value, const int* position) {
    //    if (value == 1) {
    //        double maxScore, minScore;
    //        Point maxPoint;
    //        minMaxLoc(clsScore.row(position[1]), &minScore, &maxScore, 0, &maxPoint);
    //        Mat box = preBox.row(position[1]).colRange(0, 4);

    //        myLock.try_lock();
    //        filtedBox.push_back(box);
    //        filtedConf.push_back(float(maxScore));
    //        filtedCls.push_back(float(maxPoint.x));
    //        myLock.unlock();
    //    }
    //};
    //mask.forEach<uchar>(f);

    double maxScore, minScore;
    Point maxPoint;
    for (int i = 0; i < outputNum; i++) {
        minMaxLoc(clsScore.row(i), &minScore, &maxScore, 0, &maxPoint);
        if (mask.row(i).data != 0 && (maxScore > confThreshold)) {
            filtedBox.push_back(preBox.row(i).colRange(0,4));
            filtedConf.push_back(float(maxScore));
            filtedCls.push_back(float(maxPoint.x));
        }
    }
    if (filtedBox.rows == 0)
        return Mat();

    Mat xyxyBox = xywh2xyxy(filtedBox);
    Mat newBox(filtedBox.rows, 6, CV_32F);
    xyxyBox.copyTo(newBox.colRange(0, 4));
    filtedConf.copyTo(newBox.col(4));
    filtedCls.copyTo(newBox.col(5));
    
    Mat classOffset = filtedCls * 7680.0;
    Mat classOffset_ = repeat(classOffset, 1, 4);
    Mat nmsBox = xyxyBox + classOffset_;

    std::vector<Rect> boxes;
    std::vector<float> scores;
    std::vector<int> idxs;
    for (uchar i = 0; i < nmsBox.rows; i++) {
        boxes.push_back(Rect(Point(nmsBox.at<float>(i, 0), nmsBox.at<float>(i, 1)), Point(nmsBox.at<float>(i, 2), nmsBox.at<float>(i, 3))));
        scores.push_back(filtedConf.at<float>(i, 0));
    }
    dnn::NMSBoxes(boxes, scores, confThreshold, nmsThreshold, idxs);

    Mat finallBoxes, finallclass;
    for (int idx : idxs) {
        finallBoxes.push_back(newBox.row(idx));
    };

    int width = srcImg.cols, height = srcImg.rows;

    float scaleRationH = height / float(netHeight);
    float scaleRationW = width / float(netWidth);
    finallBoxes.col(1) *= scaleRationH;
    finallBoxes.col(3) *= scaleRationH;
    finallBoxes.col(0) *= scaleRationW;
    finallBoxes.col(2) *= scaleRationW;
    
    finallBoxes.col(0).forEach<float>([&width](float& value, const int* position) {value = value < 0 ? 0 : (value > width ? width : value); });
    finallBoxes.col(1).forEach<float>([&height](float& value, const int* position) {value = value < 0 ? 0 : (value > height ? height : value); });
    finallBoxes.col(2).forEach<float>([&width](float& value, const int* position) {value = value < 0 ? 0 : (value > width ? width : value); });
    finallBoxes.col(3).forEach<float>([&height](float& value, const int* position) {value = value < 0 ? 0 : (value > height ? height : value); });

    return finallBoxes;
}

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <iostream>
#include<stdio.h>
#include "ranging.h"
#include<math.h>

using namespace cv;

StereoCamera::StereoCamera(int imgh, int imgw) : mapX1(imgh, imgw, CV_64F),
mapX2(imgh, imgw, CV_64F),
mapY1(imgh, imgw, CV_64F),
mapY2(imgh, imgw, CV_64F),
q(4, 4, CV_64F) {
    auto imgSize = Size(imgw, imgh);

    Mat r1(3, 3, CV_64F), r2(3, 3, CV_64F), p1(3, 4, CV_64F), p2(3, 4, CV_64F);
    stereoRectify(cam_matrix_left, distortion_l, cam_matrix_right, distortion_r,
        imgSize, rotate, trans, r1, r2, p1, p2, q);

     //= (Mat_<double>(4, 4) << 1, 0, 0, -24.66080, 0, 1, 0, -178.70217, 0, 0, 0, 671.72227, 0, 0, 0.01657, -0.00000);


    initUndistortRectifyMap(cam_matrix_left, distortion_l, r1, p1, imgSize, CV_32F, mapX1, mapX2);
    initUndistortRectifyMap(cam_matrix_right, distortion_r, r2, p2, imgSize, CV_32F, mapY1, mapY2);
}

void StereoCamera::rectifyImage(Mat& oriImgL, Mat& oriImgR, std::vector<Mat>& imgs) {
    remap(oriImgL, imgs[0], mapX1, mapY1, INTER_LINEAR);
    remap(oriImgR, imgs[1], mapX2, mapY2, INTER_LINEAR);
}





Mat calcSGBM(Mat imgl, Mat imgr) {
    int 	blockSize = 3;
    auto leftMather = StereoSGBM::create();

    leftMather->setP1(8 * 1 * 9 * 9);
    leftMather->setP2(32 * 1 * 9 * 9);
    leftMather->setBlockSize(blockSize);
    leftMather->setMinDisparity(0);
    leftMather->setNumDisparities(16 * 4);
    leftMather->setUniquenessRatio(3);
    leftMather->setSpeckleWindowSize(200);
    leftMather->setSpeckleRange(32);
    leftMather->setDisp12MaxDiff(1);
    leftMather->setPreFilterCap(10);
    leftMather->MODE_SGBM_3WAY;
    auto rightMather = ximgproc::createRightMatcher(leftMather);
    auto imgSize = imgl.size(); //Դ���ж�ά�Ƚ������û���������
    Mat tempL, tempR;
    pyrDown(imgl, tempL);
    pyrDown(imgr, tempR);

    float sclarRatio = float(imgl.cols / tempL.cols);
    Mat disparityL, disparityR;
    leftMather->compute(tempL, tempR, disparityL);
    rightMather->compute(tempR, tempL, disparityR);
    resize(disparityL, disparityL, imgSize, INTER_AREA);
    resize(disparityR, disparityR, imgSize, INTER_AREA);
    // ΪʲôҪ�óߴ�����ϵ����������ֵ�أ�
    disparityL = sclarRatio * disparityL;
    disparityR = sclarRatio * disparityR;
    //imshow("disparityR", disparityR);
    Ptr<ximgproc::DisparityWLSFilter> wlsFilter = ximgproc::createDisparityWLSFilterGeneric(leftMather);
    wlsFilter->setLambda(8000);
    wlsFilter->setSigmaColor(1.2);
    wlsFilter->setDepthDiscontinuityRadius(7);
    wlsFilter->setLRCthresh(24);
    Mat disparity(imgl.rows, imgl.cols, CV_64F);
    wlsFilter->filter(disparityL, imgl, disparity, disparityR);
    // �������û�õ����Բ�����
    // Mat confidenMap = wlsFilter->getConfidenceMap();
    //��ʵ�Ӳ��ΪSGBM�㷨�õ����Ӳ��ǡ�16�ģ�
    disparity = disparity / 16.0;
    // confidenMap = confidenMap / 16.0;
    //imshow("disparity", disp8u);
    //waitKey(10);
    
    return disparity;

}



std::vector<float> StereoCamera::pic2cam(int u, int v) {
    std::vector<float> loc;
    loc.push_back((u - cam_matrix_right.at<double>(0, 2)) * q.at<double>(2, 3) / cam_matrix_right.at<double>(0, 0));
    loc.push_back((v - cam_matrix_right.at<double>(1, 2)) * q.at<double>(2, 3) / cam_matrix_right.at<double>(1, 1));
    return loc;
}

/*std::vector<float> StereoCamera::getDHW(Mat& imgr, Mat points3D, Mat& box) {
    int x1 = box.at<float>(0, 0), y1 = box.at<float>(0, 1), x2 = box.at<float>(0, 2), y2 = box.at<float>(0, 3), cls = box.at<float>(0, 5);
    Mat fusedPoint = points3D.mul(points3D);

    std::vector<Mat> channels;
    split(fusedPoint.clone(), channels);
    fusedPoint = channels[0] + channels[1] + channels[2];
    fusedPoint.forEach<float>([](float& value, const int* position) {value = sqrt(value); }); // �Ծ��󿪷�
    //std::cout << "fusedPoint = " << std::endl << " " << fusedPoint << std::endl << std::endl;
    int mid_pixel = int((x1 + x2) / 2);
    std::vector<float> mid = pic2cam(imgr.cols / 2, imgr.rows); //������
    std::vector<float> loc_tar = pic2cam(mid_pixel, imgr.rows);
    float alfa = atan((loc_tar[0] - mid[0]) / q.at<double>(2,3));
    //std::cout << alfa << std::endl;
    Mat subPoint = fusedPoint.rowRange(y1, y2).colRange(x1, x2);
 
    // �����쳣ֵ
    Mat lt10000;
    for (int i = 0; i < subPoint.rows; i++) {
        for (int j = 0; j < subPoint.cols; j++) {
            float v = subPoint.at<float>(i, j);
            if (v < 1000000)
                lt10000.push_back(v);
        }
    };
    //std::cout << "lt10000 = " << std::endl << " " << lt10000 << std::endl << std::endl;

    sort(lt10000, lt10000, SORT_EVERY_COLUMN + SORT_ASCENDING);

    if (lt10000.empty()) {
        std::cout << "Function<getDHW>: There is no value litter than 10000 !" << std::endl;
        putText(imgr, "error", Point((x1 + x2) / 2, (y1 + y2) / 2), FONT_HERSHEY_PLAIN, 1.4, (0, 0, 255), 1);
        return std::vector<float> { 0, 0, 0, 0 };
    }
    else
    {
        int midLoc = lt10000.rows / 2;
        float median = lt10000.at<float>(0, midLoc);

        std::vector<float> ltPoint = pic2cam(x1, y1);
        std::vector<float> rbPoint = pic2cam(x2, y2);
        float xx1 = ltPoint[0], yy1 = ltPoint[1], xx2 = rbPoint[0], yy2 = rbPoint[1]; //��Щֵ�ķ�Χ��Լ����
        float f = q.at<double>(2, 3);
        float f1 = sqrt(xx1 * xx1 + yy1 * yy1 + f * f);
        float w1 = median * sqrt((xx1 - xx2) * (xx1 - xx2) / 4) / f1;
        float h1 = median * sqrt((yy1 - yy2) * (yy1 - yy2) / 4) / f1;
        float f2 = sqrt(xx2 * xx2 + yy2 * yy2 + f * f);
        float w2 = median * sqrt((xx2 - xx1) * (xx2 - xx1) / 4) / f2;
        float h2 = median * sqrt((yy2 - yy1) * (yy2 - yy1) / 4) / f2;
        
        w1 = (w1 + w2) /10;
        h1 = (h1 + h2) / 10;
        median /= 10;

        // ��ͼ���ϻ�����Ϣ
        char a[50], b[50];
        std::string cname = className[cls];
        sprintf(a,"dis:%.2fcm  %s", median, cname.c_str());
        String dc = a;
        sprintf(b, "W: %.2fcm H: %.2fcm", w1, h1);
        String wh = b;

        putText(imgr, dc, Point(x1, y1), FONT_HERSHEY_PLAIN, 2.2, Scalar(0, 0, 255), 2);
        putText(imgr, wh, Point(x1, y2), FONT_HERSHEY_PLAIN, 2.2, Scalar(0, 0, 255), 2);
        rectangle(imgr, Point(int(x1), int(y1)),
                Point(int(x2), int(y2)), Scalar(0, 0, 255));
        //std::cout << median << w1 << h1 << alfa << std::endl;
        //���и�alfaֵ�����Ƕ�
        return std::vector<float> { median, w1, h1, alfa };
    }
}*/

void StereoCamera::stereoProcess(Mat& imgl, Mat& imgr, Mat& detecionBoxes) {
    // ֱ��ͼ���⻯
    Mat tempL1, tempR1;
    cvtColor(imgl, tempL1, COLOR_BGR2GRAY);
    cvtColor(imgr, tempR1, COLOR_BGR2GRAY);
    /*
    equalizeHist(tempL1, tempL1);
    equalizeHist(tempR1, tempR1);
    // ��
    Mat kernel = (Mat_<float>(3, 3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);
    for (uint32_t i = 0; i < 3; i++) {
        filter2D(tempL1, tempL1, -1, kernel);
        filter2D(tempR1, tempR1, -1, kernel);
        medianBlur(tempL1, tempL1, 3);
        medianBlur(tempR1, tempR1, 3);
    }
    */
    //����ƥ��
    Mat disparity = calcSGBM(tempL1, tempR1);
    //normalize(disparity, disparity, 0, 255, NORM_MINMAX, CV_8U);
    /*
    Mat erodekernel = Mat::ones (3, 3, CV_8U);
    erode(disparity, disparity, erodekernel);  //Դ���еĲ���1�ǵ���������
    medianBlur(disparity, disparity, 5);
    */

    //std::cout << disparity << std::endl;
    Mat disparity_1;
    normalize(disparity, disparity_1, 1.0, 0.0, NORM_MINMAX);
    imshow("disparity", disparity*16);
    //imwrite("./disparity.jpg", disparity);
    waitKey(5);
    Mat point_3d = Mat(480, 640, CV_64FC(3));
    //Mat point_3d;
    reprojectImageTo3D(disparity, point_3d, q,1);
    std::vector<std::vector<float>> info;
    // for (uchar i = 0; i < detecionBoxes.rows; i++) {
    //     Mat box = detecionBoxes.row(i);
    //     info.push_back(getDHW(imgr, point_3d, box));
    // };
    imshow("res", imgr);

    waitKey(1);
}

Ranging::Ranging(int index) //: yolov5("yolov5n.onnx")
{
    if (!vcapture.open(index)) {
        exit(EXIT_FAILURE);
    }
    else {
        vcapture.set(CAP_PROP_FRAME_WIDTH, 1280);  //width=1280
        vcapture.set(CAP_PROP_FRAME_HEIGHT, 480);  //height=480

        Mat frame, lframe, rframe;
        vcapture.read(frame);
        int height = frame.rows;
        int width = frame.cols;
        int imgWidth = width / 2;

        lframe = frame.colRange(0, imgWidth);
        rframe = frame.colRange(imgWidth, width);

        imgs = std::vector<Mat>{ lframe, rframe };
        myCamera = StereoCamera(height, imgWidth);

        std::cout << "Initializing" << std::endl;
    }

}

void Ranging::get_range() {
    Mat frame, lframe, rframe;
    vcapture.read(frame);
    int height = frame.rows;
    int width = frame.cols;
    int imgWidth = width / 2;
    Mat detBoxes;
    
    //cv::namedWindow("res");
    while (!frame.empty()) {
        lframe = frame.colRange(0, imgWidth);
        rframe = frame.colRange(imgWidth, width);
        int64 t = getTickCount();
        myCamera.rectifyImage(lframe, rframe, imgs);
        // detBoxes = yolov5.Detect(rframe);
        t = getTickCount() - t;
        imshow("res", rframe);
        waitKey(1);
        int64 end = getTickCount();
        if (detBoxes.rows) {
            myCamera.stereoProcess(lframe, rframe, detBoxes);
        }
        // std::cout << "detection:" << t * 1000 / getTickFrequency() << "ms, or" << t / getTickFrequency() << "s" << std::endl;
        int64 t1 = getTickCount() - end;
        std::cout << "SGBM:" << t1 * 1000 / getTickFrequency() << "ms, or" << t1 / getTickFrequency() << "s" << std::endl;
        std::cout << "FPS:" << int(1 / (t1 / getTickFrequency() + t/getTickFrequency())) << std::endl;
        vcapture >> frame;
        std::cout << "while" << std::endl;
    };

}

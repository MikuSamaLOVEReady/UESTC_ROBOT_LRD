/******************** (C) COPYRIGHT 2024 UPBot **********************************
* File Name          : ranging.cpp
* Current Version    : V1.0
* Author             : Rockchip & linyuehang
* Date of Issued     : 2024.01.07 zhanli@review
* Comments           : 双目摄像头目标测距算法，需要依赖OpenCV以及目标检测模块
********************************************************************************/
#include <iostream>
#include <stdio.h>
#include "ranging.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <dlfcn.h>

// OpenCV相关
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <tuple>
#include <string>
#include <algorithm>


// 上面重复无用的头文件应该删除
// #include <ros/ros.h>
using namespace cv;

// 重映射函数
void Ranging::rectifyImage(Mat &oriImgL, Mat &oriImgR) 
{
    remap(oriImgL, oriImgL, mapX1, mapX2, cv::INTER_LINEAR);
    remap(oriImgR, oriImgR, mapY1, mapY2, cv::INTER_LINEAR);
}

// 像素坐标转相机坐标
std::vector<float> Ranging::pic2cam(int u, int v) 
{
    //(u,v)->(x,y)"(loc[0],loc[1])"
    std::vector<float> loc;
    loc.push_back((u - cam_matrix_right.at<double>(0, 2)) * q.at<double>(2, 3) / cam_matrix_right.at<double>(0, 0));
    loc.push_back((v - cam_matrix_right.at<double>(1, 2)) * q.at<double>(2, 3) / cam_matrix_right.at<double>(1, 1));
    return loc;
}

// 模板匹配
std::vector<int> Ranging::muban(Mat &left_image, Mat &right_image, const int *coordinates) 
{
    int x1 = coordinates[0], y1 = coordinates[1], x2 = coordinates[2], y2 = coordinates[3];
    // 获取目标框
    Mat tpl = right_image.rowRange(max(y1 - 2, 0), min(y2 + 2, 479)).colRange(x1, x2); 
    // 待匹配图像，极线约束，只需要同水平区域
    Mat target = left_image.rowRange(max(y1 - 20, 0), min(y2 + 20, 479)).colRange(0, 639); 
    int th = tpl.rows, tw = tpl.cols;
    Mat result;

    // 匹配方法：归一化相关系数即零均值归一化互相关
    matchTemplate(target, tpl, result, TM_CCOEFF_NORMED);  
    double minVal, maxVal;
    Point minLoc, maxLoc; 
    // 得到匹配点坐标
    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc); 
    Point tl = maxLoc, br;
    // 这个地方应该修改为宏
    // 转为像素坐标系 640 - 1
    br.x = min(maxLoc.x + tw, 639); 
    // 转为像素坐标系 480 - 1
    br.y = min(maxLoc.y + th, 479); 

    /*
    // 展示匹配结果
    br.x = min(maxLoc.x + tw, 319);
    br.y = min(maxLoc.y + th, 239);
    rectangle(target, tl, br, (0, 255, 0), 3);
    imshow("match-", target);
    waitKey(2);
    */
   
    std::vector<int> maxloc;
    maxloc.push_back(maxLoc.x); 
    maxloc.push_back(maxLoc.y); 
    return maxloc;
}

// 这个函数暂时不知道作用
void Ranging::horizon_estimate(Mat& img, Mat& bboxs,int k)
{
    //保证摄像头与地面平行
	int x1 = bboxs.at<float>(k, 0);
	int x2 = bboxs.at<float>(k, 2);
	int y1 = bboxs.at<float>(k, 1);
	int y2 = bboxs.at<float>(k, 3);
    float Conf = bboxs.at<float>(k, 4);
    int cls = bboxs.at<float>(k, 5);
	float Y_B, Y_H;
	cv::Mat edge, grayImage;
	std::vector<cv::Point> idx;
    cv::Mat tpl = img.rowRange(y1, min(y2+5,479)).colRange(x1, x2); // 取感兴趣范围
    //Mat target = left_image.rowRange(max(y1 - 20, 0), min(y2 + 20, 479)).colRange(0, 639);
	cv::Mat Z = Mat::zeros(2, tpl.cols, CV_32FC1);
	cvtColor(tpl, grayImage, COLOR_BGR2GRAY); 
	GaussianBlur(grayImage,grayImage,Size(5,5),0);
	Canny(grayImage, edge, 120, 180, 3);  //提取边缘，获取与地面接触点
    //cv::imshow("1",edge);
    //cv::waitKey(1);
    float cluster[650];
	for (int i = 0;i<650;i++)
	{
	    cluster[i] = 0;
	}
	int y_b, y_h;
    int j = 0;
	for (int i = 0; i < x2-x1; i++)
	{
		//Mat temp = edge.rowRange(max(y1, 0), min(y2 + 4, 479)).colRange(x1, x2);
		Mat temp = edge.col(i); //取第i列
        // std::cout << "temp: " <<temp << std::endl;
		cv::findNonZero(temp, idx);
        std::vector<float> point_b = pic2cam(x1 + i, 240);  //转为相机坐标系
		std::vector<float> point_H = pic2cam(320, 240);
        float alfa = atan((point_b[0] - point_H[0]) / q.at<double>(2, 3));
        if (idx.size() < 1)
        {
            Z.at<float>(0, i) = 0;
		    Z.at<float>(1, i) = alfa;
            continue;
        }
		int y_b = idx[idx.size() - 1].y + y1; //
        y_b = int(y_b + y_b*0.03);
		int y_h = 240;
		point_b = pic2cam(x1 + i, y_b);  //转为相机坐标系
		point_H = pic2cam(320, y_h);
		Y_B = point_b[1];
		Y_H = point_H[1];
	
        float H_c = 60; //摄像头离地高度，单位mm
        float theta = 0; //摄像头与地面夹角，弧度
		float d = (1/cos(theta)* cos(theta)) * q.at<double>(2, 3) * H_c / (Y_B - Y_H)- H_c*tan(theta);
		alfa = atan((point_b[0] - point_H[0]) / q.at<double>(2, 3));
        //cout << "d: " << d << endl;
        if (d > 700)
        {d = 0;}
		Z.at<float>(0, i) = d/cos(alfa);
		Z.at<float>(1, i) = alfa;
	}

    this->Z = Z.clone();
}

void Ranging::getInfo(Mat &imgL, Mat &imgR, Mat &detBoxes, Mat &info)
{
    Mat imgGrayL, imgGrayR;
    cvtColor(imgL, imgGrayL, COLOR_BGR2GRAY);
    cvtColor(imgR, imgGrayR, COLOR_BGR2GRAY);
    // 优化：这个地方的imgR_weight没有起到任何作用
    // 纯粹浪费CPU计算了
    Mat imgR_weight = imgR.clone();
    Mat infoRow;

    for (uchar i = 0; i < detBoxes.rows; i++)
    {
        // 获得位置信息
        int x1 = detBoxes.at<float>(i, 0);
        int y1 = detBoxes.at<float>(i, 1);
        int x2 = detBoxes.at<float>(i, 2);
        int y2 = detBoxes.at<float>(i, 3);
        // 获取置信度信息
        float Conf = detBoxes.at<float>(i, 4);
        int cls    = detBoxes.at<float>(i, 5);

        // 当目标框偏左、偏右或者过大，略去该物体  
        if (x1 > 600 || x2 < 50 || y1 < 5 || y2 > 475 || x1 < 2 || x2 > 590 || abs(x2 - x1) > 550) {
            continue;
        }

        // 绘制目标框
        rectangle(imgR, Point(int(x1), int(y1)),Point(int(x2), int(y2)), Scalar(0, 0, 255)); 
       
        // 检测的是右图，利用模板匹配去找左图的位置
        int coordinates[4] = {x1, y1, x2, y2};
        // 模板匹配
        std::vector<int> disp_pixel = muban(imgGrayL, imgGrayR, coordinates); 

        // 这里用disp_x 有点奇怪？用dispDiffX
        // 计算水平视差 左图匹配的x坐标和右图的x坐标之差
        float disp_pixel_x = disp_pixel[0] - x1;
        // 垂直的误差没有起到作用
        float disp_pixel_y = disp_pixel[1] - y1; 
        // 0.12为模板匹配产生的误差，为经验值，通过拟合得到
        disp_pixel_x = (int)(disp_pixel_x + disp_pixel_x * 0.12); 
        
        //Mat disp_matrix = Mat(1, 1, CV_32F, Scalar(disp_pixel_x)), disp_pixel_xyz;
        // 定义视差矩阵，所有值均为水平视差，方便转换为三维坐标，并具有水平距离信息
        Mat disp_matrix = Mat(imgGrayL.rows, imgGrayL.cols, CV_32F, Scalar(disp_pixel_x)); 
        Mat threed_pixel_xyz, threedImage; 
        // 通过quat将显示图像从2d映射到3d
        reprojectImageTo3D(disp_matrix, threedImage, q, false);
        // 图像的3个通道，每一像素点求平方
        threed_pixel_xyz = threedImage.mul(threedImage); 
        std::vector<Mat> channels;
        // 求平方后的结果，分离为3个通道，然后求和即是欧式距离
        split(threed_pixel_xyz.clone(), channels);

        // 计算欧式距离 = r^2 + g^2 + b^2
        threed_pixel_xyz = channels[0] + channels[1] + channels[2]; 
        threed_pixel_xyz.forEach<float>([](float &value, const int *position) { value = sqrt(value); }); // 获得距离d
       
        int mid_pixel = int((x1 + x2) / 2);

        // 计算角度，从像素坐标转为相机坐标
        // 这里用结构体比用vector更加节省CPU运算
        std::vector<float> mid     = pic2cam(imgGrayR.cols / 2, imgGrayR.rows); 
        std::vector<float> loc_tar = pic2cam(mid_pixel, imgGrayR.rows);
        float alfa = atan((loc_tar[0] - mid[0]) / q.at<double>(2, 3));


        // 距离太近，视差过大
        if (disp_pixel_x > 240)        
        {
            char cm[15];
            //sprintf(cm, "cannot match !");
            sprintf(cm, "%d , %.2f", cls,Conf);
            putText(imgR, cm, Point((x1), (y1)), FONT_HERSHEY_PLAIN, 2.2, Scalar(0, 0, 255), 2);
            infoRow = (Mat_<float>(1, 4) << -1, -1, -1, -1);
            infoRow.copyTo(info.row(i));
            continue;
        }
        else
        {
            float median = threed_pixel_xyz.at<float>((int)(y1 + y2) / 2, (int)(x1 + x2) / 2);

            std::vector<float> ltPoint = pic2cam(x1, y1);
            std::vector<float> rbPoint = pic2cam(x2, y2);
            float xx1 = ltPoint[0], yy1 = ltPoint[1], xx2 = rbPoint[0], yy2 = rbPoint[1]; //计算宽高
            float f = q.at<double>(2, 3);
            float f1 = sqrt(xx1 * xx1 + yy1 * yy1 + f * f); //推导得出
            //float w1 = median * sqrt((xx1 - xx2) * (xx1 - xx2) / 4) / f1;
            float h1 = median * sqrt((yy1 - yy2) * (yy1 - yy2) / 4) / f1;
            float f2 = sqrt(xx2 * xx2 + yy2 * yy2 + f * f);
            //float w2 = median * sqrt((xx2 - xx1) * (xx2 - xx1) / 4) / f2;
            float h2 = median * sqrt((yy2 - yy1) * (yy2 - yy1) / 4) / f2;
            float w1 = sqrt(pow((threedImage.at<cv::Vec3f>(y2, x1)[0] - threedImage.at<cv::Vec3f>(y2, x2)[0]), 2) +
                            pow((threedImage.at<cv::Vec3f>(y2, x1)[1] - threedImage.at<cv::Vec3f>(y2, x2)[1]), 2) +
                            pow((threedImage.at<cv::Vec3f>(y2, x1)[2] - threedImage.at<cv::Vec3f>(y2, x2)[2]), 2));

            w1 = w1 / 10;
            h1 = (h1 + h2) / 10;
            median /= 10;
            if (median > 120) //过远测距误差较大
            {
                //char tf[9];
                //sprintf(tf, "Too far!");
                char cm[15];
                //sprintf(cm, "cannot match !");
                sprintf(cm, "%d , %.2f", cls, Conf);
                putText(imgR, cm, Point((x1), (y1)), FONT_HERSHEY_PLAIN, 2.2, Scalar(0, 0, 255), 2);
                infoRow = (Mat_<float>(1, 4) << -1, -1, -1, -1);
                infoRow.copyTo(info.row(i));
                continue;
            }

            // char dc[50], wh[50];
            // std::string cname = className[cls + 1];
            // sprintf(dc, "dis:%.2fcm  %d", median, cls);
            // sprintf(wh, "W: %.2fcm H: %.2fcm alfa: %2f", w1, h1, alfa);
            // putText(imgR, dc, Point(x1, y2), FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 255), 2);
            // putText(imgR, wh, Point(x1, y1), FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 255), 1.5);
		    
            //返回数据
            infoRow = (Mat_<float>(1, 4) << median, w1, h1, alfa);
            infoRow.copyTo(info.row(i));
        };
    }
    //cv::imshow("kk",imgR);
    //cv::waitKey(1);
}

Ranging::Ranging(int index, int imgw, int imgh) : //初始化
    mapX1(imgh, imgw, CV_64F),   //初始化矩阵  ，用于计算无畸变和修正转换映射。                                
    mapX2(imgh, imgw, CV_64F),                                     
    mapY1(imgh, imgw, CV_64F),
    mapY2(imgh, imgw, CV_64F),
    q(4, 4, CV_64F),
    imgw(imgw),
    imgh(imgh)
{
    // Z = Mat::zeros(2, 1, CV_32FC1);
    vcapture = cv::VideoCapture(index);
    //vcapture.set(CAP_PROP_FOURCC, CAP_OPENCV_MJPEG);
    vcapture.set(cv::CAP_PROP_FPS, 30);
    vcapture.set(cv::CAP_PROP_FRAME_WIDTH, imgw * 2);
    vcapture.set(cv::CAP_PROP_FRAME_HEIGHT, imgh);
    vcapture.set(cv::CAP_PROP_BUFFERSIZE, 1);
                    

    auto imgSize = Size(imgw, imgh);
    Mat r1(3, 3, CV_64F), r2(3, 3, CV_64F), p1(3, 4, CV_64F), p2(3, 4, CV_64F);

    // 立体校正
    stereoRectify(cam_matrix_left.t(), distortion_l, cam_matrix_right.t(), distortion_r,
                    imgSize, rotate.t(), trans, r1, r2, p1, p2, q);

    // 计算无畸变和修正转换映射
    initUndistortRectifyMap(cam_matrix_left.t(), distortion_l, r1, p1, imgSize, CV_32F, mapX1, mapX2); 
    // 计算无畸变和修正转换映射
    initUndistortRectifyMap(cam_matrix_right.t(), distortion_r, r2, p2, imgSize, CV_32F, mapY1, mapY2);
		
    // RKNN_Create(&hdx, modelPath); // 初始化检测模型
	std::cout<< " ******************* CAMERA  initialization ********************" << std::endl;
}

/**---------------------------------------------------------------------
* Function    : detObjectRanging
* Description : 摄像头帧读取、目标检测以及目标测距任务
* Author      : hongchuyuan
* Date        : 2023/12/13 zhanli@review 719901725@qq.com
*---------------------------------------------------------------------**/
std::vector<Mat> Ranging::detObjectRanging() 
{
    
	double rang_old, rang_now;
	// rang_old = ros::Time::now().toSec(); //测试运行时间
    Mat frame, lframe, rframe;
    // 获取视频帧
    vcapture >> frame;
    // 显示原始图像
    //cv::imshow("frame",frame);
    
    if (!frame.empty())
    {
        int64 t = getTickCount();
        // 拷贝左图
        Mat lframe(frame.colRange(0, imgw).clone()); 
        // 拷贝右图
        Mat rframe(frame.colRange(imgw, imgw * 2).clone()); 
   
        rectifyImage(lframe, rframe); 

        // 这里拷贝一次，然后再yolo里面再拷贝一次，以及函数传参也拷贝了一次?
        cv::Mat Rframe = rframe.clone();
        // yolov5s.outputPars修改为detectImage()
        detect_result_group = yolov5s.outputParse(Rframe);

        if (detect_result_group.count<=0)
        {
            std::cout<<"detect nothing"<<std::endl;
        }

        // 定义矩阵，存储目标检测内容，存储格式(x,y,x,y,conf,cls)
        Mat detBoxes(detect_result_group.count, 5, CV_32F); 
        char text[256];
        // 存储目标检测内容 (x,y,x,y,conf,cls)

        // 这里应该写一个函数drawDetectBobox()
        for (int i = 0; i < detect_result_group.count; i++)
        {
            detect_result_t *det_result = &(detect_result_group.results[i]);
	        sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);

            // printf("%s @ (%d %d %d %d) %f\n", det_result->name, det_result->box.left, det_result->box.top,
            // det_result->box.right, det_result->box.bottom, det_result->prop);
            
            int xmin = det_result->box.left;
            int ymin = det_result->box.top;
            int xmax = det_result->box.right;
            int ymax = det_result->box.bottom;
            
            rectangle(Rframe, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(256, 0, 0, 256), 3);
            putText(Rframe, text, cv::Point(xmin, ymin + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
            
            // (x,y) (x,y) conf 
            detBoxes.at<float>(i, 0) = xmin;
            detBoxes.at<float>(i, 1) = ymin;
            detBoxes.at<float>(i, 2) = xmax;
            detBoxes.at<float>(i, 3) = ymax;
            detBoxes.at<float>(i, 4) = det_result->prop;
            // 实验测试，过滤过大的误检框
            // float ratio = (xmax - xmin) * (ymax - ymin) / 308480.;
            // if (ratio > 0.7)
            // {
            //     detBoxes.at<float>(i, 5) = -1;
            //     continue;
            // }
        }

        // 存储测距信息，存储格式：距离d，宽w，高h，角度alpha
        Mat info(detect_result_group.count, 4, CV_32F); 
        
        // 如果有检测目标
        if (detect_result_group.count)
        {
            // getInfo()修改为calObjectRanging()
            // 在这里测量目标的距离并存储到info中
            // 优化:这个地方重复的将左右视图转换为灰度图，浪费CPU的算力
            getInfo(lframe, rframe, detBoxes, info);

            //show stereo distance
            //zai zhe li kai ke neng cheng xu hui beng le

            // for(int i=0; i<info.rows;i++)
            // {
            //     detect_result_t *det_result = &(detect_result_group.results[i]);

            //     float median = info.at<float>(i, 0);
            //     float w1 = info.at<float>(i, 1);
            //     float h1 = info.at<float>(i, 2);
            //     float alfa = info.at<float>(i, 3);
                
            //     int x1 = det_result->box.left;
            //     int y1 = det_result->box.top;
            //     int x2 = det_result->box.right;
            //     int y2 = det_result->box.bottom;
            //     char dc[50], wh[50];
            //     sprintf(dc, "dis:%.2fcm ", median);
            //     sprintf(wh, "W: %.2fcm H: %.2fcm ", w1, h1);
            //     putText(Rframe, dc, Point(x1, y2), FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 255), 2);
            //     putText(Rframe, wh, Point(x1, y1), FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 255), 1.5);
            // }
        }

        // 这个地方应该整为一个函数，负责显示帧率图像
        t = getTickCount() - t;
        char fps[50];
        sprintf(fps, "fps: %d", int(1 / (t / getTickFrequency())));
        putText(Rframe, fps, Point(20, 20), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 255), 1.5);
        
        cv::imshow("k",Rframe);
        cv::waitKey(1);

        return std::vector<Mat>{rframe, detBoxes, info};
    }
    return std::vector<Mat>{rframe};
}

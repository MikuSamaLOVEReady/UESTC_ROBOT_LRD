/******************** (C) COPYRIGHT 2024 UPBot **********************************
* File Name          : main.cpp
* Current Version    : V1.0
* Date of Issued     : 2024.01.96 zhanli@review
* Comments           : UPbot机器人双目目标测距算法
********************************************************************************/
#include <iostream>
#include <stdio.h>
// #include "lidar.h"
// #include <ros/ros.h>
// #include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include "ranging.h"
// #include "fusion.h"
#include <string>
#include <pthread.h>
#include <mutex>
#include <sys/time.h>
// #include "detection.h"
/*#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>*/
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <ctime>
#include <queue>

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)

Ranging r(12, 640, 480);
std::queue<std::vector<cv::Mat>> frameInfo;
std::mutex g_mutex;

/**---------------------------------------------------------------------
* Function    : RangingNodeTask
* Description : 双目测距节点任务，完成摄像头帧读取、目标检测以及目标测距任务
* Author      : hongchuyuan
* Date        : 2023/12/13 zhanli@review 719901725@qq.com
*---------------------------------------------------------------------**/
void *RangingNodeTask(void *args)		
{
	while (true)
	{
		// std::cout<<" ************ enter ranging *********** "<<std::endl;
		std::vector<cv::Mat> result = r.detObjectRanging();
		g_mutex.lock();
		for (uchar i = 0; i < frameInfo.size(); i++)		// 只保存当前最新的图片帧信息
		frameInfo.pop();
		frameInfo.push(result);
		g_mutex.unlock();
	}
}

int main(int argc, char **argv)
{
	pthread_t tids[1];		// 执行ranging线程
	int ret = pthread_create(&tids[0], NULL, RangingNodeTask, NULL);

	while(1){
		usleep(150000);
	}
	return 0;
}

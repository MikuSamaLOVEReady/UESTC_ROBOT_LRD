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

void *ranging(void *args)		// ranging线程
{
	while (true)
	{
		// std::cout<<" ************ enter ranging *********** "<<std::endl;
		std::vector<cv::Mat> result = r.get_range();
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
	int ret = pthread_create(&tids[0], NULL, ranging, NULL);

	while(1){
		usleep(150000);
	}

	return 0;
}

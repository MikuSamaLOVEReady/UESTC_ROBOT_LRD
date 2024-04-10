#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "rknn_yolov5_demo/ranging.h"
#include <string>
#include <pthread.h>
#include <mutex>
#include <sys/time.h>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include <ctime>
#include <queue>
#include "rknn_yolov5_demo/dis_info.h"
#include "rknn_yolov5_demo/dis_info_array.h"

//LRD_TEST
#include <unistd.h>
#include "rknn_yolov5_demo/Utils/UESTC_queue.h"

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)

//Ranging r(0, 640, 480);
std::queue<std::vector<cv::Mat>> frameInfo;
std::mutex g_mutex;

// void *ranging(void *args)		// ranging线程
// {
// 	ros::Publisher dis_pub_;
// 	ros::NodeHandle nh; 
//     dis_pub_ = nh.advertise<rknn_yolov5_demo::dis_info_array>("ceju_info",10);
// 	while (true)
// 	{
// 		// std::cout<<" ************ enter ranging *********** "<<std::endl;
// 		std::vector<cv::Mat> result = r.get_range();
// 		cv ::Mat info = result[2];
// 		rknn_yolov5_demo::dis_info_array dis_array;
// 		if(!info.empty())
// 		{
// 			for(int i=0;i<info.rows;i++)
// 			{
// 				// std::cout<<"1"<<std::endl;
// 				if(info.at<float>(i,0)>0&&info.at<float>(i,0)<200)
// 				{
// 					rknn_yolov5_demo::dis_info data;
// 					data.distance = info.at<float>(i,0);
// 					data.width = info.at<float>(i,1);
// 					data.height = info.at<float>(i,2);
// 					data.angle = info.at<float>(i,3);
// 					dis_array.dis.push_back(data);
// 					std::cout<<data.distance<<std::endl;
// 				}	
// 			}
			
//         dis_pub_.publish(dis_array);
// 		// if (!dis_array.dis.empty())
// 		// {
// 		// 	for (const auto& data : dis_array.dis)
// 		// 	{
// 		// 		ROS_INFO("distance: %.2f, width: %.2f, height: %.2f, angle: %.2f",
// 		// 				data.distance, data.width, data.height, data.angle);
// 		// 	}
// 		// }

		
// 		}
// 		// else{
// 		// 	 std::cerr << "Info matrix is empty!" << std::endl;
// 		// }
// 		g_mutex.lock();
// 		for (uchar i = 0; i < frameInfo.size(); i++)		// 只保存当前最新的图片帧信息
// 		frameInfo.pop();
// 		frameInfo.push(result);
// 		g_mutex.unlock();
// 	}
	
// }


void* Record_Cam(void *args) {

 	cv::VideoCapture capture;
	Mat frame, lframe, rframe;
	int imgw = 640;
	int imgh = 480;
	capture = cv::VideoCapture(0);
	if( !capture.isOpened() )
	{
		std::cout<< " can not open Video  Capture" << std::endl;
		return nullptr;
	}
	capture.set(cv::CAP_PROP_FPS, 30);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, imgw * 2);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, imgh);
	capture.set(cv::CAP_PROP_BUFFERSIZE, 1);

    /// Init Writer
    VideoWriter writer;
    int encoder = VideoWriter::fourcc('M','J','P','G');
	std::string filename = "video.avi";
	double fps = capture.get(cv::CAP_PROP_FPS);
	writer.open(filename , encoder , fps , cv::Size(640,480),true);
	if( !writer.isOpened() )
	{
		std::cout<< " can not open Video Writer" << std::endl;
		return nullptr;
	}else {
		std::cout<< "SUCCESS !!!" << std::endl;
	}

	while(1){
		capture >> frame;
		if( !frame.empty() )
		{
			//std::cout<< "SUCCESS !!!" << std::endl;
			Mat rframe(frame.colRange(imgw, imgw * 2).clone()); //拷贝右图
			writer << rframe ;
			char c = waitKey(10);
			if(c == 'q') break;
		}
	}

	/// release writer
	writer.release();
	return 0;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "stereo");
	ros::NodeHandle nh; 
	char cwd[10240];
	if(getcwd(cwd, sizeof(cwd)) != NULL) std::cout << "WORKING DIR" << cwd << std::endl;
	
	UESTC_STL::mutex_queue<Mat> frame_queue{};

	pthread_t tids[2];		// 执行ranging线程
	//int ret = pthread_create(&tids[0], NULL, ranging, NULL);
	
	int record_thd = pthread_create(&tids[1] , NULL , Record_Cam , NULL);	///录制操作与ranging会冲突使用摄像头接口
	if(record_thd !=0 ){
		std::cout <<  "Record_Cam Thread Fail" << std::endl;
	}else{
		std::cout <<  "Record_Cam Thread SUCCESS" << std::endl;

	}
	//pthread_join(tids[0],NULL); /// 线程等待 需滞后
	pthread_join(tids[1],NULL);
	
	return 0;
}

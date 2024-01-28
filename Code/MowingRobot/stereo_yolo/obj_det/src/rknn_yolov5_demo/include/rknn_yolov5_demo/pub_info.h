#ifndef PUB_INFO_H
#define PUB_INFO_H
#include <ros/ros.h>
#include <thread>
#include "rknn_yolov5_demo/dis_info.h"
#include "rknn_yolov5_demo/dis_info_array.h"
#include "rknn_yolov5_demo/ranging.h"

class Pub_info{
    public:
    Pub_info(){
        dis_pub_ = nh_.advertise<rknn_yolov5_demo::dis_info_array>("ceju_info",10);
        thread_1 = std::thread(&Pub_info::pub_dis,this);
        // thread_2 = std::thread(&Pub_info::)

    };
    ~Pub_info(){
        thread_1.join();
    }
    void pub_dis();
    public:
    ros::NodeHandle nh_;
    ros::Publisher dis_pub_;
    rknn_yolov5_demo::dis_info_array dis_array_;
    rknn_yolov5_demo::dis_info data;
    std::thread thread_1;
    // std::thread thread_2;

    

};
#endif
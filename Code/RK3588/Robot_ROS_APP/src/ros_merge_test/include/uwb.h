#include <ros/ros.h>
#include <mutex>
#include <boost/asio.hpp>
#include <iostream>
#include <string>
#include <cstdint>
#include "type.h"
#include <queue>
#include <chrono>
#ifndef __UWB_H__
#define __UWB_H__


namespace uwb_slam{

class Uwb
{
public:
    Uwb();
    void Run();
    bool checknewdata();
    void feed_imu_odom_pose_data();
    void UartUSBRead();
       
public:
    int pre_seq = -1;
    int cur_seq = -1;
    uint8_t tmpdata[13];
    float x, y, theta, distance;
    
    Uwb_data uwb_data_;
    std::mutex mMutexUwb;
};
};

#endif

#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <utility>
#include <queue>
#include <fstream>
#include "ros_merge_test/RawImu.h"
#include "type.h"
#include "uwb.h"
#include <opencv2/opencv.hpp>

#ifndef ALIGN_H
#define AlIGN_H
namespace uwb_slam{
    class Align
    {
    public:
        Align(){};
        void Run();
        void wheel_odomCB(const nav_msgs::Odometry& wheel_odom);
        void imuCB(const ros_merge_test::RawImu& imu);
        void odomCB(const nav_msgs::Odometry& odom);

    public:
        ros::NodeHandle nh_;
        ros::Subscriber wheel_odom_sub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber odom_sub_;
        Imu_odom_pose_data imu_odom_;
        Uwb_data uwb_data_;
        ros::Time tmp ;

        ros::Time odom_tmp_ ;
        bool write_data_ = false;
        cv::Mat img1;
        std::queue<std::pair<Imu_odom_pose_data,Uwb_data>> data_queue;
        std::shared_ptr<uwb_slam::Uwb> uwb_;

    };
};
#endif

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mutex>
#include "uwb.h"
#ifndef SENDDATA_H
#define SENDDATA_H

namespace uwb_slam{
class Senddata{
    public:
    Senddata(){};
    void publishOdometry( std::shared_ptr<uwb_slam::Uwb>uwb);
    void Run(std::shared_ptr<uwb_slam::Uwb>uwb);
    void odomCB(const nav_msgs::Odometry& odom);

    std::mutex mMutexSend;
    private:
    ros::Publisher position_pub_;
    ros::Subscriber odom_sub_;
    ros::NodeHandle nh_;
    nav_msgs::Odometry odom_;//odom的消息类型
    nav_msgs::Odometry sub_odom_;//odom的消息类型
};
}

#endif
/******************** (C) COPYRIGHT 2023 UPBot **********************************
* File Name          : senddata.cpp
* Current Version    : V1.0
* Date of Issued     : 2023.12.13 zhanli@review
* Comments           : UWB数据的发送
********************************************************************************/
#include "senddata.h"
namespace uwb_slam{

/**---------------------------------------------------------------------
* Function    : Senddata::Run
* Description : UWB位置发布以及odm数据的订阅
* Date        : 2023/12/13 zhanli@review
*---------------------------------------------------------------------**/
void Senddata::Run(std::shared_ptr<uwb_slam::Uwb>uwb){
    // 初始化了一个名为loop_rate的ros::Rate对象,频率设置为10赫兹
    ros::Rate loop_rate(10);
    // 初始化一个ROS发布者，用于发布nav_msgs::Odometry类型的消息
    // 主题被设置为"uwb_odom"，队列大小为50
    position_pub_ = nh_.advertise<nav_msgs::Odometry>("uwb_odom", 50);
    // 初始化了一个ROS订阅者，用于订阅"odom"主题。它指定了当在该主题上接收到
    // 消息时，将调用Senddata类的odomCB回调函数。队列大小被设置为10
    odom_sub_ = nh_.subscribe("odom", 10, &Senddata::odomCB,this);

    while(ros::ok()){
        // 按照10Hz频率发布uwb信息
        publishOdometry(uwb);
        ros::spinOnce();
        // 用于控制循环速率
        loop_rate.sleep();
    }
}

void Senddata::odomCB(const nav_msgs::Odometry& odom){
    // 这个地方接收的是轮速里程计的信息
    // 包含位置和姿态
    sub_odom_ = odom;
    return;
}

/**---------------------------------------------------------------------
* Function    : Senddata::publishOdometry
* Description : 发布UWB里程计数据，这里读取的数据到底是什么，依旧存在疑问
* Date        : 2023/12/13 zhanli@review
*---------------------------------------------------------------------**/
void Senddata::publishOdometry(std::shared_ptr<uwb_slam::Uwb> uwb)
{

    std::mutex mMutexSend;
    
    ros::Time current_time = ros::Time::now();

    // 设置 Odometry 消息的头部信息
    odom_.header.stamp = current_time;   // 这个地方获取的时间是否会存在问题?
    odom_.header.frame_id = "odom";      // 设置坐标系为 "map"
    odom_.child_frame_id = "base_link";  // 设置坐标系为 "base_link"
    
    // 填充 Odometry 消息的位置信息
    odom_.pose.pose.position.x = uwb->x;
    odom_.pose.pose.position.y = uwb->y;
    odom_.pose.pose.position.z = 0.0;
    

    // 填充 Odometry 消息的姿态信息（使用四元数来表示姿态）
    // tf2::Quaternion quat;
    // quat.setRPY(0, 0, uwb->theta);  
    
    // 设置了 yaw 角度，其他 roll 和 pitch 设置为 0
    // odom.pose.pose.orientation.x = quat.x();
    // odom.pose.pose.orientation.y = quat.y();
    // odom.pose.pose.orientation.z = quat.z();
    // odom.pose.pose.orientation.w = quat.w();

    // 从里程计拿到姿态信息
    odom_.pose.pose.orientation.x = sub_odom_.pose.pose.orientation.x;
    odom_.pose.pose.orientation.y = sub_odom_.pose.pose.orientation.y;
    odom_.pose.pose.orientation.z = sub_odom_.pose.pose.orientation.z;
    odom_.pose.pose.orientation.w = sub_odom_.pose.pose.orientation.w;

    // 发布 Odometry 消息
    position_pub_.publish(odom_);
}
} // namespace uwb_slam
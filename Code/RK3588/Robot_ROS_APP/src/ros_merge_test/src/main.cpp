/******************** (C) COPYRIGHT 2023 UPBot **********************************
* File Name          : main.cpp
* Current Version    : V1.0
* Date of Issued     : 2023.12.13 zhanli@review
* Comments           : UPbot割草机器人项目传感器融合定位入口函数
********************************************************************************/
#include "../include/system.h"
#include "../include/uwb.h"
#include <iostream>
#include <ros/ros.h>
#include <thread>
#include "senddata.h"


/**---------------------------------------------------------------------
* Function    : main
* Description : 多传感器融合定位的入口函数
* Date        : 2023/12/13 zhanli@review
*---------------------------------------------------------------------**/
int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "locate_info_pub_node"); 

    std::shared_ptr<uwb_slam::Mapping>  mp        = std::make_shared<uwb_slam::Mapping>();
    std::shared_ptr<uwb_slam::Uwb>      uwb       = std::make_shared<uwb_slam::Uwb>();
    std::shared_ptr<uwb_slam::Senddata> sender    = std::make_shared<uwb_slam::Senddata>();
    std::shared_ptr<uwb_slam::Align>    align     = std::make_shared<uwb_slam::Align>();
    
    mp->uwb_    = uwb;
    align->uwb_ = uwb;

    // uwb serried read
    std::thread uwb_trd([&uwb]() {
        uwb->Run();
    });

    // 建图部分暂时没有使用到
    /*std::thread map_trd([&mp]() {
        mp->Run();
    });*/

    std::thread sender_trd([&sender, uwb]() {
        sender->Run(uwb);
    });

    std::thread align_trd([&align]() {
        align->Run();
    });
      
    // Start the ROS node's main loop  
    ros::spin(); 
}

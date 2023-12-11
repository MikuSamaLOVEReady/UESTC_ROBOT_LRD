#include "../include/system.h"
#include "../include/uwb.h"


#include <iostream>
#include <ros/ros.h>
#include <thread>
#include "senddata.h"



int main(int argc, char** argv)
{
    ros::init(argc, argv, "locate_info_pub_node"); // Initialize the ROS node

    std::shared_ptr<uwb_slam::System>system = std::make_shared<uwb_slam::System>();
    std::shared_ptr<uwb_slam::Mapping>mp = std::make_shared<uwb_slam::Mapping>();
    std::shared_ptr<uwb_slam::Uwb>uwb = std::make_shared<uwb_slam::Uwb>();
    std::shared_ptr<uwb_slam::Senddata>sender = std::make_shared<uwb_slam::Senddata>();
    std::shared_ptr<uwb_slam::Align>align = std::make_shared<uwb_slam::Align>();

    // uwb_slam::System* system = new uwb_slam::System();
    // uwb_slam::Mapping* mp = new uwb_slam::Mapping();
    // uwb_slam::Uwb*  uwb = new uwb_slam::Uwb();
    // uwb_slam::Senddata* sender = new uwb_slam::Senddata();
    
    
    system->Mapping_ = mp;
    system->Mapping_->uwb_ = uwb;
    system->Uwb_  = uwb;
    system->Sender_ = sender;
    system->Align_ = align;
    

    mp->uwb_ = system->Uwb_;
    align->uwb_=system->Uwb_;


    //  control data fllow in system
    std::thread rose_trd ([&system]() {
        system->Run();
    });
    // uwb serried read
    std::thread uwb_trd([&uwb]() {
        uwb->Run();
    });
    // build map
    /*std::thread map_trd ([&mp]() {
        mp->Run();
    });*/

    std::thread sender_trd ([&sender, uwb]() {
        sender->Run(uwb);
    });

    std::thread align_trd ([&align]() {
        align->Run();
    });

    ros::spin(); // Start the ROS node's main loop
    //System->run()
}

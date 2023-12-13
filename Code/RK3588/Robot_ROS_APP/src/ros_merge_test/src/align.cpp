/******************** (C) COPYRIGHT 2023 UPBot **********************************
* File Name          : align.cpp
* Current Version    : V1.0
* Date of Issued     : 2023.12.13 zhanli@review
* Comments           : 传感器数据对齐
********************************************************************************/
#include "align.h"

namespace uwb_slam{

    void Align::Run()
    {
        tmp = ros::Time::now();
        ros::Time tmp1 = ros::Time::now();
        ros::Time tmp2 = ros::Time::now();

        // 这个地方分别订阅了原始IMU、轮式里程计、里程计(推测是UWB)
        wheel_odom_sub_= nh_.subscribe("wheel_odom",10,&Align::wheel_odomCB,this);
        imu_sub_       = nh_.subscribe("raw_imu",10,&Align::imuCB,this);
        odom_sub_      = nh_.subscribe("odom",10,&Align::odomCB,this);

        std::ofstream outfile("data.txt",std::ofstream::out);
        if(outfile.is_open())
        {
            img1 = cv::Mat(200, 200, CV_8UC1, cv::Scalar(255,255,255));
            cv::imshow("Image1", img1);
            int key2 = cv::waitKey(0);
            // if(key2 =='w'){
            //     bool write_data_ = true;
            // }
            
        //    int count=0;
            // while(write_data_){
            while(1){

                int key3 = cv::waitKey(1);
                if(key3 == 'w'){
                    break;
                }
                if(tmp!=imu_odom_.imu_data_.imu_t_)
                {
                    // outfile <<"imu_odom_: "<< "imu_timestamp  "<<"imu_linear_acc_x_y_z  "<<"imu_angular_x_y_z  "<<
                    // "odom_vxy  "<<"odom_angle_v_  "<<"\n";
                    // if(tmp1!=uwb_->uwb_data_.uwb_t_&& tmp2!=odom_tmp_){
                    outfile << std::left << std::setw(12)<<"imu_odom_: "<< std::setw(10)<< imu_odom_.imu_data_.imu_t_.sec << '.' <<std::setw(11)<< imu_odom_.imu_data_.imu_t_.nsec   << std::setw(11)
                    <<imu_odom_.imu_data_.a_[0]  << std::setw(11)<<imu_odom_.imu_data_.a_[1]  << std::setw(11)<<imu_odom_.imu_data_.a_[2]  << std::setw(12)
                    <<imu_odom_.imu_data_.w_[0]  << std::setw(12)<<imu_odom_.imu_data_.w_[1]  << std::setw(12)<<imu_odom_.imu_data_.w_[2]  << std::setw(11)
                    <<imu_odom_.vxy_  << std::setw(11)<<imu_odom_.angle_v_  << std::setw(7)
                    <<"   pose: "  << std::setw(1)<<(odom_tmp_-tmp2).sec<<'.'<<std::setw(12)<<(odom_tmp_-tmp2).nsec<<std::setw(11)
                    <<imu_odom_.pose_[0]  << std::setw(12)<<imu_odom_.pose_[1]  << std::setw(2)<<imu_odom_.pose_[2]  << std::setw(12)
                    <<imu_odom_.quat_[0]  << std::setw(12)<<imu_odom_.quat_[1]  << std::setw(12)<<imu_odom_.quat_[2]  << std::setw(12)<<imu_odom_.quat_[3]  << std::setw(6)
                    <<"   uwb: "  << std::setw(1)<<(uwb_->uwb_data_.uwb_t_-tmp1).sec<<'.'<<std::setw(13)<<(uwb_->uwb_data_.uwb_t_-tmp1).nsec   << std::setw(9)
                    <<uwb_->uwb_data_.x_  << std::setw(9)<<uwb_->uwb_data_.y_<<"\n";
                    tmp1 = uwb_->uwb_data_.uwb_t_;
                    tmp2 = odom_tmp_;
                    // }
                //     else if(tmp1!=uwb_->uwb_data_.uwb_t_){
                //         outfile <<"imu_odom_: "<< imu_odom_.imu_data_.imu_t_ <<"*"
                //          <<imu_odom_.imu_data_.a_[0]<<"*"<<imu_odom_.imu_data_.a_[1]<<"*"<<imu_odom_.imu_data_.a_[2]<<"*"
                //          <<imu_odom_.imu_data_.w_[0]<<"*"<<imu_odom_.imu_data_.w_[1]<<"*"<<imu_odom_.imu_data_.w_[2]<<"*"
                //          <<imu_odom_.vxy_<<"*"<<imu_odom_.angle_v_<<"*"
                //          <<"pose: "<<"****************************"
                //          <<"uwb: "<<uwb_->uwb_data_.uwb_t_<<"*"<<uwb_->uwb_data_.x_<<"*"<<uwb_->uwb_data_.y_<<"\n";
                //          tmp1 = uwb_->uwb_data_.uwb_t_;
                //     }
                //     else if(tmp2!=odom_tmp_){
                //         outfile <<"imu_odom_: "<< imu_odom_.imu_data_.imu_t_ <<"*"
                //          <<imu_odom_.imu_data_.a_[0]<<"*"<<imu_odom_.imu_data_.a_[1]<<"*"<<imu_odom_.imu_data_.a_[2]<<"*"
                //          <<imu_odom_.imu_data_.w_[0]<<"*"<<imu_odom_.imu_data_.w_[1]<<"*"<<imu_odom_.imu_data_.w_[2]<<"*"
                //          <<imu_odom_.vxy_<<"*"<<imu_odom_.angle_v_<<"*"
                //          <<"pose: "
                //          <<imu_odom_.pose_[0]<<"*"<<imu_odom_.pose_[1]<<"*"<<imu_odom_.pose_[2]<<"*"
                //          <<imu_odom_.quat_[0]<<"*"<<imu_odom_.quat_[1]<<"*"<<imu_odom_.quat_[2]<<"*"<<imu_odom_.quat_[3]<<"*"
                //          <<"uwb: "<<"****************************"<<"\n";
                //          tmp2 = odom_tmp_;

                //    }
                //    else {
                //         outfile <<"imu_odom_: "<< imu_odom_.imu_data_.imu_t_ <<"*"
                //          <<imu_odom_.imu_data_.a_[0]<<"*"<<imu_odom_.imu_data_.a_[1]<<"*"<<imu_odom_.imu_data_.a_[2]<<"*"
                //          <<imu_odom_.imu_data_.w_[0]<<"*"<<imu_odom_.imu_data_.w_[1]<<"*"<<imu_odom_.imu_data_.w_[2]<<"*"
                //          <<imu_odom_.vxy_<<"*"<<imu_odom_.angle_v_<<"*"
                //          <<"pose: "<<"****************************"
                //          <<"uwb: "<<"****************************"<<"\n";
                //    }
                    tmp = imu_odom_.imu_data_.imu_t_;
                        
                    // tmp1 = uwb_->uwb_data_.uwb_t_;
                    
                    // if(count>300)
                    //     break;
                }
            }

            outfile.close();
            std::cout<< "Data written to file." << std::endl;
        }
        else{
            std::cout<<"file can not open"<<std::endl;
        }
        
    }
    
    void Align::wheel_odomCB(const nav_msgs::Odometry& wheel_odom)
    {
        imu_odom_.vxy_= wheel_odom.twist.twist.linear.x;
        imu_odom_.angle_v_ = wheel_odom.twist.twist.angular.z;
        // imu_odom_.pose_[0] = wheel_odom.pose.pose.position.x;
        // imu_odom_.pose_[1] = wheel_odom.pose.pose.position.y;
        // imu_odom_.pose_[2] = wheel_odom.pose.pose.position.z;
        // imu_odom_.quat_[0] = wheel_odom.pose.pose.orientation.x;
        // imu_odom_.quat_[1] = wheel_odom.pose.pose.orientation.y;
        // imu_odom_.quat_[2] = wheel_odom.pose.pose.orientation.z;
        // imu_odom_.quat_[3] = wheel_odom.pose.pose.orientation.w;
        return;
    }
    
    void Align::imuCB(const ros_merge_test::RawImu& imu)
    {
        imu_odom_.imu_data_.imu_t_ = imu.header.stamp;
        imu_odom_.imu_data_.a_[0] = imu.raw_linear_acceleration.x;
        imu_odom_.imu_data_.a_[1] = imu.raw_linear_acceleration.y;
        imu_odom_.imu_data_.a_[2] = imu.raw_linear_acceleration.z;

        imu_odom_.imu_data_.w_[0] = imu.raw_angular_velocity.x;
        imu_odom_.imu_data_.w_[1] = imu.raw_angular_velocity.y;
        imu_odom_.imu_data_.w_[2] = imu.raw_angular_velocity.z;

        return;
    }

    /**---------------------------------------------------------------------
    * Function    : odomCB
    * Description : 里程计的回调函数, 定期会被ROS调用传参，这个函数不能做过于耗时
    *               的操作
    * Input       : nav_msgs::Odometry& odom : 里程计输入结构体
    * Date        : 2023/12/13 zhanli@review
    *---------------------------------------------------------------------**/
    void Align::odomCB(const nav_msgs::Odometry& odom)
    {
        odom_tmp_ = odom.header.stamp;
        imu_odom_.pose_[0] = odom.pose.pose.position.x;
        imu_odom_.pose_[1] = odom.pose.pose.position.y;
        imu_odom_.pose_[2] = odom.pose.pose.position.z;
        
        imu_odom_.quat_[0] = odom.pose.pose.orientation.x;
        imu_odom_.quat_[1] = odom.pose.pose.orientation.y;
        imu_odom_.quat_[2] = odom.pose.pose.orientation.z;
        imu_odom_.quat_[3] = odom.pose.pose.orientation.w;
    }
};




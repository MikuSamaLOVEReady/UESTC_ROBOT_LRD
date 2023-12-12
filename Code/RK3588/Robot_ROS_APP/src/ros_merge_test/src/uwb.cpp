/******************** (C) COPYRIGHT 2023 UPBot **********************************
* File Name          : uwb.cpp
* Current Version    : V1.0
* Date of Issued     : 2023.12.13 zhanli@review
* Comments           : UWB数据驱动, 负责从串口读取USB并发布出去
********************************************************************************/
#include "uwb.h"
#include <cmath>

#define PI acos(-1)

namespace uwb_slam{

    //
    Uwb::Uwb(){
    }

    void Uwb::Run() {
        while(1){
            // 这个地方不控制速率?
            this->UartUSBRead();
        }
    }


    void Uwb::UartUSBRead()
    {
        try {
            boost::asio::io_service io;
            boost::asio::serial_port serial(io, "/dev/ttyUSB0"); // 替换成你的串口设备路径

            serial.set_option(boost::asio::serial_port_base::baud_rate(115200));                                         // 设置波特率
            serial.set_option(boost::asio::serial_port_base::character_size(8));                                         // 设置数据位
            serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));       // 设置校验位
            serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));  // 设置停止位

            uint8_t tmpdata[12];
            
            size_t bytesRead = boost::asio::read(serial, boost::asio::buffer(tmpdata, 12)); // 读取串口数据
            // std::cerr << "after read" << std::endl;

            // for (int i=0;i< bytesRead;i++)
            // {
            //     std::cout << "Received data: " << std::hex<<static_cast<int>(tmpdata[i]) ;
            // }
            memcpy(&this->distance, &tmpdata[3], sizeof(distance));
            memcpy(&this->theta, &tmpdata[7], sizeof(theta));
            /*this->x = cosf(theta/180*PI)*distance+1000;
            this->y = sinf(theta/180*PI)*distance+1000;
            this->theta = theta;*/
            this->uwb_data_.x_ = cosf(theta/180*PI)*distance+1000;
            this->uwb_data_.y_ = sinf(theta/180*PI)*distance+1000;
            this->uwb_data_.uwb_t_ = ros::Time::now(); 
            
            // uwb_data_queue_.push(uwb_data_);
        
            //std::cout << "uwb_data_: " << uwb_data_.uwb_t_<< std::endl;
            // cur_seq = static_cast<int>(tmpdata[3]);
            //std::cout << "****** cur _ sequence: " <<  cur_seq << "x: " << x << " y: " << y <<std::endl;
            // if( cur_seq  - pre_seq != 1){
            //     std::cout << "****** cur _ sequence: " << cur_seq << "pre _ sequence: " << pre_seq << " ******\n";
            // }
            // pre_seq = static_cast<int>(tmpdata[3]);

            std::cout << "theta: " << theta << " distance: " << distance << std::endl;
            
        } catch (const std::exception& ex) {
            std::cerr << "[ERR]: uwb.cpp::Uart USB read data exception: " 
                << ex.what() << std::endl;
        }
    }
    
    void fusion()
    {
    }
};



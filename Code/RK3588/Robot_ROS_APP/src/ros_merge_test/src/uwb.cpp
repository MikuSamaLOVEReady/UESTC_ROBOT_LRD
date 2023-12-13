/******************** (C) COPYRIGHT 2023 UPBot **********************************
* File Name          : uwb.cpp
* Current Version    : V1.0
* Date of Issued     : 2023.12.13 zhanli@review
* Comments           : UWB数据驱动, 负责从串口读取USB并发布出去，这个代码可能存在以下
                       改进：1) 创建固定的串口读取对象serial_port 2) 修改代码为异步
                       读取。3) 确认串口数据的长度
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
            // UartUSBRead 这个地方本身就是同步读取串口，是阻塞的函数
            this->UartUSBRead();
        }
    }

    /**---------------------------------------------------------------------
    * Function    : UartUSBRead
    * Description : 通过串口读取数据，目前这段代码存在部分问题：1) 每次都重复创建
    *               串口读取对象，可能会影响性能。
    * Date        : 2023/12/13 zhanli@review
    *---------------------------------------------------------------------**/
    void Uwb::UartUSBRead()
    {
        try {
            boost::asio::io_service io;
            boost::asio::serial_port serial(io, "/dev/ttyUSB0");                                                         // 替换成你的串口设备路径

            serial.set_option(boost::asio::serial_port_base::baud_rate(115200));                                         // 设置波特率
            serial.set_option(boost::asio::serial_port_base::character_size(8));                                         // 设置数据位
            serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));       // 设置校验位
            serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));  // 设置停止位

            uint8_t tmpdata[12];
            // 读取串口数据
            size_t bytesRead = boost::asio::read(serial, boost::asio::buffer(tmpdata, 12)); 

            // UWB获取的数据是半径R和角度Theta
            memcpy(&this->distance, &tmpdata[3], sizeof(distance));
            memcpy(&this->theta, &tmpdata[7], sizeof(theta));

            // 这个地方是为了把UWB的坐标移动到图像的中心位置 2023/12/13@李瑞瑞
            this->uwb_data_.x_ = cosf(theta / 180*PI)*distance + 1000;
            this->uwb_data_.y_ = sinf(theta / 180*PI)*distance + 1000;
            // 获取此时的系统时间戳
            this->uwb_data_.uwb_t_ = ros::Time::now(); 
            
            std::cout << "theta: " << theta << " distance: " << distance << std::endl;
            
        } catch (const std::exception& ex) {
            std::cerr << "[ERR]: uwb.cpp::Uart USB read data exception: " 
                << ex.what() << std::endl;
        }
    }
};



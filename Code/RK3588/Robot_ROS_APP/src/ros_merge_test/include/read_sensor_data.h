#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <boost/thread/mutex.hpp>
#include "type.h"
#include "uwb.h"


#ifndef READ_SENSOR_DATA_H
#define READ_SENSOR_DATA_H

namespace uwb_slam{
    typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
    typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
    class ReadSensorData
    {
        public:
        ReadSensorData();

        void Run(int argc, char* argv[]);
        //void set_uwb(Uwb * uwb);
        void imu_call_back(const ImuConstPtr& imu);
        void odom_call_back(const OdomConstPtr& odom);

        private:
        ros::Subscriber imu_sub_;
        ros::Subscriber odom_sub_;
        
    };
}
#endif
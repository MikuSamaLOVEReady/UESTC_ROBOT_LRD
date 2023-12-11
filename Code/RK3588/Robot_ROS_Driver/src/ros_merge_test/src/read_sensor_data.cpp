#include <read_sensor_data.h>

namespace uwb_slam {


    //void Read_sensor_data::set_uwb(){}


    void Read_sensor_data::imu_call_back(const ImuConstPtr& imu){
        Imu_data d1= Imu_data(imu->linear_acceleration.x,imu->linear_acceleration.y,imu->linear_acceleration.z,
                              imu->angular_velocity.x,imu->angular_velocity.y,imu->angular_velocity.z);

    }
    void Read_sensor_data::odom_call_back(const OdomConstPtr& odom){
        Odom_data d1 = Odom_data(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z,
                                 odom->pose.pose.orientation.w,odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,
                                 odom->header.stamp,odom->twist.twist.linear.x,odom->twist.twist.linear.y,odom->twist.twist.angular.z);

    }
    void Read_sensor_data::Run(int argc, char* argv[]){

        ros::init(argc, argv, "imu_odom");
        // 创建一个节点句柄
        ros::NodeHandle nh;
        //imu_sub_ = nh.subscribe<std_msgs::String>("imu/data_raw", 1000, this->imu_call_back);
        //odom_sub_ =nh.subscribe("odom", 1000,odom_call_back);
        ros::spin();
    }

}

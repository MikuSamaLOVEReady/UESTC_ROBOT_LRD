#include <ros/ros.h>
#include <ros/time.h>
#ifndef TYPE_H
#define TYPE_H

namespace uwb_slam{
struct Imu_data
{
    ros::Time imu_t_;
    double a_[3];
    double w_[3];
    Imu_data(){};
    Imu_data(double ax,double ay,double az, double wx, double wy, double wz)
    :a_{ax,ay,az},w_{wz,wy,wz}{};
};



struct Imu_odom_pose_data
{
    Imu_data imu_data_;
    double pose_[3];
    double quat_[4];
    double vxy_;
    double angle_v_;
    Imu_odom_pose_data(){};
    Imu_odom_pose_data( Imu_data imu_data,double x,double y,double z, double qw, double qx, double qy, double qz,double vxy, double angle_v):imu_data_(imu_data),pose_{x,y,z},quat_{qw,qx,qy,qz},vxy_(vxy),angle_v_(angle_v){};
};

/*struct Odom_data
{
    Imu_data imu_data_;
    ros::Time odom_t_;
    double pose_[3];
    double quat_[4];
    double vxy_,;
    double angle_v_;

    Odom_data(double x,double y,double z,
              double qw, double qx, double qy,double qz,
              ros::Time odom_t,double vxy,  double angle_v)
    :pose_{x,y,z},quat_{qw,qx,qy,qz},odom_t_(odom_t),vxy_(vxy),angle_v_(angle_v){};
};
*/

struct  Uwb_data
{
    float x_,y_;
    ros::Time uwb_t_;
    Uwb_data(){};
    Uwb_data(float x,float y,float t):x_(x),y_(y),uwb_t_(t){};
};

/*struct Imu_odom_pose_data
{
    Imu_data imu_data_;
    Odom_data odom_data;
    Imu_odom_pose_data(Imu_data i_data, Odom_data o_data):imu_data_(i_data),odom_data(o_data){};
};
*/
}
#endif
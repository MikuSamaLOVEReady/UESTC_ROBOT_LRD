#include "ros/ros.h"
#include "rknn_yolov5_demo/dis_info.h"
#include "rknn_yolov5_demo/dis_info_array.h"

void doMsg(const rknn_yolov5_demo::dis_info_array::ConstPtr& ceju_msg){
 
    for (const auto& obstacle_info : ceju_msg->dis)
    {
        
        float distance = obstacle_info.distance;
        float width = obstacle_info.width;
        float height = obstacle_info.height;
        ROS_INFO("distance: %.2f ", distance);

        // // 执行避障逻辑
        // if (distance < obstacle_distance_threshold)
        // {
        //     ROS_INFO("Obstacle detected at distance: %.2f meters. Avoiding obstacle.", distance);

        //     // 在这里执行避障动作，例如停止机器人
        //     geometry_msgs::Twist cmd_vel;
        //     cmd_vel.linear.x = 0.0;
        //     cmd_vel.angular.z = 0.0;
        //     cmd_vel_pub.publish(cmd_vel);

        //     // 这里可以添加更复杂的避障逻辑，例如避开障碍物或调整方向
        // }
        // else
        // {
        //     ROS_INFO("No obstacle detected at distance: %.2f meters. Continuing.", distance);
        // }
    }
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"sub_dis");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<rknn_yolov5_demo::dis_info_array>("ceju_info",10,doMsg);
  
    ros::spin();//循环读取接收的数据，并调用回调函数处理

    return 0;
}
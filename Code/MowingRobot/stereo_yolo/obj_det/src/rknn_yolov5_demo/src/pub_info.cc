#include "rknn_yolov5_demo/pub_info.h"


void Pub_info::pub_dis()
{
    Ranging ranging;
    ros::Rate loop_rate(10);

    while(ros::ok()){  
        std::vector<cv::Mat> result = ranging.get_range();
        cv::Mat info = result[2];
        for(int i=0;i<info.rows;i++)
        {
            data.distance = info.at<float>(i,0);
            data.width = info.at<float>(i,1);
            data.height = info.at<float>(i,2);
            data.angle = info.at<float>(i,3);
            dis_array_.dis.push_back(data);
        }
        dis_pub_.publish(dis_array_);
        loop_rate.sleep();

    }
 
    

}


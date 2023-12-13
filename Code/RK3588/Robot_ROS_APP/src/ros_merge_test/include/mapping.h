#include <queue>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "uwb.h"

#ifndef MAPPING_H
#define MAPPING_H

namespace uwb_slam{
    class Mapping
    {
        public:
        const double PIXEL_SCALE = 5.0;
        const int    AREA_SIZE = 2000;
        Mapping() {};
        void Run();
        bool check_uwb_point();
        void feed_uwb_data(const cv::Point2d & data);
        void process();
        std::mutex mMutexMap;
        std::shared_ptr<uwb_slam::Uwb> uwb_;

        private:
        std::queue<cv::Point2d> mv_uwb_point_;
        bool read_uwb_ = false;
        cv::Mat img;
        cv::Point2d cur_point = {-1,-1};
    };
}

#endif

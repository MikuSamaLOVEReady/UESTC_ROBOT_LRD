#ifndef _COMMON_UTILS_H_
#define _COMMON_UTILS_H_

// ros//
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
// std//
#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>
// opencv//
#include <opencv2/opencv.hpp>

#include <sys/stat.h>
#include <unistd.h>

using namespace ros;
using namespace std;
using namespace cv;

#define DEGREE_TO_RADIAN(X) X* CV_PI / 180.0f
#define RADIAN_TO_DEGREE(X) X * 180.0f / CV_PI

#define ANGLE_THRESHOLD 180.0f

#define SPARSE_PATH 1

enum LDSDirection { LDS_FRONT = 0,
                    LDS_LEFT,
                    LDS_BACK,
                    LDS_RIGHT };

enum RotateDirection {
  CLOCKWISE,  //角度减小//
  ANTICLOCKWISE
};

// Regions//
struct CleanRegion {
  CleanRegion(string name) {
    regionName = name;
    floorType = 0;
    sweepType = 0;
    regionPoints.clear();
  }

  string regionName;
  int floorType;
  int sweepType;
  vector<Point> regionPoints;
};

// utils methods//
float calcDistance(const Point& p1, const Point& p2);
float calcDistance(const Point2f& p1, const Point2f& p2);

//两个点位之间角度//
float calcDegreeAngle(const Point& startPoint, const Point& endPoint);
float calcRadianAngle(const Point& startPoint, const Point& endPoint);

float calcDegreeAngle(const Point2f& startPoint, const Point2f& endPoint);

//计算两个角度差//
float calcAngleDiff(float angle1, float angle2);

//沿着路径的点，计算角度//
float calcPathAngle(const vector<Point>& pathPoints, int index);

RotateDirection calcRotateDirection(float robot_angle, float path_angle);

#endif

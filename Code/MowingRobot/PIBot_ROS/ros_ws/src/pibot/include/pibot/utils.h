#ifndef _PIBOT_UTILS_H_
#define _PIBOT_UTILS_H_

#include <ros/ros.h>
#include <string>

// msgs
#include "pibot_msgs/PointArray.h"
#include "pibot_msgs/Pose.h"
#include "pibot_msgs/RobotState.h"

// srvs
#include "pibot_msgs/Action.h"
#include "pibot_msgs/ActionParams.h"
#include "pibot_msgs/ManualLocation.h"
#include "pibot_msgs/MappingParams.h"
#include "pibot_msgs/NaviParams.h"
#include "pibot_msgs/NaviStatus.h"
#include "pibot_msgs/RelocateResult.h"
#include "pibot_msgs/Relocation.h"
#include "pibot_msgs/SetMapParams.h"
#include "pibot_msgs/SetPoseParams.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <numeric>
#include <opencv2/opencv.hpp>

#include <jsoncpp/json/json.h>
#include <yaml-cpp/yaml.h>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

#include <fstream>
#include <iostream>

#include <common_utils/common_utils.h>

#include "pibot/msg_definition.h"

template <typename T>
void operator>>(const YAML::Node& node, T& i) {
  i = node.as<T>();
}

#endif

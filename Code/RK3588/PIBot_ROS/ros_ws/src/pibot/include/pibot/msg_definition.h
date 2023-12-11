#ifndef _MSG_DEFINITION_H_
#define _MSG_DEFINITION_H_

#include <string>

// msgs
//  地图坐标系(opencv坐标系)
//  地图左上角为原点
//  ----------------------> x
//  |
//  |
//  |
//  |
//  |
//  |
//  Y

// topic names
static const std::string LaserPointsTopic = "laser_points";  // 激光雷达的激光点(地图坐标系)
static const std::string RobotStateTopic = "robot_state";    // j运行状态 位置 速度 等
static const std::string PathPointsTopic = "path_points";    // 导航规划的路径点(地图坐标系)
static const std::string NaviStateTopic = "navi_state";      // 导航状态

// advice service
static const std::string MappingService = "mapping";                    // 开始/停止/取消 建图
static const std::string SetLocationModeService = "set_location_mode";  // 设置定位模式，定位模式下可以进行导航
static const std::string NaviService = "navi";                          // 启动/取消 导航到目的地
static const std::string SetPoseService =
    "set_pose";                                             // 设置当前定位(应用于辅助定位)
static const std::string RelocationService = "relocation";  // 重定位

static const std::string StaicMapService = "/static_map";

// call service
static const std::string AmclSetPoseTopic = "/initialpose";  // 设置当前定位(应用于辅助定位)
static const std::string CartoSetPoseService = "set_pose";   // 设置当前定位(应用于辅助定位)
static const std::string SetMapService = "/set_map";
static const std::string CartoMappingService = "/map_build_trans";

// sub topic
static const std::string MappingTopic = "/map";

static const std::string CartoPoseTopic = "/tracked_pose";
static const std::string AmclPoseTopic = "/amcl_pose";
static const std::string planTopic = "/move_base/NavfnROS/plan";  // movebase path

#endif

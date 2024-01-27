#ifndef _PIBOT_FACADE_H_
#define _PIBOT_FACADE_H_

#include <boost/thread.hpp>
#include "nav_msgs/Odometry.h"

#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "pibot/utils.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <nav_msgs/GetMap.h>
#include <std_srvs/Empty.h>
#include <mutex>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#ifndef SCAN_PC_TYPE
#define SCAN_PC_TYPE 2
#endif

class PibotManager {
  //状态基
  enum RobotState {
    RS_WAIT,
    RS_MAPPING,
    RS_LOCALIZATION,
    RS_NAVIGATION,
    RS_RELOCATION
  };

 public:
  PibotManager();
  ~PibotManager();

  void PublishMessages();

 private:
  // callback method
  bool MappingCB(pibot_msgs::MappingParams::Request& req, pibot_msgs::MappingParams::Response& res);
  bool SetLocationModeCB(pibot_msgs::ActionParams::Request& req, pibot_msgs::ActionParams::Response& res);
  bool NaviCB(pibot_msgs::NaviParams::Request& req, pibot_msgs::NaviParams::Response& res);
  bool SetPoseCB(pibot_msgs::SetPoseParams::Request& req, pibot_msgs::SetPoseParams::Response& res);
  bool RelocationCB(pibot_msgs::ActionParams::Request& req, pibot_msgs::ActionParams::Response& res);

  bool StaticMapCB(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res);

  // subscribe method
  void onReceiveCartoPose(const geometry_msgs::PoseStamped::ConstPtr msg);
  void onReceiveAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

#if SCAN_PC_TYPE == 1
  void onReceiveLDS(const sensor_msgs::PointCloud::ConstPtr msg);
#else
  void onReceiveLDS2(const sensor_msgs::PointCloud2::ConstPtr msg);
#endif

  void onReceiveMap(const nav_msgs::OccupancyGrid::ConstPtr msg);
  void onReceivePath(const nav_msgs::Path::ConstPtr msg);
  void onReceiveOdom(const nav_msgs::Odometry& msg);
  void onReceiveRelocationResult(const pibot_msgs::RelocateResult::ConstPtr& msg);

  // load map config
  int LoadConfigFile();
  int LoadMapInfo();
  int LoadYamlFile(string fileName);

  int WorldToMap(const geometry_msgs::Point& world_point, pibot_msgs::Pose& map_pose);
  int WorldToMap(const geometry_msgs::Pose& world_pose, pibot_msgs::Pose& map_pose);

  int MapToWorld(const pibot_msgs::Pose& map_pose, geometry_msgs::Pose& world_pose);

  int MapToWorld(const Point& mapPoint, Point2f& worldPoint);

  // publish the map
  int PublishMap(const Mat& image);

 private:
  // movebase action callback
  void MoveBaseActionDoneCallback(const actionlib::SimpleClientGoalState& state,
                                  const move_base_msgs::MoveBaseResultConstPtr& result);
  void MoveBaseActionActiveCallback();
  void MoveBaseActionFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  bool SendGoal(const Point& point, float angle, bool normal = true);
  bool CancelGoal();

 private:
  MoveBaseClient move_base_action_client_;
  boost::thread* pub_thread_ = nullptr;
  ros::NodeHandle nodeHandle;

  // params
  bool inner_mapping_func_;  // true 内部建图命令 false 调用子进程启动相关launch
  std::string start_mapping_cmd_;
  std::string stop_mapping_cmd_;
  std::string cancel_mapping_cmd_;

  std::string map_mapping_topic_;
  std::string map_dir_;

  std::string laser_frame_;

  bool inner_location_func_;   // true 内部定位命令 false 调用子进程启动相关launch
  std::string location_type_;  // amcl or other
  std::string odom_topic_;
  std::string scan_pcl_topic_;
  std::string start_location_cmd_;
  std::string cancel_location_cmd_;

  // suply ros service server for client
  ros::ServiceServer mapping_srv_;     // start/stop/cancel mapping
  ros::ServiceServer set_mode_srv_;    // enable/disable location mode(for navi)
  ros::ServiceServer navi_srv_;        // start/cancel navi gaols
  ros::ServiceServer set_pose_srv_;    // estimate robot position for a giving pose
  ros::ServiceServer relocation_srv_;  // auto relocation
  ros::ServiceServer static_map_srv_;

  // publisher
  ros::Publisher robot_status_pub_;  // robot status used of image coordinate;
  ros::Publisher laser_ponits_pub_;  // out pub laser points used of image coordinate
  ros::Publisher path_points_pub_;   // send out path points used of image coordinate
  // ros::Publisher goal_pub_;                   // pub goal used of image
  // coordinate;
  ros::Publisher navi_state_pub_;  // navi state

  ros::Publisher map_meta_data_pub_;
  ros::Publisher grid_map_pub_;

  ros::Publisher amcl_initialpose_pub_;

  // subscribe
  ros::Subscriber planner_path_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber lds_sub_;

  ros::Subscriber mapping_map_sub_;

  ros::Subscriber relocation_result_sub_;

  // tf
  tf::TransformListener laser_listener_;
  tf::TransformListener base_listener_;

 private:
  // map coordinates
  pibot_msgs::Pose robot_pose_in_img_;

  // world coordinates
  Point2f origin_world_point_;
  float origin_world_angle_;

  nav_msgs::OccupancyGrid occupancy_grid_map_msg_;
  std::mutex map_mutex_;

  // active map info
  int map_img_width_;
  int map_img_height_;
  float map_resolution_;
  string map_name_;
  string pb_name_;
  string map_config_file_;
  int negate_;
  double occ_th_, free_th_;
  vector<vector<Point>> _virtual_wall_list_;

  RobotState now_robot_state_;  // robot state

  bool relocation_result_ = false;  // last relocation result

  double odom_vx_, odom_vy_, odom_w_;
  ros::Time last_pub_scan_time_;
};

#endif

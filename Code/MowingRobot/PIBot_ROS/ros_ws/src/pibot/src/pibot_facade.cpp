#include "pibot/pibot_facade.h"

#define STATE_PUB_RATE 100
#define LDS_PUB_INTERVAL 1000

PibotManager::PibotManager()
    : move_base_action_client_("move_base", true),
      odom_vx_(0.0f),
      odom_vy_(0.0f),
      odom_w_(0.0f) {
  ros::NodeHandle private_nh("~");
  // params
  private_nh.param("location_type", location_type_, std::string("amcl"));
  private_nh.param("odom_topic", odom_topic_, std::string("/odom"));
  private_nh.param("scan_pcl_topic", scan_pcl_topic_, std::string("/scan_cloud"));
  private_nh.param("mapping_topic", map_mapping_topic_, MappingTopic);

  private_nh.param("laser_frame", laser_frame_, std::string("/laser_link"));

  char* dir = getenv("PIBOT_MAPS_DIR");
  if (dir == NULL) {
    map_dir_ = "~/maps";
  } else {
    map_dir_ = dir;
  }

  // mapping cmd
  private_nh.param("inner_mapping_func", inner_mapping_func_, false);
  private_nh.param("start_mapping_cmd", start_mapping_cmd_,
                   std::string("roslaunch pibot start_mapping.launch"));
  start_mapping_cmd_ += "&";
  private_nh.param("stop_mapping_cmd", stop_mapping_cmd_, std::string("roslaunch pibot stop_mapping.launch"));
  private_nh.param("cancel_mapping_cmd", cancel_mapping_cmd_, std::string("rosnode kill slam_gmapping"));

  // location cmd
  private_nh.param("inner_location_func", inner_location_func_, false);
  private_nh.param("start_location_cmd", start_location_cmd_, std::string("roslaunch pibot start_location.launch"));
  start_location_cmd_ += "&";
  private_nh.param("cancel_location_cmd", cancel_location_cmd_, std::string("rosnode kill amcl"));

  ROS_INFO("mapping func: %d, location func : %d", inner_mapping_func_, inner_location_func_);

  // advertise service
  mapping_srv_ = private_nh.advertiseService(MappingService, &PibotManager::MappingCB, this);
  set_mode_srv_ = private_nh.advertiseService(SetLocationModeService, &PibotManager::SetLocationModeCB, this);
  navi_srv_ = private_nh.advertiseService(NaviService, &PibotManager::NaviCB, this);
  set_pose_srv_ = private_nh.advertiseService(SetPoseService, &PibotManager::SetPoseCB, this);
  relocation_srv_ = private_nh.advertiseService(RelocationService, &PibotManager::RelocationCB, this);

  // pub
  robot_status_pub_ = private_nh.advertise<pibot_msgs::RobotState>(RobotStateTopic.c_str(), 1);   // robot status used of image coordinate;
  laser_ponits_pub_ = private_nh.advertise<pibot_msgs::PointArray>(LaserPointsTopic.c_str(), 1);  // out pub laser points used of image coordinate;
  last_pub_scan_time_ = ros::Time::now();
  path_points_pub_ = private_nh.advertise<pibot_msgs::PointArray>(PathPointsTopic.c_str(), 1);  // send out path points
  navi_state_pub_ = private_nh.advertise<pibot_msgs::NaviStatus>(NaviStateTopic.c_str(), 1);

  mapping_map_sub_ = nodeHandle.subscribe(map_mapping_topic_, 100, &PibotManager::onReceiveMap, this);
  if (inner_location_func_) {
    robot_pose_sub_ = nodeHandle.subscribe(CartoPoseTopic, 100, &PibotManager::onReceiveCartoPose,
                                           this);  // pose from carto
  } else {
    if (location_type_ == "amcl") {
      robot_pose_sub_ = nodeHandle.subscribe(AmclPoseTopic, 100, &PibotManager::onReceiveAmclPose, this);           // pose from amcl
      amcl_initialpose_pub_ = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>(AmclSetPoseTopic, 1);  // set pose by amcl
      static_map_srv_ = nodeHandle.advertiseService(StaicMapService, &PibotManager::StaticMapCB, this);
    } else {
      ROS_WARN("unknown location type");
    }
  }

  odom_sub_ = nodeHandle.subscribe(odom_topic_, 10, &PibotManager::onReceiveOdom, this);  // odom

#if SCAN_PC_TYPE == 1
  lds_sub_ = nodeHandle.subscribe(scan_pcl_topic_, 100, &PibotManager::onReceiveLDS, this);  // laser point cloud
#else
  lds_sub_ = nodeHandle.subscribe(scan_pcl_topic_, 100, &PibotManager::onReceiveLDS2, this);  // laser point cloud
#endif

  planner_path_sub_ = nodeHandle.subscribe(planTopic, 100, &PibotManager::onReceivePath, this);  // path from planner

  relocation_result_sub_ = nodeHandle.subscribe("/relocation_result", 100, &PibotManager::onReceiveRelocationResult, this);

  // wait for service
  ROS_INFO("waitForTransform...");
  laser_listener_.waitForTransform("map", laser_frame_, ros::Time(0), ros::Duration(1.0));

  map_name_ = "";

  now_robot_state_ = RS_WAIT;
  pub_thread_ = new boost::thread(boost::bind(&PibotManager::PublishMessages, this));

  ROS_INFO("Pibot Manager Inited!");
}

PibotManager::~PibotManager() {
  pub_thread_->interrupt();
  pub_thread_->join();
  delete pub_thread_;
}

void PibotManager::onReceiveOdom(const nav_msgs::Odometry& msg) {
  odom_vx_ = msg.twist.twist.linear.x;
  odom_vy_ = msg.twist.twist.linear.y;
  odom_w_ = msg.twist.twist.angular.z;

  if (!inner_mapping_func_ && (now_robot_state_ == RS_MAPPING)) {
    tf::StampedTransform tf_transform;

    tf::Vector3 origin;
    tf::Quaternion rotation;
    tf::Vector3 axis;
    int seq = 0;

    try {
      base_listener_.lookupTransform("map", "base_link", ros::Time(0), tf_transform);
    } catch (tf::TransformException& exception) {
      ROS_ERROR("%s", exception.what());
    }

    geometry_msgs::Pose robot_map_pose;

    robot_map_pose.position.x = tf_transform.getOrigin().getX();
    robot_map_pose.position.y = tf_transform.getOrigin().getY();
    robot_map_pose.position.z = tf_transform.getOrigin().getZ();
    robot_map_pose.orientation.x = tf_transform.getRotation().getX();
    robot_map_pose.orientation.y = tf_transform.getRotation().getY();
    robot_map_pose.orientation.z = tf_transform.getRotation().getZ();
    robot_map_pose.orientation.w = tf_transform.getRotation().getW();

    if (WorldToMap(robot_map_pose, robot_pose_in_img_) == 0) {
      // ROS_INFO("===%0.3f %0.3f %0.3f", robot_pose_in_img_.x,
      // robot_pose_in_img_.y, robot_pose_in_img_.theta);
    }
  }
}

void PibotManager::PublishMessages() {
  ros::Rate rate(STATE_PUB_RATE);
  ros::NodeHandle nh;
  while (nh.ok()) {
    {
      pibot_msgs::RobotState robot_state_msg;
      robot_state_msg.vlinear_x = odom_vx_;
      robot_state_msg.vlinear_y = odom_vy_;
      robot_state_msg.vangular = odom_w_;
      if (map_img_height_ != 0 && map_img_width_ != 0) {
        robot_state_msg.pose.x = robot_pose_in_img_.x;
        robot_state_msg.pose.y = robot_pose_in_img_.y;
        robot_state_msg.pose.theta = robot_pose_in_img_.theta;
      } else {
        robot_state_msg.pose.x = 0;
        robot_state_msg.pose.y = 0;
        robot_state_msg.pose.theta = 0;
      }

      if (now_robot_state_ == RS_WAIT) {
        robot_state_msg.state = 0;
      } else if (now_robot_state_ == RS_MAPPING) {
        robot_state_msg.state = 1;
      } else if (now_robot_state_ == RS_LOCALIZATION) {
        robot_state_msg.state = 2;
      } else {
        robot_state_msg.state = 3;
      }

      if (now_robot_state_ == RS_LOCALIZATION) {
        // TODO relocation
      }

      robot_status_pub_.publish(robot_state_msg);
    }
    rate.sleep();
  }
}

int PibotManager::LoadMapInfo() {
  ifstream ifs(map_config_file_);
  if (!ifs) {
    ROS_ERROR("NO JSON FILE, ErrorCode=%d", -1);
    return -1;
  }

  Json::Value jsonroot;
  Json::Reader jsonreader;

  if (!jsonreader.parse(ifs, jsonroot)) {
    ROS_ERROR("FAIL TO LOAD JSON, ErrorCode=%d", -2);
    return -2;
  }

  // virtual wall
  _virtual_wall_list_.clear();
  Json::Value vwalls = jsonroot["vwalls"];

  for (int i = 0; i < vwalls.size(); i++) {
    Json::Value points = vwalls[i]["points"];
    vector<Point> vwallPoints;

    for (int j = 0; j < points.size(); j++) {
      vwallPoints.push_back(Point(points[j]["x"].asInt(), points[j]["y"].asInt()));
    }
    _virtual_wall_list_.push_back(vwallPoints);
  }

  return 0;
}

int PibotManager::LoadConfigFile() {
  string config_path = map_dir_ + "/config.json";

  cout << config_path << endl;
  ifstream ifs(config_path.c_str());

  if (!ifs) {
    ROS_ERROR("%s not found", config_path.c_str());
    return -1;
  }

  Json::Value jsonroot;
  Json::Reader jsonreader;

  if (!jsonreader.parse(ifs, jsonroot)) {
    ROS_ERROR("fail to load %s", config_path.c_str());
    return -2;
  }

  string mapID = jsonroot["default_map"].asString();
  string yaml_file = map_dir_ + "/" + mapID + "/map.yaml";

  int state = LoadYamlFile(yaml_file);
  if (state != 0) {
    return state;
  }

  pb_name_ = map_dir_ + "/" + mapID + "/map.pbstream";

  map_config_file_ = map_dir_ + "/" + mapID + "/map.json";

  Mat mMatLoad = imread(map_name_);
  if (mMatLoad.empty()) {
    ROS_ERROR("fail to read image %s", map_name_.c_str());
    return -4;
  }

  LoadMapInfo();

  map_img_width_ = mMatLoad.cols;
  map_img_height_ = mMatLoad.rows;

  PublishMap(mMatLoad);

  return 0;
}

int PibotManager::LoadYamlFile(string path) {
  ifstream ifs2(path.c_str());
  if (!ifs2) {
    ROS_ERROR("%s not found", path.c_str());
    return -3;
  }

  YAML::Node doc = YAML::Load(ifs2);

  doc["image"] >> map_name_;

  doc["origin"][0] >> origin_world_point_.x;
  doc["origin"][1] >> origin_world_point_.y;
  doc["origin"][2] >> origin_world_angle_;

  doc["resolution"] >> map_resolution_;

  // other infomation
  doc["negate"] >> negate_;
  doc["occupied_thresh"] >> occ_th_;
  doc["free_thresh"] >> free_th_;

  return 0;
}

bool PibotManager::MappingCB(pibot_msgs::MappingParams::Request& req, pibot_msgs::MappingParams::Response& res) {
  bool valid = false;

  // 状态判断
  if (req.action.type == pibot_msgs::Action::ACTION_START) {  // start mapping
    ROS_INFO("start mapping");
    if (now_robot_state_ != RS_WAIT) {
      res.result = 1;
      ROS_INFO("NOT AVAILABLE,CANT START MAPPING, ErrorCode=%d", res.result);
      return true;
    }
    valid = true;
  } else {
    ROS_INFO("finish mapping");
    if (now_robot_state_ == RS_MAPPING) {
      valid = true;
    }
  }

  if (!valid) {
    res.result = 2;
    ROS_WARN("nowstate is wrong");
    return true;
  }

  if (inner_mapping_func_) {  // 内部service client 启动相关建图应用
    ros::ServiceClient client_map_build = nodeHandle.serviceClient<pibot_msgs::MappingParams>(CartoMappingService);

    pibot_msgs::MappingParams map_build_request;
    map_build_request.request = req;

    client_map_build.call(map_build_request);

    res = map_build_request.response;

    if (res.result == 0) {
      if (now_robot_state_ == RS_WAIT) {
        now_robot_state_ = RS_MAPPING;
      } else if (now_robot_state_ == RS_MAPPING) {
        now_robot_state_ = RS_WAIT;
      }
      res.result = 0;
    } else {
      res.result = 1;
      ROS_ERROR("mapbuild call failed: %d", map_build_request.response.result);
    }
  } else {  // 使用子进程的方式加载建图应用
    res.result = 0;
    if (req.action.type == pibot_msgs::Action::ACTION_START) {
      if (!start_mapping_cmd_.empty()) {
        ROS_INFO("launch map cmd: %s", start_mapping_cmd_.c_str());
        FILE* fp = popen(start_mapping_cmd_.c_str(), "r");

        fclose(fp);

        ROS_WARN("wait for tf of map to base_link");
        base_listener_.waitForTransform("map", "base_link", ros::Time(0),
                                        ros::Duration(5.0));
      } else {
        ROS_WARN("null start_mapping_cmd");
        res.result = -1;
      }
    } else {
      if (req.action.type == pibot_msgs::Action::ACTION_STOP) {
        std::string map_dir = map_dir_;
        map_dir += "/" + req.id;

        if (access(map_dir.c_str(), F_OK) == -1) {
          if (mkdir(map_dir.c_str(), 0777) != 0) {
            ROS_WARN("create failed");
            return true;
          }
        }

        std::string save_mapping_cmd = stop_mapping_cmd_ + " map_name:=" + map_dir + "/map";

        ROS_INFO("save_map cmd: %s", save_mapping_cmd.c_str());
        FILE* fp = popen(save_mapping_cmd.c_str(), "r");

        fclose(fp);
      }

      if (!cancel_mapping_cmd_.empty()) {
        ROS_INFO("kill mappping cmd: %s", cancel_mapping_cmd_.c_str());
        FILE* fp = popen(cancel_mapping_cmd_.c_str(), "r");
        fclose(fp);
      } else {
        ROS_WARN("null cancel_mapping_cmd");
        res.result = -1;
      }
    }

    if (res.result == 0) {
      if (now_robot_state_ == RS_WAIT) {
        now_robot_state_ = RS_MAPPING;
      } else if (now_robot_state_ == RS_MAPPING) {
        now_robot_state_ = RS_WAIT;
      }
    }
  }

  return true;
}

bool PibotManager::SetLocationModeCB(pibot_msgs::ActionParams::Request& req, pibot_msgs::ActionParams::Response& res) {
  if (req.action.type == pibot_msgs::Action::ACTION_START) {  // set to localization mode for navi
    if (now_robot_state_ != RS_WAIT) {
      ROS_ERROR("NOT AVAILABLE,CANT START LOCALIZATION,  ErrorCode is %d",
                res.result);
      res.result = 1;
      return true;
    }

    int result = LoadConfigFile();
    if (0 != result) {
      res.result = 2;
      ROS_ERROR("load Default JsonFile is failed, ErrorCode is %d", res.result);
      return true;
    }

    if (inner_location_func_) {  // 内部service client 启动相关建图应用
      ros::ServiceClient client_set_map = nodeHandle.serviceClient<pibot_msgs::SetMapParams>(SetMapService);

      pibot_msgs::SetMapParams set_map_params;
      set_map_params.request.id = pb_name_;

      client_set_map.call(set_map_params);

      if (set_map_params.response.result != 0) {
        res.result = 3;
        ROS_ERROR("set_map request is fail, ErrorCode is %d", res.result);
        return true;
      }
    } else {
      if (location_type_ == "amcl") {
        ROS_INFO("launch location cmd: %s", start_location_cmd_.c_str());
        FILE* fp = popen(start_location_cmd_.c_str(), "r");

        fclose(fp);
      }

      // wait for move_base
      while (!move_base_action_client_.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for move_base action server startup");
      }
    }

    // 启动重定位
    if (inner_location_func_) {  // 内部service client 启动相关建图应用
      //   ROS_INFO("start_location(inner)");
      //   now_robot_state_ = RS_RELOCATION;
      //   ros::ServiceClient start_location_client = nodeHandle.serviceClient<pibot_msgs::start_location>("start_location");

      //   pibot_msgs::start_location start_location_req;
      //   start_location_req.request.filename = pb_name_;
      //   start_location_req.request.action_mode = 0;
      //   start_location_req.request.to_trajectory_id = 0;

      //   start_location_client.call(start_location_req);

      //   now_robot_state_ = RS_LOCALIZATION;

      //   if (relocation_result_ != 0) {
      //     ROS_ERROR("start_location Service return fail, ErrorCode is %d",
      //               res.result);
      //     res.result = start_location_req.response.result;
      //     return true;
      //   }
    } else {
      if (location_type_ == "amcl") {
        ROS_INFO("start_location for amcl");
        ros::ServiceClient global_localization_client = nodeHandle.serviceClient<std_srvs::Empty>("global_localization");
        std_srvs::Empty params;
        if (global_localization_client.call(params)) {
        }
      }

      now_robot_state_ = RS_LOCALIZATION;
    }
  } else {  // set free mode
    if (now_robot_state_ == RS_LOCALIZATION) {
      map_meta_data_pub_.shutdown();
      grid_map_pub_.shutdown();

      if (inner_location_func_) {  // 内部service client 启动相关建图应用
        // ROS_INFO("start_finish_location");
        // ros::ServiceClient start_location_client = nodeHandle.serviceClient<pibot_msgs::start_location>("start_location");

        // pibot_msgs::start_location start_location_req;
        // start_location_req.request.filename = pb_name_;
        // start_location_req.request.action_mode = 1;
        // start_location_req.request.to_trajectory_id = 0;

        // start_location_client.call(start_location_req);
        // res.result = start_location_req.response.result;

        // if (res.result == 0) {
        //   now_robot_state_ = RS_WAIT;
        // }
      } else {
        if (location_type_ == "amcl") {
          std::unique_lock<std::mutex> lock(map_mutex_);
          occupancy_grid_map_msg_.data.clear();
        }

        ROS_INFO("cancel location cmd: %s", cancel_location_cmd_.c_str());
        FILE* fp = popen(cancel_location_cmd_.c_str(), "r");

        fclose(fp);
        now_robot_state_ = RS_WAIT;
      }
    } else {
      ROS_INFO("now_robot_state_ is already [WAIT],res.result = %d", 2);
      res.result = 2;
      return true;
    }
  }

  res.result = 0;

  return true;
}

bool PibotManager::NaviCB(pibot_msgs::NaviParams::Request& req, pibot_msgs::NaviParams::Response& res) {
  ROS_INFO("Navi, action: %d, goal: [%f , %f, %f]", req.action.type, req.goal.x,
           req.goal.y, req.goal.theta);

  if (now_robot_state_ != RS_LOCALIZATION) {
    ROS_INFO("NO LOCALIZATION INFO,CANT NAVIGATION");
    res.result = 1;
    return true;
  }

  if (map_img_width_ == 0 || map_img_height_ == 0) {
    ROS_ERROR("size of image map is exception");
    res.result = 2;
    return true;
  }

  if (req.action.type == pibot_msgs::Action::ACTION_START) {
    ros::ServiceClient clear_costmaps_client = nodeHandle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    std_srvs::Empty params;
    if (clear_costmaps_client.call(params)) {
    }

    bool ret = SendGoal(Point(req.goal.x, req.goal.y), req.goal.theta);
    if (!ret) {
      res.result = 3;
      return true;
    }
  } else {
    CancelGoal();
  }

  res.result = 0;
  return true;
}

bool PibotManager::SetPoseCB(pibot_msgs::SetPoseParams::Request& req, pibot_msgs::SetPoseParams::Response& res) {
  ROS_INFO("Set Pose CB: %f, %f, %f", req.pose.x, req.pose.y, req.pose.theta);

  if (map_img_width_ == 0 || map_img_height_ == 0) {
    ROS_ERROR("size of image map is exception");
    res.result = 2;
    return true;
  }

  geometry_msgs::PoseWithCovarianceStamped estimate_pose_msg;
  estimate_pose_msg.header.stamp = ros::Time::now();
  estimate_pose_msg.header.frame_id = "map";

  if (MapToWorld(req.pose, estimate_pose_msg.pose.pose) != 0) {
    ROS_WARN("err req pose");
    res.result = 3;
    return true;
  }

  if (inner_location_func_) {
    ros::service::waitForService("manual_location_trans");
    ros::ServiceClient manual_relocation_client = nodeHandle.serviceClient<::pibot_msgs::ManualLocation>(
        "manual_location_trans");

    pibot_msgs::ManualLocation relocation_req;
    relocation_req.request.pose = estimate_pose_msg.pose.pose;
    relocation_req.request.filename = pb_name_;
    manual_relocation_client.call(relocation_req);

    if (relocation_req.response.result != 0) {
      ROS_ERROR("response of manual manualReLocationCallback is fail!");
      res.result = 1;
      return true;
    }
    ROS_INFO("Service [manual_location_trans] return");
    res.result = 0;
  } else {
    if (location_type_ == "amcl") {
      amcl_initialpose_pub_.publish(estimate_pose_msg);
    }
  }

  return true;
}

bool PibotManager::RelocationCB(pibot_msgs::ActionParams::Request& req, pibot_msgs::ActionParams::Response& res) {
  ROS_INFO("Auto Location");

  if (map_img_width_ == 0 || map_img_height_ == 0) {
    ROS_ERROR("size of image map is exception");
    res.result = 2;
    return true;
  }

  if (location_type_ == "amcl") {
    ros::ServiceClient global_localization_client = nodeHandle.serviceClient<std_srvs::Empty>("global_localization");
    std_srvs::Empty params;
    if (global_localization_client.call(params)) {
    }
  } else {
  }

  return true;
}

void PibotManager::onReceiveRelocationResult(const pibot_msgs::RelocateResult::ConstPtr& msg) {
  relocation_result_ = msg->relocate_result;

  ROS_ERROR("onReceiveRelocationResult: %d", (int)msg->relocate_result);
}

void PibotManager::onReceiveCartoPose(const geometry_msgs::PoseStamped::ConstPtr msg) {
  WorldToMap(msg->pose, robot_pose_in_img_);
}

void PibotManager::onReceiveAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {
  WorldToMap(msg->pose.pose, robot_pose_in_img_);
}

#if SCAN_PC_TYPE == 1
void PibotManager::onReceiveLDS(const sensor_msgs::PointCloud::ConstPtr msg) {
  if (map_img_width_ == 0 || map_img_height_ == 0) {
    return;
  }

  // upload freq
  ros::Time current_scan_time = ros::Time::now();
  if ((current_scan_time - last_pub_scan_time_).toSec() * 1000 < LDS_PUB_INTERVAL) {
    return;
  }

  last_pub_scan_time_ = ros::Time::now();

  tf::StampedTransform transform;

  try {
    laser_listener_.lookupTransform("map", laser_frame_, ros::Time(0), transform);

    pibot_msgs::PointArray laser_points_msg;
    laser_points_msg.header.stamp = ros::Time::now();

    if (0 != msg->points.size()) {
      laser_points_msg.points.resize(msg->points.size());
    }

    for (int i = 0; i < msg->points.size(); i++) {
      geometry_msgs::PointStamped local_point;
      local_point.header.frame_id = laser_frame_;
      local_point.point.x = msg->points[i].x;
      local_point.point.y = msg->points[i].y;
      local_point.point.z = msg->points[i].z;

      geometry_msgs::PointStamped world_point;
      world_point.header.frame_id = "map";
      laser_listener_.transformPoint("map", local_point, world_point);  // laser link转换为world坐标

      pibot_msgs::Pose laser_pose;
      WorldToMap(world_point.point, laser_pose);  // 转换得到地图像素坐标

      laser_points_msg.points[i].x = laser_pose.x;
      laser_points_msg.points[i].y = laser_pose.y;
    }

    if (0 != msg->points.size()) {
      laser_ponits_pub_.publish(laser_points_msg);
    }
  } catch (tf::TransformException& ex) {
  }
}
#else
void PibotManager::onReceiveLDS2(const sensor_msgs::PointCloud2::ConstPtr msg) {
  if (map_img_width_ == 0 || map_img_height_ == 0) {
    return;
  }

  // upload freq
  ros::Time current_scan_time = ros::Time::now();
  if ((current_scan_time - last_pub_scan_time_).toSec() * 1000 < LDS_PUB_INTERVAL) {
    return;
  }

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

  last_pub_scan_time_ = ros::Time::now();

  tf::StampedTransform transform;

  try {
    laser_listener_.lookupTransform("map", laser_frame_, ros::Time(0), transform);

    pibot_msgs::PointArray laser_points_msg;
    laser_points_msg.header.stamp = ros::Time::now();

    if (0 != temp_cloud->size()) {
      laser_points_msg.points.resize(temp_cloud->size());
    }

    for (int i = 0; i < temp_cloud->size(); i++) {
      geometry_msgs::PointStamped local_point;
      local_point.header.frame_id = laser_frame_;
      local_point.point.x = (*temp_cloud)[i].x;
      local_point.point.y = (*temp_cloud)[i].y;
      local_point.point.z = (*temp_cloud)[i].z;

      geometry_msgs::PointStamped world_point;
      world_point.header.frame_id = "map";
      laser_listener_.transformPoint("map", local_point, world_point);  // laser link转换为world坐标

      pibot_msgs::Pose laser_pose;
      WorldToMap(world_point.point, laser_pose);  // 转换得到地图像素坐标

      laser_points_msg.points[i].x = laser_pose.x;
      laser_points_msg.points[i].y = laser_pose.y;
    }

    if (0 != temp_cloud->size()) {
      laser_ponits_pub_.publish(laser_points_msg);
    }
  } catch (tf::TransformException& ex) {
  }
}
#endif

void PibotManager::onReceiveMap(const nav_msgs::OccupancyGrid::ConstPtr msg) {
  if (now_robot_state_ != RS_MAPPING) {
    return;
  }

  map_img_width_ = msg->info.width;
  map_img_height_ = msg->info.height;
  map_resolution_ = msg->info.resolution;

  origin_world_point_ = Point2f(msg->info.origin.position.x, msg->info.origin.position.y);
}

void PibotManager::onReceivePath(const nav_msgs::Path::ConstPtr msg) {
  if (map_img_width_ == 0 || map_img_height_ == 0) {
    return;
  }

  pibot_msgs::PointArray path_msg;
  path_msg.header.stamp = ros::Time::now();
  int num = msg->poses.size();

  if (num > 0) {
    path_msg.points.resize(num);
    for (int i = 0; i < num; i++) {
      pibot_msgs::Pose path_pose;
      WorldToMap(msg->poses[i].pose.position, path_pose);

      path_msg.points[i].x = path_pose.x;
      path_msg.points[i].y = path_pose.y;
    }
  }
  path_points_pub_.publish(path_msg);
}

int PibotManager::WorldToMap(const geometry_msgs::Point& world_point, pibot_msgs::Pose& map_pose) {
  if (map_resolution_ == 0.0f || map_img_width_ == 0 || map_img_height_ == 0 ||
      origin_world_point_ == Point2f()) {
    return -1;
  }

  map_pose.x = (world_point.x - origin_world_point_.x) / map_resolution_;
  map_pose.y = map_img_height_ - 1 - (world_point.y - origin_world_point_.y) / map_resolution_;

  if (map_pose.x < 0 || map_pose.x > map_img_width_ - 1 || map_pose.y < 0 ||
      map_pose.y > map_img_height_ - 1) {
    return -2;
  }

  return 0;
}

int PibotManager::WorldToMap(const geometry_msgs::Pose& world_pose, pibot_msgs::Pose& map_pose) {
  if (map_resolution_ == 0.0f || map_img_width_ == 0 || map_img_height_ == 0 ||
      origin_world_point_ == Point2f()) {
    return -1;
  }

  map_pose.x = (world_pose.position.x - origin_world_point_.x) / map_resolution_;
  map_pose.y = map_img_height_ - 1 - (world_pose.position.y - origin_world_point_.y) / map_resolution_;
  map_pose.theta = RADIAN_TO_DEGREE(-tf::getYaw(world_pose.orientation));

  if (map_pose.x < 0 || map_pose.x > map_img_width_ - 1 || map_pose.y < 0 ||
      map_pose.y > map_img_height_ - 1) {
    return -2;
  }

  return 0;
}

int PibotManager::MapToWorld(const pibot_msgs::Pose& map_pose, geometry_msgs::Pose& world_pose) {
  if (map_resolution_ == 0.0f || map_img_width_ == 0 || map_img_height_ == 0 ||
      origin_world_point_ == Point2f()) {
    return -1;
  }

  if (map_pose.x < 0 || map_pose.x > map_img_width_ - 1 || map_pose.y < 0 ||
      map_pose.y > map_img_height_ - 1) {
    return -2;
  }

  world_pose.position.x = map_pose.x * map_resolution_ + origin_world_point_.x;
  world_pose.position.y = (map_img_height_ - 1 - map_pose.y) * map_resolution_ + origin_world_point_.y;
  world_pose.orientation = tf::createQuaternionMsgFromYaw(DEGREE_TO_RADIAN(-map_pose.theta));

  return 0;
}

int PibotManager::MapToWorld(const Point& mapPoint, Point2f& worldPoint) {
  if (map_resolution_ == 0.0f || map_img_width_ == 0 || map_img_height_ == 0 || origin_world_point_ == Point2f()) {
    return -1;
  }

  if (mapPoint.x < 0 || mapPoint.x > map_img_width_ - 1 || mapPoint.y < 0 || mapPoint.y > map_img_height_ - 1) {
    return -2;
  }

  worldPoint.x = mapPoint.x * map_resolution_ + origin_world_point_.x;
  worldPoint.y = mapPoint.y * map_resolution_ + origin_world_point_.y;

  return 0;
}

int PibotManager::PublishMap(const Mat& image) {
  if (image.empty()) {
    return -1;
  }

  Mat mMat = image.clone();
  Mat mMatClone = imread(map_name_, CV_8UC1);
  // add map infomation//
  for (int i = 0; i < _virtual_wall_list_.size(); i++) {
    // dont link the last and first point
    for (int j = 0; j < _virtual_wall_list_[i].size() - 1; j++) {
      int start = j;
      int end = j + 1;  // getIndex(j + 1,_virtual_wall_list_[i].size());

      line(mMat, Point(_virtual_wall_list_[i][start]),
           Point(_virtual_wall_list_[i][end]), Scalar(0, 0, 0), 1, 8, 0);
      line(mMatClone, Point(_virtual_wall_list_[i][start]),
           Point(_virtual_wall_list_[i][end]), Scalar(0, 0, 0), 1, 8, 0);
    }
  }

  std::unique_lock<std::mutex> lock(map_mutex_);

  // Copy the image data into the map structure
  occupancy_grid_map_msg_.info.width = mMat.cols;
  occupancy_grid_map_msg_.info.height = mMat.rows;
  occupancy_grid_map_msg_.info.resolution = map_resolution_;
  occupancy_grid_map_msg_.info.origin.position.x = origin_world_point_.x;
  occupancy_grid_map_msg_.info.origin.position.y = origin_world_point_.y;
  occupancy_grid_map_msg_.info.origin.position.z = 0.0;
  occupancy_grid_map_msg_.info.origin.orientation = tf::createQuaternionMsgFromYaw(origin_world_angle_);
  occupancy_grid_map_msg_.data.resize(occupancy_grid_map_msg_.info.width * occupancy_grid_map_msg_.info.height);

  for (int row = 0; row < mMat.rows; row++) {
    for (int col = 0; col < mMat.cols; col++) {
      double color = mMat.at<Vec3b>(row, col)[0];
      if (negate_) {
        color = 255 - color;
      }

      double occ = (255 - color) / 255.0;
      unsigned char value;

      if (occ > occ_th_) {
        value = 100;  // black
      } else if (occ < free_th_) {
        value = 0;  // white
      } else {
        value = -1;  // gray
      }

      int index = col + (mMat.rows - 1 - row) * mMat.cols;
      occupancy_grid_map_msg_.data[index] = value;
    }
  }

  occupancy_grid_map_msg_.info.map_load_time = ros::Time::now();
  occupancy_grid_map_msg_.header.frame_id = "map";
  occupancy_grid_map_msg_.header.stamp = ros::Time::now();

  map_mutex_.unlock();

  nav_msgs::MapMetaData map_meta_data_msg = occupancy_grid_map_msg_.info;

  map_meta_data_pub_ = nodeHandle.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  grid_map_pub_ = nodeHandle.advertise<nav_msgs::OccupancyGrid>("map", 1, true);

  map_meta_data_pub_.publish(map_meta_data_msg);
  grid_map_pub_.publish(occupancy_grid_map_msg_);

  return 0;
}

bool PibotManager::StaticMapCB(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res) {
  std::unique_lock<std::mutex> lock(map_mutex_);

  if (occupancy_grid_map_msg_.data.empty()) {
    return false;
  }

  res.map = occupancy_grid_map_msg_;

  return true;
}

void PibotManager::MoveBaseActionDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("Goal Plan Complete, state: %d-%s", state.state_, state.toString().c_str());
  pibot_msgs::NaviStatus navi_msg;
  navi_msg.status = state.state_;
  navi_state_pub_.publish(navi_msg);
}

void PibotManager::MoveBaseActionActiveCallback() {
  ROS_INFO("Planning Goal Active");
}

void PibotManager::MoveBaseActionFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  if (feedback->base_position.header.frame_id == "map") {
    pibot_msgs::Pose map_pose;
    WorldToMap(feedback->base_position.pose, map_pose);

    // output pose in map
    ROS_INFO("MoveBase Feedback [%.2f %.2f %.2f]", map_pose.x, map_pose.y, map_pose.theta);
  }
}

bool PibotManager::SendGoal(const Point& point, float angle, bool normal) {
  Point2f worldPoint;

  Point mapPoint = Point(point.x, map_img_height_ - 1 - point.y);
  int result = MapToWorld(mapPoint, worldPoint);
  if (0 != result) {
    return false;
  }

#if 0
    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = worldPoint.x;
    goal.pose.position.y = worldPoint.y;
    goal.pose.position.z = normal ? 0 : -1;
    goal.pose.orientation = tf::createQuaternionMsgFromYaw(-DEGREE_TO_RADIAN(angle));
    goal_pub_.publish(goal);
#else
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = worldPoint.x;
  goal.target_pose.pose.position.y = worldPoint.y;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-DEGREE_TO_RADIAN(angle));

  // move_base_action_client_.sendGoal(goal);
  move_base_action_client_.sendGoal(goal,
                                    boost::bind(&PibotManager::MoveBaseActionDoneCallback, this, _1, _2),
                                    boost::bind(&PibotManager::MoveBaseActionActiveCallback, this),
                                    boost::bind(&PibotManager::MoveBaseActionFeedbackCallback, this, _1));
#endif

  return true;
}

bool PibotManager::CancelGoal() {
  move_base_action_client_.cancelAllGoals();
  return true;
}

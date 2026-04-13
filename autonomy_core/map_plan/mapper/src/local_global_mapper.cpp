#include "mapper/local_global_mapper.h"

LocalGlobalMapperNode::LocalGlobalMapperNode()
    : rclcpp::Node("local_global_mapper") {
  initParams();

  cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_name_,
      1,
      std::bind(
          &LocalGlobalMapperNode::cloudCallback, this, std::placeholders::_1));

  global_map_pub = this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
      "global_voxel_map", rclcpp::QoS(1).transient_local());
  storage_map_pub = this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
      "storage_voxel_map", rclcpp::QoS(1).transient_local());
  local_map_pub = this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
      "local_voxel_map", rclcpp::QoS(1).transient_local());
  local_map_no_inflation_pub =
      this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
          "local_voxel_no_inflation_map", rclcpp::QoS(1).transient_local());

  time_pub = this->create_publisher<sensor_msgs::msg::Temperature>(
      "/timing/mapper", 1);

  // storage map should have same resolution and z_dim as local map
  storage_map_info_.resolution = local_map_info_.resolution;
  local_map_info_.dim.z = static_cast<int>(
      ceil((local_map_dim_d_z_) / storage_map_info_.resolution));
  storage_map_info_.dim.z = local_map_info_.dim.z;

  storage_map_info_.origin.x = global_map_info_.origin.x;
  storage_map_info_.origin.y = global_map_info_.origin.y;
  storage_map_info_.origin.z = global_map_info_.origin.z;
  storage_map_info_.dim.x = static_cast<int>(
      ceil((global_map_dim_d_x_) / storage_map_info_.resolution));
  storage_map_info_.dim.y = static_cast<int>(
      ceil((global_map_dim_d_y_) / storage_map_info_.resolution));

  local_map_info_.dim.x =
      static_cast<int>(ceil((local_map_dim_d_x_) / local_map_info_.resolution));
  local_map_info_.dim.y =
      static_cast<int>(ceil((local_map_dim_d_y_) / local_map_info_.resolution));

  local_map_info_.origin.z = storage_map_info_.origin.z;

  const Eigen::Vector3d local_dim_d(
      local_map_info_.dim.x * local_map_info_.resolution,
      local_map_info_.dim.y * local_map_info_.resolution,
      local_map_info_.dim.z * local_map_info_.resolution);

  if (!local_ignore_offset_) local_ori_offset_ = -local_dim_d / 2;

  // Initialize maps.
  globalMapInit();
  storageMapInit();
  localInflaInit();

  timer.stop();
}

void LocalGlobalMapperNode::initParams() {
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("lidar_frame", "lidar");
  this->declare_parameter<bool>("real_robot", false);
  this->declare_parameter<std::string>("cloud_msg", "cloud");
  this->declare_parameter<double>("occ_map_height", 2.0);
  this->declare_parameter<double>("robot_r", 0.2);
  this->declare_parameter<double>("robot_h", 0.0);

  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("real_robot", real_robot_);
  this->get_parameter("cloud_msg", cloud_name_);
  this->get_parameter("occ_map_height", occ_map_height_);
  this->get_parameter("robot_r", robot_r_);
  this->get_parameter("robot_h", robot_h_);

  double global_map_origin_x, global_map_origin_y, global_map_origin_z;
  double global_resolution_d;
  this->declare_parameter<double>("global.resolution", 2.0);
  this->get_parameter("global.resolution", global_resolution_d);
  global_map_info_.resolution = static_cast<float>(global_resolution_d);

  this->declare_parameter<double>("global.origin_x", 0.0);
  this->declare_parameter<double>("global.origin_y", 0.0);
  this->declare_parameter<double>("global.origin_z", 0.0);
  this->get_parameter("global.origin_x", global_map_origin_x);
  this->get_parameter("global.origin_y", global_map_origin_y);
  this->get_parameter("global.origin_z", global_map_origin_z);

  this->declare_parameter<double>("global.range_x", 500.0);
  this->declare_parameter<double>("global.range_y", 500.0);
  this->declare_parameter<double>("global.range_z", 2.0);
  this->get_parameter("global.range_x", global_map_dim_d_x_);
  this->get_parameter("global.range_y", global_map_dim_d_y_);
  this->get_parameter("global.range_z", global_map_dim_d_z_);

  this->declare_parameter<int>("global.decay_times_to_empty", 0);
  this->get_parameter("global.decay_times_to_empty",
                      global_decay_times_to_empty_);

  this->declare_parameter<int>("global.num_point_cloud_skip", 5);
  this->get_parameter("global.num_point_cloud_skip", update_interval_);

  this->declare_parameter<double>("global.max_raycast_range", 100.0);
  this->get_parameter("global.max_raycast_range", global_max_raycast_);

  this->declare_parameter<bool>("global.dilate_xy", true);
  this->declare_parameter<bool>("global.dilate_z", false);
  this->get_parameter("global.dilate_xy", global_use_robot_dim_xy_);
  this->get_parameter("global.dilate_z", global_use_robot_dim_z_);

  global_map_info_.origin.x = global_map_origin_x - global_map_dim_d_x_ / 2;
  global_map_info_.origin.y = global_map_origin_y - global_map_dim_d_y_ / 2;
  global_map_info_.origin.z = global_map_origin_z - global_map_dim_d_z_ / 2;
  global_map_info_.dim.x = static_cast<int>(
      ceil((global_map_dim_d_x_) / global_map_info_.resolution));
  global_map_info_.dim.y = static_cast<int>(
      ceil((global_map_dim_d_y_) / global_map_info_.resolution));
  global_map_info_.dim.z = static_cast<int>(
      ceil((global_map_dim_d_z_) / global_map_info_.resolution));

  double local_resolution_d;
  this->declare_parameter<double>("local.resolution", 0.25);
  this->get_parameter("local.resolution", local_resolution_d);
  local_map_info_.resolution = static_cast<float>(local_resolution_d);

  this->declare_parameter<double>("local.range_x", 20.0);
  this->declare_parameter<double>("local.range_y", 20.0);
  this->declare_parameter<double>("local.range_z", 10.0);
  this->get_parameter("local.range_x", local_map_dim_d_x_);
  this->get_parameter("local.range_y", local_map_dim_d_y_);
  this->get_parameter("local.range_z", local_map_dim_d_z_);

  this->declare_parameter<double>("local.max_raycast_range", 20.0);
  this->get_parameter("local.max_raycast_range", local_max_raycast_);

  this->declare_parameter<int>("local.decay_times_to_empty", 0);
  this->get_parameter("local.decay_times_to_empty", local_decay_times_to_empty_);

  this->declare_parameter<bool>("local.ignore_offset", false);
  this->get_parameter("local.ignore_offset", local_ignore_offset_);
}

void LocalGlobalMapperNode::globalMapInit() {
  RCLCPP_WARN(this->get_logger(), "[Mapper]: get 3D map info!");
  const Eigen::Vector3d global_origin(global_map_info_.origin.x,
                                      global_map_info_.origin.y,
                                      global_map_info_.origin.z);
  const Eigen::Vector3d global_dim_d(
      global_map_info_.dim.x * global_map_info_.resolution,
      global_map_info_.dim.y * global_map_info_.resolution,
      global_map_info_.dim.z * global_map_info_.resolution);
  const double global_res = global_map_info_.resolution;
  int8_t global_val_default = 0;
  global_voxel_mapper_.reset(
      new mapper::VoxelMapper(global_origin,
                              global_dim_d,
                              global_res,
                              global_val_default,
                              global_decay_times_to_empty_));

  global_infla_array_.clear();
  if (global_use_robot_dim_xy_) {
    int global_rn = std::ceil(robot_r_ / global_res);
    int global_hn = std::ceil(robot_h_ / global_res);
    for (int nx = -global_rn; nx <= global_rn; ++nx) {
      for (int ny = -global_rn; ny <= global_rn; ++ny) {
        if ((nx == 0 && ny == 0) || (std::hypot(nx, ny) > global_rn)) {
          continue;
        }
        if (global_use_robot_dim_z_) {
          for (int nz = -global_hn; nz <= global_hn; ++nz) {
            global_infla_array_.push_back(Eigen::Vector3i(nx, ny, nz));
          }
        } else {
          global_infla_array_.push_back(Eigen::Vector3i(nx, ny, 0));
        }
      }
    }
  }
}

void LocalGlobalMapperNode::storageMapInit() {
  const Eigen::Vector3d storage_origin(storage_map_info_.origin.x,
                                       storage_map_info_.origin.y,
                                       storage_map_info_.origin.z);
  const Eigen::Vector3d storage_dim_d(
      storage_map_info_.dim.x * storage_map_info_.resolution,
      storage_map_info_.dim.y * storage_map_info_.resolution,
      storage_map_info_.dim.z * storage_map_info_.resolution);
  const double res = storage_map_info_.resolution;
  int8_t storage_val_default = 0;
  storage_voxel_mapper_.reset(
      new mapper::VoxelMapper(storage_origin,
                              storage_dim_d,
                              res,
                              storage_val_default,
                              local_decay_times_to_empty_));
}

void LocalGlobalMapperNode::localInflaInit() {
  const double res = storage_map_info_.resolution;
  local_infla_array_.clear();
  int rn = std::ceil(robot_r_ / res);
  int hn = std::ceil(robot_h_ / res);
  for (int nx = -rn; nx <= rn; ++nx) {
    for (int ny = -rn; ny <= rn; ++ny) {
      for (int nz = -hn; nz <= hn; ++nz) {
        if (nx == 0 && ny == 0) continue;
        if (std::hypot(nx, ny) > rn) continue;
        local_infla_array_.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }
}

void LocalGlobalMapperNode::cropLocalMap(
    const Eigen::Vector3d& center_position_map,
    const Eigen::Vector3d& center_position_odom) {
  const Eigen::Vector3d local_dim_d(
      local_map_info_.dim.x * local_map_info_.resolution,
      local_map_info_.dim.y * local_map_info_.resolution,
      local_map_info_.dim.z * local_map_info_.resolution);
  Eigen::Vector3d local_origin_map = center_position_map + local_ori_offset_;
  local_origin_map(2) = storage_map_info_.origin.z;

  kr_planning_msgs::msg::VoxelMap local_voxel_map =
      storage_voxel_mapper_->getInflatedLocalMap(local_origin_map, local_dim_d);

  Eigen::Vector3d local_origin_odom = center_position_odom + local_ori_offset_;
  local_origin_odom(2) = storage_map_info_.origin.z;
  local_voxel_map.origin.x = local_origin_odom(0);
  local_voxel_map.origin.y = local_origin_odom(1);
  local_voxel_map.origin.z = local_origin_odom(2);

  local_voxel_map.header.frame_id = map_frame_;
  local_map_pub->publish(local_voxel_map);

  kr_planning_msgs::msg::VoxelMap local_voxel_map_non_inflated =
      storage_voxel_mapper_->getLocalMap(local_origin_map, local_dim_d);
  local_voxel_map_non_inflated.origin.x = local_origin_odom(0);
  local_voxel_map_non_inflated.origin.y = local_origin_odom(1);
  local_voxel_map_non_inflated.origin.z = local_origin_odom(2);
  local_voxel_map_non_inflated.header.frame_id = map_frame_;
  local_map_no_inflation_pub->publish(local_voxel_map_non_inflated);
}

void LocalGlobalMapperNode::getLidarPoses(
    const std_msgs::msg::Header& cloud_header,
    geometry_msgs::msg::Pose* pose_map_lidar_ptr,
    geometry_msgs::msg::Pose* pose_odom_lidar_ptr) {
  static mapper::TFListener tf_listener(this->get_clock());
  if (real_robot_) {
    auto tf_map_lidar = tf_listener.LookupTransform(
        map_frame_, lidar_frame_, cloud_header.stamp, this->get_logger());
    auto tf_odom_lidar = tf_listener.LookupTransform(
        odom_frame_, lidar_frame_, cloud_header.stamp, this->get_logger());
    if ((!tf_map_lidar) || (!tf_odom_lidar)) {
      RCLCPP_WARN(
          this->get_logger(),
          "[Mapper real-robot:] Failed to get transform (either from %s to %s; "
          "or from %s to %s)",
          lidar_frame_.c_str(),
          map_frame_.c_str(),
          lidar_frame_.c_str(),
          odom_frame_.c_str());
      return;
    } else {
      *pose_map_lidar_ptr = *tf_map_lidar;
      *pose_odom_lidar_ptr = *tf_odom_lidar;
    }
  } else {
    auto tf_map_lidar = tf_listener.LookupTransform(map_frame_,
                                                    cloud_header.frame_id,
                                                    cloud_header.stamp,
                                                    this->get_logger());
    auto tf_odom_lidar = tf_listener.LookupTransform(odom_frame_,
                                                     cloud_header.frame_id,
                                                     cloud_header.stamp,
                                                     this->get_logger());
    if (!tf_map_lidar) {
      RCLCPP_WARN(
          this->get_logger(),
          "[Mapper simulation:] Failed to get transform map to lidar (from %s "
          "to %s)",
          cloud_header.frame_id.c_str(),
          map_frame_.c_str());
      return;
    } else if (!tf_odom_lidar) {
      RCLCPP_WARN(
          this->get_logger(),
          "[Mapper simulation:] Failed to get transform odom to lidar (from %s "
          "to %s)",
          cloud_header.frame_id.c_str(),
          odom_frame_.c_str());
      return;
    } else {
      *pose_map_lidar_ptr = *tf_map_lidar;
      *pose_odom_lidar_ptr = *tf_odom_lidar;
    }
  }
}

void LocalGlobalMapperNode::processCloud(
    const sensor_msgs::msg::PointCloud& cloud) {
  if ((storage_voxel_mapper_ == nullptr) || (global_voxel_mapper_ == nullptr)) {
    RCLCPP_WARN(this->get_logger(), "voxel mapper not initialized!");
    return;
  }

  geometry_msgs::msg::Pose pose_map_lidar;
  geometry_msgs::msg::Pose pose_odom_lidar;
  getLidarPoses(cloud.header, &pose_map_lidar, &pose_odom_lidar);

  const Eigen::Affine3d T_map_lidar = kr::toTF(pose_map_lidar);
  const Eigen::Affine3d T_odom_lidar = kr::toTF(pose_odom_lidar);

  const Eigen::Vector3d lidar_position_map(T_map_lidar.translation().x(),
                                           T_map_lidar.translation().y(),
                                           T_map_lidar.translation().z());
  const Eigen::Vector3d lidar_position_odom(T_odom_lidar.translation().x(),
                                            T_odom_lidar.translation().y(),
                                            T_odom_lidar.translation().z());

  cpu_times tc;

  double min_range = 0.75;
  double min_range_squared;
  min_range_squared = min_range * min_range;
  const auto pts = kr::cloud_to_vec_filter(cloud, min_range_squared);

  timer.start();
  storage_voxel_mapper_->addCloud(
      pts, T_map_lidar, local_infla_array_, false, local_max_raycast_);
  RCLCPP_DEBUG(this->get_logger(),
               "[storage map addCloud]: %f",
               static_cast<double>(timer.elapsed().wall) / 1e6);

  if (pub_storage_map_) {
    kr_planning_msgs::msg::VoxelMap storage_map =
        storage_voxel_mapper_->getInflatedMap();
    storage_map.header.frame_id = map_frame_;
    storage_map_pub->publish(storage_map);
  }

  timer.start();
  cropLocalMap(lidar_position_map, lidar_position_odom);
  RCLCPP_DEBUG(this->get_logger(),
               "[local map crop]: %f",
               static_cast<double>(timer.elapsed().wall) / 1e6);

  ++counter_;
  if (counter_ % update_interval_ == 0) {
    timer.start();
    global_voxel_mapper_->addCloud(
        pts, T_map_lidar, global_infla_array_, false, global_max_raycast_);
    RCLCPP_DEBUG(this->get_logger(),
                 "[global map addCloud]: %f",
                 static_cast<double>(timer.elapsed().wall) / 1e6);
    timer.start();

    RCLCPP_DEBUG(this->get_logger(),
                 "[global map freeVoxels]: %f",
                 static_cast<double>(timer.elapsed().wall) / 1e6);

    timer.start();
    RCLCPP_DEBUG(this->get_logger(),
                 "[global map getInflatedMap]: %f",
                 static_cast<double>(timer.elapsed().wall) / 1e6);

    counter_ = 0;
    kr_planning_msgs::msg::VoxelMap global_map =
        global_voxel_mapper_->getInflatedMap();
    global_map.header.frame_id = map_frame_;
    global_map_pub->publish(global_map);
  }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(),
                        *this->get_clock(),
                        1000,
                        "[Mapper]: Got cloud, number of points: [%zu]",
                        cloud.points.size());
}

void LocalGlobalMapperNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  RCLCPP_WARN_ONCE(this->get_logger(), "[Mapper]: got the point cloud!");
  sensor_msgs::msg::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);
  processCloud(cloud);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalGlobalMapperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

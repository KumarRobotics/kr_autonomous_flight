#include "mapper/local_global_mapper.h"

// LocalGlobalMapperNode::LocalGlobalMapperNode(const ros::NodeHandle& nh)
//     : nh_(nh) {
LocalGlobalMapperNode::LocalGlobalMapperNode(const rclcpp::NodeOptions & options)
  : Node("local_global_mapper", options){
  initParams();

  // cloud_sub = nh_.subscribe(
  //     cloud_name_, 1, &LocalGlobalMapperNode::cloudCallback, this);
  cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_name_, std::bind(&LocalGlobalMapperNode::cloudCallback, this,
                                std::placeholders::_1));

  // global_map_pub =
  //     nh_.advertise<kr_planning_msgs::VoxelMap>("global_voxel_map", 1, true);
  // storage_map_pub =
  //     nh_.advertise<kr_planning_msgs::VoxelMap>("storage_voxel_map", 1, true);
  // local_map_pub =
  //     nh_.advertise<kr_planning_msgs::VoxelMap>("local_voxel_map", 1, true);

  // time_pub = nh_.advertise<sensor_msgs::Temperature>("/timing/mapper", 1);
  global_map_pub = this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
      "global_voxel_map", 1);
  storage_map_pub = this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
      "storage_voxel_map", 1);
  local_map_pub = this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
      "local_voxel_map", 1);
  time_pub = this->create_publisher<sensor_msgs::msg::Temperature>(
      "/timing/mapper", 1);

  // storage map should have same resolution and z_dim as local map
  storage_map_info_.resolution = local_map_info_.resolution;
  local_map_info_.dim.z = static_cast<int>(
      ceil((local_map_dim_d_z_) / storage_map_info_.resolution));
  storage_map_info_.dim.z = local_map_info_.dim.z;

  // storage map should have same x y z center and x_dim y_dim as global map
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

  // local map range z and center_z will be the same as storage map
  local_map_info_.origin.z = storage_map_info_.origin.z;

  const Eigen::Vector3d local_dim_d(
      local_map_info_.dim.x * local_map_info_.resolution,
      local_map_info_.dim.y * local_map_info_.resolution,
      local_map_info_.dim.z * local_map_info_.resolution);

  // origin is the left lower corner of the voxel map, therefore, adding
  // this offset make the map centered around the given position
  local_ori_offset_ = -local_dim_d / 2;

  // Initialize maps.
  globalMapInit();
  storageMapInit();
  localInflaInit();

  timer.stop();
}

void LocalGlobalMapperNode::initParams() {
  // nh_.param("map_frame", map_frame_, std::string("map"));
  // nh_.param("odom_frame", odom_frame_, std::string("odom"));
  // nh_.param("lidar_frame", lidar_frame_, std::string("lidar"));
  // nh_.param("real_robot", real_robot_, false);
  // nh_.param("cloud_msg", cloud_name_, std::string("cloud"));
  // nh_.param("occ_map_height", occ_map_height_, 2.0);
  // nh_.param("robot_r", robot_r_, 0.2);
  // nh_.param("robot_h", robot_h_, 0.0);
  this->declare_parameter("map_frame", "map");
  map_frame_ = this->get_parameter("map_frame").as_string();
  this->declare_parameter("odom_frame", "odom");
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  this->declare_parameter("lidar_frame", "lidar");
  lidar_frame_ = this->get_parameter("lidar_frame").as_string();
  this->declare_parameter("real_robot", false);
  real_robot_ = this->get_parameter("real_robot").as_bool();
  this->declare_parameter("cloud_msg", "cloud");
  cloud_name_ = this->get_parameter("cloud_msg").as_string();
  this->declare_parameter("occ_map_height", 2.0);
  occ_map_height_ = this->get_parameter("occ_map_height").as_double();
  this->declare_parameter("robot_r", 0.2);
  robot_r_ = this->get_parameter("robot_r").as_double();
  this->declare_parameter("robot_h", 0.0);
  robot_h_ = this->get_parameter("robot_h").as_double();

  double global_map_cx, global_map_cy, global_map_cz;
  // nh_.param("global/resolution", global_map_info_.resolution, 2.0f);
  // nh_.param("global/center_x", global_map_cx, 0.0);
  // nh_.param("global/center_y", global_map_cy, 0.0);
  // nh_.param("global/center_z", global_map_cz, 0.0);
  // nh_.param("global/range_x", global_map_dim_d_x_, 500.0);
  // nh_.param("global/range_y", global_map_dim_d_y_, 500.0);
  // nh_.param("global/range_z", global_map_dim_d_z_, 2.0);
  // nh_.param("global/decay_times_to_empty", global_decay_times_to_empty_, 0);
  this->declare_parameter("global/resolution", 2.0);
  global_map_info_.resolution = this->get_parameter("global/resolution").as_double();
  this->declare_parameter("global/center_x", 0.0);
  global_map_cx = this->get_parameter("global/center_x").as_double();
  this->declare_parameter("global/center_y", 0.0);
  global_map_cy = this->get_parameter("global/center_y").as_double();
  this->declare_parameter("global/center_z", 0.0);
  global_map_cz = this->get_parameter("global/center_z").as_double();
  this->declare_parameter("global/range_x", 500.0);
  global_map_dim_d_x_ =
      this->get_parameter("global/range_x").as_double();
  this->declare_parameter("global/range_y", 500.0);
  global_map_dim_d_y_ =
      this->get_parameter("global/range_y").as_double();
  this->declare_parameter("global/range_z", 2.0);
  global_map_dim_d_z_ =
      this->get_parameter("global/range_z").as_double();
  this->declare_parameter("global/decay_times_to_empty", 0);
  global_decay_times_to_empty_ =
      this->get_parameter("global/decay_times_to_empty").as_int();

  // only update voxel once every update_interval_ point clouds
  // nh_.param("global/num_point_cloud_skip", update_interval_, 5);  // int
  // nh_.param("global/max_raycast_range", global_max_raycast_, 100.0);
  // nh_.param("global/dilate_xy", global_use_robot_dim_xy_, true);
  // nh_.param("global/dilate_z", global_use_robot_dim_z_, false);
  this->declare_parameter("global/num_point_cloud_skip", 5);
  update_interval_ =
      this->get_parameter("global/num_point_cloud_skip").as_int();
  this->declare_parameter("global/max_raycast_range", 100.0);
  global_max_raycast_ =
      this->get_parameter("global/max_raycast_range").as_double();
  this->declare_parameter("global/dilate_xy", true);
  global_use_robot_dim_xy_ =
      this->get_parameter("global/dilate_xy").as_bool();
  this->declare_parameter("global/dilate_z", false);

  // map origin is the left lower corner of the voxel map, therefore, adding
  // an offset make the map centered around the given position
  global_map_info_.origin.x = global_map_cx - global_map_dim_d_x_ / 2;
  global_map_info_.origin.y = global_map_cy - global_map_dim_d_y_ / 2;
  global_map_info_.origin.z = global_map_cz - global_map_dim_d_z_ / 2;
  global_map_info_.dim.x = static_cast<int>(
      ceil((global_map_dim_d_x_) / global_map_info_.resolution));
  global_map_info_.dim.y = static_cast<int>(
      ceil((global_map_dim_d_y_) / global_map_info_.resolution));
  global_map_info_.dim.z = static_cast<int>(
      ceil((global_map_dim_d_z_) / global_map_info_.resolution));

  // nh_.param("local/resolution", local_map_info_.resolution, 0.25f);
  // nh_.param("local/range_x", local_map_dim_d_x_, 20.0);
  // nh_.param("local/range_y", local_map_dim_d_y_, 20.0);
  // nh_.param("local/range_z", local_map_dim_d_z_, 10.0);
  // nh_.param("local/max_raycast_range", local_max_raycast_, 20.0);
  // nh_.param("local/decay_times_to_empty", local_decay_times_to_empty_, 0);
  this->declare_parameter("local/resolution", 0.25);
  local_map_info_.resolution =
      this->get_parameter("local/resolution").as_double();
  this->declare_parameter("local/range_x", 20.0);
  local_map_dim_d_x_ =
      this->get_parameter("local/range_x").as_double();
  this->declare_parameter("local/range_y", 20.0);
  local_map_dim_d_y_ =
      this->get_parameter("local/range_y").as_double();
  this->declare_parameter("local/range_z", 10.0);
  local_map_dim_d_z_ =
      this->get_parameter("local/range_z").as_double();
  this->declare_parameter("local/max_raycast_range", 20.0);
  local_max_raycast_ =
      this->get_parameter("local/max_raycast_range").as_double();
  this->declare_parameter("local/decay_times_to_empty", 0);
  local_decay_times_to_empty_ =
      this->get_parameter("local/decay_times_to_empty").as_int();
}

void LocalGlobalMapperNode::globalMapInit() {
  // TODO(xu): combine two parts into one.
  // ROS_WARN("[Mapper]: get 3D map info!");
  RCLCPP_WARN(this->get_logger(), "[Mapper]: got 3D map info!");

  // part1: global
  const Eigen::Vector3d global_origin(global_map_info_.origin.x,
                                      global_map_info_.origin.y,
                                      global_map_info_.origin.z);
  const Eigen::Vector3d global_dim_d(
      global_map_info_.dim.x * global_map_info_.resolution,
      global_map_info_.dim.y * global_map_info_.resolution,
      global_map_info_.dim.z * global_map_info_.resolution);
  const double global_res = global_map_info_.resolution;
  int8_t global_val_default = 0;
  // Initialize the mapper
  global_voxel_mapper_.reset(
      new mapper::VoxelMapper(global_origin,
                              global_dim_d,
                              global_res,
                              global_val_default,
                              global_decay_times_to_empty_));

  // build the array for map inflation
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
  // Initialize the mapper
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

  // core function: crop local map from the storage map
  // kr_planning_msgs::VoxelMap local_voxel_map =
  //     storage_voxel_mapper_->getInflatedLocalMap(local_origin_map, local_dim_d);
  kr_planning_msgs::msg::VoxelMap local_voxel_map =
  storage_voxel_mapper_->getInflatedLocalMap(local_origin_map, local_dim_d);

  // Transform local map by moving its origin. This is because we want the
  // cropped map to be centered around lidar's position wrt odom frame
  Eigen::Vector3d local_origin_odom = center_position_odom + local_ori_offset_;
  local_origin_odom(2) = storage_map_info_.origin.z;
  local_voxel_map.origin.x = local_origin_odom(0);
  local_voxel_map.origin.y = local_origin_odom(1);
  local_voxel_map.origin.z = local_origin_odom(2);

  // The local map should still be published in map frame, not odom frame
  // (since we already applied transform between odom and map frame already to
  // compensate for the drift)
  local_voxel_map.header.frame_id = map_frame_;
  // local_map_pub.publish(local_voxel_map);
  local_map_pub->publish(local_voxel_map);
}

void LocalGlobalMapperNode::getLidarPoses(
    const std_msgs::msg::Header& cloud_header,
    geometry_msgs::msg::Pose* pose_map_lidar_ptr,
    geometry_msgs::msg::Pose* pose_odom_lidar_ptr) {
  // get the transform from fixed frame to lidar frame
  static mapper::TFListener tf_listener;
  if (real_robot_) {
    // for real robot, the point cloud frame_id may not exist in the tf tree,
    // manually defining it here.
    // TODO(xu): make this automatic
    auto tf_map_lidar = tf_listener.LookupTransform(
        map_frame_, lidar_frame_, cloud_header.stamp);
    auto tf_odom_lidar = tf_listener.LookupTransform(
        odom_frame_, lidar_frame_, cloud_header.stamp);
    if ((!tf_map_lidar) || (!tf_odom_lidar)) {
      RCLCPP_WARN(this->get_logger(),
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
    auto tf_map_lidar = tf_listener.LookupTransform(
        map_frame_, cloud_header.frame_id, cloud_header.stamp);
    auto tf_odom_lidar = tf_listener.LookupTransform(
        odom_frame_, cloud_header.frame_id, cloud_header.stamp);
    if (!tf_map_lidar) {
      RCLCPP_WARN(this->get_logger(),
          "[Mapper simulation:] Failed to get transform map to lidar (from %s "
          "to %s)",
          cloud_header.frame_id.c_str(),
          map_frame_.c_str());
      return;
    } else if (!tf_odom_lidar) {
      RCLCPP_WARN(this->get_logger(),
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

// void LocalGlobalMapperNode::processCloud(const sensor_msgs::PointCloud& cloud) {
  void LocalGlobalMapperNode::processCloud(const sensor_msgs::msg::PointCloud& cloud) {
  if ((storage_voxel_mapper_ == nullptr) || (global_voxel_mapper_ == nullptr)) {
    RCLCPP_WARN(this->get_logger(),"voxel mapper not initialized!");
    return;
  }

  geometry_msgs::Pose pose_map_lidar;
  geometry_msgs::Pose pose_odom_lidar;
  getLidarPoses(cloud.header, &pose_map_lidar, &pose_odom_lidar);

  const Eigen::Affine3d T_map_lidar = kr::toTF(pose_map_lidar);
  const Eigen::Affine3d T_odom_lidar = kr::toTF(pose_odom_lidar);

  // This is the lidar position in the odom frame, used for raytracing &
  // cropping local map
  const Eigen::Vector3d lidar_position_map(T_map_lidar.translation().x(),
                                           T_map_lidar.translation().y(),
                                           T_map_lidar.translation().z());
  // This is the lidar position in the odom frame
  // used for transforming local map to center around lidar's position in odom
  // frame
  const Eigen::Vector3d lidar_position_odom(T_odom_lidar.translation().x(),
                                            T_odom_lidar.translation().y(),
                                            T_odom_lidar.translation().z());

  // ros::Time t0;
  rclcpp::Time t0;
  cpu_times tc;

  double min_range = 0.75;  // points within this distance will be discarded
  double min_range_squared;
  min_range_squared = min_range * min_range;
  const auto pts = kr::cloud_to_vec_filter(cloud, min_range_squared);

  timer.start();
  // local raytracing using lidar position in the map frame (not odom frame)
  storage_voxel_mapper_->addCloud(
      pts, T_map_lidar, local_infla_array_, false, local_max_raycast_);
  RCLCPP_DEBUG(this->get_logger(),"[storage map addCloud]: %f",
            static_cast<double>(timer.elapsed().wall) / 1e6);

  // get and publish storage map (this is very slow)
  if (pub_storage_map_) {
    // kr_planning_msgs::VoxelMap storage_map =
    //     storage_voxel_mapper_->getInflatedMap();
    kr_planning_msgs::msg::VoxelMap storage_map =
        storage_voxel_mapper_->getInflatedMap();
    storage_map.header.frame_id = map_frame_;
    // storage_map_pub.publish(storage_map);
    storage_map_pub->publish(storage_map);
  }

  timer.start();
  // crop local voxel map
  // the cropping step is using the lidar position in map frame
  cropLocalMap(lidar_position_map, lidar_position_odom);
  RCLCPP_WARN(this->get_logger(),"[local map crop]: %f",
            static_cast<double>(timer.elapsed().wall) / 1e6);

  // only global voxel map once every update_interval_ point clouds
  ++counter_;
  if (counter_ % update_interval_ == 0) {
    timer.start();
    global_voxel_mapper_->addCloud(
        pts, T_map_lidar, global_infla_array_, false, global_max_raycast_);
    RCLCPP_WARN(this->get_logger(),"[global map addCloud]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);
    timer.start();

    // for global map, free voxels surrounding the robot to make sure the start
    // of the global planner is not occupied
    // global_voxel_mapper_->freeVoxels(lidar_position_map, clear_ns_);
    RCLCPP_WARN(this->get_logger(),"[global map freeVoxels]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    timer.start();
    RCLCPP_WARN(this->get_logger(),"[global map getInflatedMap]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    counter_ = 0;
    kr_planning_msgs::VoxelMap global_map =
        global_voxel_mapper_->getInflatedMap();
    global_map.header.frame_id = map_frame_;
    // global_map_pub.publish(global_map);
    global_map_pub->publish(global_map);
  }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this,
      1, "[Mapper]: Got cloud, number of points: [%zu]", cloud.points.size());
}

// void LocalGlobalMapperNode::cloudCallback(
//     const sensor_msgs::PointCloud2::ConstPtr& msg) {
void LocalGlobalMapperNode::cloudCallback(
       const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
 RCLCPP_WARN_ONCE(this->get_logger(),"[Mapper]: got the point cloud!");
  sensor_msgs::msg::PointCloud cloud;
  sensor_msgs::msg::convertPointCloud2ToPointCloud(*msg, cloud);
  processCloud(cloud);
}

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "local_global_mapper");
//   ros::NodeHandle nh("~");

//   LocalGlobalMapperNode lgMapper(nh);
//   ros::spin();
//   return 0;
// }

int main(int argc, char** argv)
{
  // 1) Initialize the ROS 2 client library
  rclcpp::init(argc, argv);

  // 2) Create your node (we forward default NodeOptions here,
  //    but you could customize namespace, parameters, etc.)
  auto lgMapper = std::make_shared<LocalGlobalMapperNode>(rclcpp::NodeOptions());

  // 3) Spin it until shutdown
  rclcpp::spin(lgMapper);

  // 4) Shutdown ROS 2 cleanly
  rclcpp::shutdown();
  return 0;
}

#include "mapper/local_global_mapper.h"

LocalGlobalMapperNode::LocalGlobalMapperNode(const ros::NodeHandle& nh)
    : nh_(nh) {
  initParams();

  cloud_sub = nh_.subscribe(
      cloud_name_, 1, &LocalGlobalMapperNode::cloudCallback, this);

  global_map_pub =
      nh_.advertise<kr_planning_msgs::VoxelMap>("global_voxel_map", 1, true);
  storage_map_pub =
      nh_.advertise<kr_planning_msgs::VoxelMap>("storage_voxel_map", 1, true);
  local_map_pub =
      nh_.advertise<kr_planning_msgs::VoxelMap>("local_voxel_map", 1, true);

  time_pub = nh_.advertise<sensor_msgs::Temperature>("/timing/mapper", 1);

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
  nh_.param("map_frame", map_frame_, std::string("map"));
  nh_.param("odom_frame", odom_frame_, std::string("odom"));
  nh_.param("lidar_frame", lidar_frame_, std::string("lidar"));
  nh_.param("real_robot", real_robot_, false);
  nh_.param("cloud_msg", cloud_name_, std::string("cloud"));
  nh_.param("occ_map_height", occ_map_height_, 2.0);
  nh_.param("robot_r", robot_r_, 0.2);
  nh_.param("robot_h", robot_h_, 0.0);

  double global_map_cx, global_map_cy, global_map_cz;
  nh_.param("global/resolution", global_map_info_.resolution, 2.0f);
  nh_.param("global/center_x", global_map_cx, 0.0);
  nh_.param("global/center_y", global_map_cy, 0.0);
  nh_.param("global/center_z", global_map_cz, 0.0);
  nh_.param("global/range_x", global_map_dim_d_x_, 500.0);
  nh_.param("global/range_y", global_map_dim_d_y_, 500.0);
  nh_.param("global/range_z", global_map_dim_d_z_, 2.0);
  nh_.param("global/decay_times_to_empty", global_decay_times_to_empty_, 0);

  // only update voxel once every update_interval_ point clouds
  nh_.param("global/num_point_cloud_skip", update_interval_, 5);  // int
  nh_.param("global/max_raycast_range", global_max_raycast_, 100.0);
  nh_.param("global/dilate_xy", global_use_robot_dim_xy_, true);
  nh_.param("global/dilate_z", global_use_robot_dim_z_, false);

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

  nh_.param("local/resolution", local_map_info_.resolution, 0.25f);
  nh_.param("local/range_x", local_map_dim_d_x_, 20.0);
  nh_.param("local/range_y", local_map_dim_d_y_, 20.0);
  nh_.param("local/range_z", local_map_dim_d_z_, 10.0);
  nh_.param("local/max_raycast_range", local_max_raycast_, 20.0);
  nh_.param("local/decay_times_to_empty", local_decay_times_to_empty_, 0);
}

void LocalGlobalMapperNode::globalMapInit() {
  // TODO(xu): combine two parts into one.
  ROS_WARN("[Mapper]: get 3D map info!");
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
  kr_planning_msgs::VoxelMap local_voxel_map =
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
  local_map_pub.publish(local_voxel_map);
}

void LocalGlobalMapperNode::getLidarPoses(
    const std_msgs::Header& cloud_header,
    geometry_msgs::Pose* pose_map_lidar_ptr,
    geometry_msgs::Pose* pose_odom_lidar_ptr) {
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
      ROS_WARN(
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
      ROS_WARN(
          "[Mapper simulation:] Failed to get transform map to lidar (from %s "
          "to %s)",
          cloud_header.frame_id.c_str(),
          map_frame_.c_str());
      return;
    } else if (!tf_odom_lidar) {
      ROS_WARN(
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

void LocalGlobalMapperNode::processCloud(const sensor_msgs::PointCloud& cloud) {
  if ((storage_voxel_mapper_ == nullptr) || (global_voxel_mapper_ == nullptr)) {
    ROS_WARN("voxel mapper not initialized!");
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

  ros::Time t0;
  cpu_times tc;

  double min_range = 0.75;  // points within this distance will be discarded
  double min_range_squared;
  min_range_squared = min_range * min_range;
  const auto pts = kr::cloud_to_vec_filter(cloud, min_range_squared);

  timer.start();
  // local raytracing using lidar position in the map frame (not odom frame)
  storage_voxel_mapper_->addCloud(
      pts, T_map_lidar, local_infla_array_, false, local_max_raycast_);
  ROS_DEBUG("[storage map addCloud]: %f",
            static_cast<double>(timer.elapsed().wall) / 1e6);

  // get and publish storage map (this is very slow)
  if (pub_storage_map_) {
    kr_planning_msgs::VoxelMap storage_map =
        storage_voxel_mapper_->getInflatedMap();
    storage_map.header.frame_id = map_frame_;
    storage_map_pub.publish(storage_map);
  }

  timer.start();
  // crop local voxel map
  // the cropping step is using the lidar position in map frame
  cropLocalMap(lidar_position_map, lidar_position_odom);
  ROS_DEBUG("[local map crop]: %f",
            static_cast<double>(timer.elapsed().wall) / 1e6);

  // only global voxel map once every update_interval_ point clouds
  ++counter_;
  if (counter_ % update_interval_ == 0) {
    timer.start();
    global_voxel_mapper_->addCloud(
        pts, T_map_lidar, global_infla_array_, false, global_max_raycast_);
    ROS_DEBUG("[global map addCloud]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);
    timer.start();

    // for global map, free voxels surrounding the robot to make sure the start
    // of the global planner is not occupied
    // global_voxel_mapper_->freeVoxels(lidar_position_map, clear_ns_);
    ROS_DEBUG("[global map freeVoxels]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    timer.start();
    ROS_DEBUG("[global map getInflatedMap]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    counter_ = 0;
    kr_planning_msgs::VoxelMap global_map =
        global_voxel_mapper_->getInflatedMap();
    global_map.header.frame_id = map_frame_;
    global_map_pub.publish(global_map);
  }

  ROS_DEBUG_THROTTLE(
      1, "[Mapper]: Got cloud, number of points: [%zu]", cloud.points.size());
}

void LocalGlobalMapperNode::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_WARN_ONCE("[Mapper]: got the point cloud!");
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);
  processCloud(cloud);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_global_mapper");
  ros::NodeHandle nh("~");

  LocalGlobalMapperNode lgMapper(nh);
  ros::spin();
  return 0;
}

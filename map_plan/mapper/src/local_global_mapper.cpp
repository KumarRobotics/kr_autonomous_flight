#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>

#include <boost/timer/timer.hpp>

#include "mapper/tf_listener.h"
#include "mapper/voxel_mapper.h"

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;
cpu_timer timer;
// This fils is modified based on mapper/cloud_to_map.cpp

// Timing stuff
ros::Publisher time_pub;

std::unique_ptr<mapper::VoxelMapper> global_voxel_mapper_;   // mapper
std::unique_ptr<mapper::VoxelMapper> storage_voxel_mapper_;  // mapper
// std::unique_ptr<VoxelMapper> local_voxel_mapper_;  // mapper

planning_ros_msgs::VoxelMap global_map_info_;
planning_ros_msgs::VoxelMap storage_map_info_;
planning_ros_msgs::VoxelMap local_map_info_;

ros::Publisher global_map_pub;
ros::Publisher storage_map_pub;
ros::Publisher local_map_pub;

// ros::Publisher global_occ_map_pub;
ros::Publisher local_voxel_map_pub;
// ros::Publisher local_cloud_pub;

bool debug_;
bool real_robot_;          // define it's real-robot experiment or not
std::string map_frame_;    // map frame
std::string lidar_frame_;  // lidar frame
std::string cloud_name_;   // cloud msg name frame
// std::string odom_name_;    // odom msg name frame
// nav_msgs::Odometry last_odom_; // record of robot odom
// bool odom_received_ = false;

vec_Vec3i global_infla_array_;  // inflation array
vec_Vec3i local_infla_array_;   // inflation array

double robot_r_, robot_h_;
bool global_use_robot_dim_;

double local_max_raycast_, global_max_raycast_;  // maximum raycasting range
double occ_map_height_;
Eigen::Vector3d local_ori_offset_;

int update_interval_;
int counter_ = 0;
int counter_clear_ = 0;

vec_Vec3i clear_ns_;

void cropLocalMap(const Eigen::Vector3d &center_position) {
  const Eigen::Vector3d local_dim(local_map_info_.dim.x, local_map_info_.dim.y,
                                  local_map_info_.dim.z);
  Eigen::Vector3d local_origin = center_position + local_ori_offset_;
  local_origin(2) = local_map_info_.origin.z;

  // core function: crop local map from the storage map
  auto local_voxel_map =
      storage_voxel_mapper_->getInflatedLocalMap(local_origin, local_dim);
  local_voxel_map.header.frame_id = map_frame_;
  local_map_pub.publish(local_voxel_map);
}

void processCloud(const sensor_msgs::PointCloud &cloud) {
  if ((storage_voxel_mapper_ == nullptr) || (global_voxel_mapper_ == nullptr)) {
    ROS_WARN("voxel mapper not initialized!");
    return;
  }

  static mapper::TFListener tf_listener;
  // get the transform from fixed frame to lidar frame
  geometry_msgs::Pose pose_map_cloud;
  if (real_robot_) {
    // for real robot, the point cloud frame_id may not exist in the tf tree,
    // manually defining it here.
    // TODO(xu): make this automatic
    auto tf_map_cloud = tf_listener.LookupTransform(map_frame_, lidar_frame_,
                                                    cloud.header.stamp);
    if (!tf_map_cloud) {
      ROS_WARN("Failed to get transform from %s to %s", lidar_frame_.c_str(),
               map_frame_.c_str());
      return;
    }
    pose_map_cloud = *tf_map_cloud;
  } else {
    auto tf_map_cloud = tf_listener.LookupTransform(
        map_frame_, cloud.header.frame_id, cloud.header.stamp);
    if (!tf_map_cloud) {
      ROS_WARN("Failed to get transform from %s to %s",
               cloud.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    pose_map_cloud = *tf_map_cloud;
  }
  const Eigen::Affine3d T_m_c = toTF(pose_map_cloud);
  // This is the lidar position in the fixed frame
  const Eigen::Vector3d sensor_position(T_m_c.translation().x(),
                                        T_m_c.translation().y(),
                                        T_m_c.translation().z());

  ros::Time t0;
  cpu_times tc;

  double min_range = 0.75;  // points within this distance will be discarded
  double min_range_squared;
  min_range_squared = min_range * min_range;
  const auto pts = cloud_to_vec_filter(cloud, min_range_squared);

  timer.start();
  storage_voxel_mapper_->addCloud(pts, T_m_c, local_infla_array_, false,
                                  local_max_raycast_);
  ROS_DEBUG("[storage map addCloud]: %f",
            static_cast<double>(timer.elapsed().wall) / 1e6);

  // crop local voxel map
  timer.start();
  cropLocalMap(sensor_position);
  ROS_DEBUG("[local map crop]: %f",
            static_cast<double>(timer.elapsed().wall) / 1e6);

  //  const double total_t = dt_storage_add + dt_local_crop;
  //  if (total_t > 0.5) {
  //    ROS_WARN(
  //        "[Mapper]: Time for processing storage and local map is too
  //        large!!!");
  //    ROS_WARN(
  //        "Time for updating storage map: %f. Total "
  //        "time for cropping and publishing local map: %f",
  //        dt_storage_add, dt_local_crop);
  //  }

  //  sensor_msgs::Temperature tmsg;
  //  tmsg.header.stamp = t0;
  //  tmsg.temperature = total_t * 1000;
  //  time_pub.publish(tmsg);

  // only global voxel map once every update_interval_ point clouds
  if (counter_ % update_interval_ == 0) {
    timer.start();
    global_voxel_mapper_->addCloud(pts, T_m_c, global_infla_array_, false,
                                   global_max_raycast_);
    ROS_DEBUG("[global map addCloud]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    // for global map, free voxels surrounding the robot to make sure the start
    // of the global planner is not occupied
    timer.start();
    global_voxel_mapper_->freeVoxels(sensor_position, clear_ns_);
    ROS_DEBUG("[global map freeVoxels]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    timer.start();
    auto global_map = global_voxel_mapper_->getInflatedMap();
    ROS_DEBUG("[global map getInflatedMap]: %f",
              static_cast<double>(timer.elapsed().wall) / 1e6);

    global_map.header.frame_id = map_frame_;
    global_map_pub.publish(global_map);

    counter_ = 0;
  }
  ++counter_;

  ROS_DEBUG_THROTTLE(1, "[Mapper]: Got cloud, number of points: [%zu]",
                     cloud.points.size());
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  ROS_WARN_ONCE("[Mapper]: got the point cloud!");
  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);
  processCloud(cloud);
}

void mapInit() {
  // TODO(xu): combine two parts into one.
  ROS_WARN("[Mapper]: get 3D map info!");
  // part1: global
  const Eigen::Vector3d global_origin(global_map_info_.origin.x,
                                      global_map_info_.origin.y,
                                      global_map_info_.origin.z);
  const Eigen::Vector3d global_dim(
      global_map_info_.dim.x, global_map_info_.dim.y, global_map_info_.dim.z);
  const double global_res = global_map_info_.resolution;
  // Initialize the mapper
  global_voxel_mapper_.reset(
      new mapper::VoxelMapper(global_origin, global_dim, global_res));
  global_infla_array_.clear();
  if (global_use_robot_dim_) {
    int global_rn = std::ceil(robot_r_ / global_res);
    int global_hn = std::ceil(robot_h_ / global_res);
    for (int nx = -global_rn; nx <= global_rn; ++nx) {
      for (int ny = -global_rn; ny <= global_rn; ++ny) {
        for (int nz = -global_hn; nz <= global_hn; ++nz) {
          if (nx == 0 && ny == 0) continue;
          if (std::hypot(nx, ny) > global_rn) continue;
          global_infla_array_.push_back(Eigen::Vector3i(nx, ny, nz));
        }
      }
    }
  }

  // part2: local
  const Eigen::Vector3d origin(storage_map_info_.origin.x,
                               storage_map_info_.origin.y,
                               storage_map_info_.origin.z);
  const Eigen::Vector3d dim(storage_map_info_.dim.x, storage_map_info_.dim.y,
                            storage_map_info_.dim.z);
  const double res = storage_map_info_.resolution;
  // Initialize the mapper
  storage_voxel_mapper_.reset(new mapper::VoxelMapper(origin, dim, res));
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_to_map");
  ros::NodeHandle nh("~");

  nh.param("map_frame", map_frame_, std::string("map"));
  nh.param("lidar_frame", lidar_frame_, std::string("lidar"));
  nh.param("real_robot", real_robot_, false);
  nh.param("debug", debug_, true);
  nh.param("cloud_msg", cloud_name_, std::string("cloud"));
  // nh.param("odom_msg", odom_name_, true);

  ros::Subscriber cloud_sub = nh.subscribe(cloud_name_, 1, cloudCallback);
  // ros::Subscriber odom_sub = nh.subscribe(odom_name_, 1, odomCallback);

  global_map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("global_voxel_map", 1, true);
  storage_map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("storage_voxel_map", 1, true);
  local_map_pub =
      nh.advertise<planning_ros_msgs::VoxelMap>("local_voxel_map", 1, true);

  // global_occ_map_pub =
  // nh.advertise<planning_ros_msgs::VoxelMap>("global_occ_map", 1, true);

  // local_cloud_pub =
  //     nh.advertise<sensor_msgs::PointCloud>("local_cloud", 1, true);

  time_pub = nh.advertise<sensor_msgs::Temperature>("/timing/mapper", 1);

  nh.param("occ_map_height", occ_map_height_, 2.0);

  nh.param("robot_r", robot_r_, 0.2);
  nh.param("robot_h", robot_h_, 0.0);

  double global_map_cx, global_map_cy, global_map_cz;
  nh.param("global/resolution", global_map_info_.resolution, 2.0f);
  nh.param("global/center_x", global_map_cx, 0.0);
  nh.param("global/center_y", global_map_cy, 0.0);
  nh.param("global/center_z", global_map_cz, 0.0);
  nh.param("global/range_x", global_map_info_.dim.x, 500.);
  nh.param("global/range_y", global_map_info_.dim.y, 500.);
  nh.param("global/range_z", global_map_info_.dim.z, 2.0);
  // only update voxel once every update_interval_ point clouds
  nh.param("global/num_point_cloud_skip", update_interval_, 5);  // int
  nh.param("global/max_raycast_range", global_max_raycast_, 100.0);
  nh.param("global/consider_robot_dim", global_use_robot_dim_, false);

  // map origin is the left lower corner of the voxel map, therefore, adding an
  // offset make the map centered around the given position
  global_map_info_.origin.x = global_map_cx - global_map_info_.dim.x / 2;
  global_map_info_.origin.y = global_map_cy - global_map_info_.dim.y / 2;
  global_map_info_.origin.z = global_map_cz - global_map_info_.dim.z / 2;

  double storage_map_cx, storage_map_cy, storage_map_cz;
  nh.param("storage/center_x", storage_map_cx, 0.0);
  nh.param("storage/center_y", storage_map_cy, 0.0);
  nh.param("storage/center_z", storage_map_cz, 5.0);
  nh.param("storage/range_x", storage_map_info_.dim.x, 500.);
  nh.param("storage/range_y", storage_map_info_.dim.y, 500.);
  nh.param("storage/range_z", storage_map_info_.dim.z, 500.);

  nh.param("local/resolution", local_map_info_.resolution, 0.25f);
  nh.param("local/range_x", local_map_info_.dim.x, 20.);
  nh.param("local/range_y", local_map_info_.dim.y, 20.);
  // nh.param("local/range_z", local_map_info_.dim.z, 10.0);
  nh.param("local/max_raycast_range", local_max_raycast_, 20.0);

  storage_map_info_.resolution = local_map_info_.resolution;
  // storage_map_info_.dim.z = local_map_info_.dim.z;

  storage_map_info_.origin.x = storage_map_cx - storage_map_info_.dim.x / 2;
  storage_map_info_.origin.y = storage_map_cy - storage_map_info_.dim.y / 2;
  storage_map_info_.origin.z = storage_map_cz - storage_map_info_.dim.z / 2;

  // local map range z and center_z will be the same as storage map
  local_map_info_.dim.z = storage_map_info_.dim.z;
  local_map_info_.origin.z = storage_map_info_.origin.z;

  const Eigen::Vector3d local_dim(local_map_info_.dim.x, local_map_info_.dim.y,
                                  local_map_info_.dim.z);
  // origin is the left lower corner of the voxel map, therefore, adding
  // this offset make the map centered around the given position
  local_ori_offset_ = -local_dim / 2;

  // dimension (in voxels) of the region to free voxels
  for (int nx = -1; nx <= 1; nx++) {
    for (int ny = -1; ny <= 1; ny++) {
      for (int nz = -1; nz <= 1; nz++) {
        clear_ns_.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }

  // Initialize maps
  mapInit();
  timer.stop();

  ros::spin();

  return 0;
}

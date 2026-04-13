#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>

#include <boost/timer/timer.hpp>
#include <memory>
#include <string>

#include "mapper/tf_listener.h"
#include "mapper/voxel_mapper.h"

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;

class LocalGlobalMapperNode : public rclcpp::Node {
 public:
  /**
   * @brief Local Global Mapper Constructor
   */
  LocalGlobalMapperNode();

 private:
  /**
   * @brief Reads parameters from ROS parameter server
   */
  void initParams();

  /**
   * @brief Crops the local map from the storage map, transforms it to odometry
   * frame and publishes
   */
  void cropLocalMap(const Eigen::Vector3d& center_position_map,
                    const Eigen::Vector3d& center_position_odom);

  /**
   * @brief Lookup the transform from lidar to map frame and from lidar to odom
   * frame
   */
  void getLidarPoses(const std_msgs::msg::Header& cloud_header,
                     geometry_msgs::msg::Pose* pose_map_lidar_ptr,
                     geometry_msgs::msg::Pose* pose_odom_lidar_ptr);

  /**
   * @brief Adds input cloud to storage map, publishes new local map and global
   * map (if enough msgs were received)
   */
  void processCloud(const sensor_msgs::msg::PointCloud& cloud);

  /**
   * @brief Point Cloud topic callback. Will convert from PointCloud2 to
   * PointCloud
   */
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  /**
   * @brief Allocates memory for the global mapper and initializes the arrays
   * for inflation
   */
  void globalMapInit();

  /**
   * @brief Allocates memory for the storage mapper
   */
  void storageMapInit();

  /**
   * @brief Initializes the array for inflation for the local map
   */
  void localInflaInit();

  cpu_timer timer;

  // Timing stuff
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr time_pub;

  std::unique_ptr<mapper::VoxelMapper> global_voxel_mapper_;
  std::unique_ptr<mapper::VoxelMapper> storage_voxel_mapper_;

  kr_planning_msgs::msg::VoxelMap global_map_info_;
  kr_planning_msgs::msg::VoxelMap storage_map_info_;
  kr_planning_msgs::msg::VoxelMap local_map_info_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp::Publisher<kr_planning_msgs::msg::VoxelMap>::SharedPtr global_map_pub;
  rclcpp::Publisher<kr_planning_msgs::msg::VoxelMap>::SharedPtr storage_map_pub;
  rclcpp::Publisher<kr_planning_msgs::msg::VoxelMap>::SharedPtr local_map_pub;
  rclcpp::Publisher<kr_planning_msgs::msg::VoxelMap>::SharedPtr
      local_map_no_inflation_pub;

  bool real_robot_;
  std::string map_frame_;
  std::string odom_frame_;

  std::string lidar_frame_;
  std::string cloud_name_;

  vec_Vec3i global_infla_array_;
  vec_Vec3i local_infla_array_;

  int local_decay_times_to_empty_;
  int global_decay_times_to_empty_;

  double robot_r_, robot_h_;
  bool global_use_robot_dim_xy_;
  bool global_use_robot_dim_z_;
  double global_map_dim_d_x_, global_map_dim_d_y_, global_map_dim_d_z_;
  double local_map_dim_d_x_, local_map_dim_d_y_, local_map_dim_d_z_;

  double local_max_raycast_, global_max_raycast_;
  double occ_map_height_;
  Eigen::Vector3d local_ori_offset_{Eigen::Vector3d::Zero()};
  bool local_ignore_offset_;
  bool pub_storage_map_ = false;

  int update_interval_;
  int counter_ = 0;
  int counter_clear_ = 0;

  vec_Vec3i clear_ns_;
};

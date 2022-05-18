#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <kr_planning_rviz_plugins/data_ros_utils.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Bool.h>

#include <boost/timer/timer.hpp>
#include <memory>
#include <string>

#include "mapper/tf_listener.h"
#include "mapper/voxel_mapper.h"

// Timer stuff
using boost::timer::cpu_timer;
using boost::timer::cpu_times;

class LocalGlobalMapperNode {
 public:
  /**
   * @brief Local Global Mapper Constructor
   * @param nh ROS Node handler
   */
  explicit LocalGlobalMapperNode(const ros::NodeHandle& nh);

 private:
  /**
   * @brief Reads parameters from ROS parameter server
   */
  void initParams();

  /**
   * @brief Crops the local map from the storage map, transforms it to odometry
   * frame and publishes
   * @param center_position_map  Robot position in map frame (for sampling the
   * storage map)
   * @param center_position_odom  Robot position in odom frame (to shift the
   * local map)
   */
  void cropLocalMap(const Eigen::Vector3d& center_position_map,
                    const Eigen::Vector3d& center_position_odom);

  /**
   * @brief Lookup the transform from lidar to map frame and from lidar to odom
   * frame
   * @param cloud_header  Input cloud message header
   * @param pose_map_lidar_ptr  Output tf from lidar to map
   * @param pose_odom_lidar_ptr  Output tf from lidar to odom
   */
  void getLidarPoses(const std_msgs::Header& cloud_header,
                     geometry_msgs::Pose* pose_map_lidar_ptr,
                     geometry_msgs::Pose* pose_odom_lidar_ptr);

  /**
   * @brief Adds input cloud to storage map, publishes new local map and global
   * map (if enough msgs were received)
   * @param cloud Input cloud message
   */
  void processCloud(const sensor_msgs::PointCloud& cloud);

  /**
   * @brief Point Cloud topic callback. Will convert from
   * sensor_msgs::PointCloud2 to sensor_msgs::PointCloud
   * @param msg Const pointer to input cloud message
   */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

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
  ros::Publisher time_pub;

  std::unique_ptr<mapper::VoxelMapper> global_voxel_mapper_;   // mapper
  std::unique_ptr<mapper::VoxelMapper> storage_voxel_mapper_;  // mapper
  // std::unique_ptr<VoxelMapper> local_voxel_mapper_;  // mapper

  kr_planning_msgs::VoxelMap global_map_info_;
  kr_planning_msgs::VoxelMap storage_map_info_;
  kr_planning_msgs::VoxelMap local_map_info_;

  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub;
  ros::Publisher global_map_pub;
  ros::Publisher storage_map_pub;
  ros::Publisher local_map_pub;

  // ros::Publisher global_occ_map_pub;
  ros::Publisher local_voxel_map_pub;
  // ros::Publisher local_cloud_pub;

  bool real_robot_;         // define it's real-robot experiment or not
  std::string map_frame_;   // map frame
  std::string odom_frame_;  // odom frame

  std::string lidar_frame_;  // lidar frame
  std::string cloud_name_;   // cloud msg name frame
  // std::string odom_name_;    // odom msg name frame
  // nav_msgs::Odometry last_odom_; // record of robot odom
  // bool odom_received_ = false;

  vec_Vec3i global_infla_array_;  // inflation array
  vec_Vec3i local_infla_array_;   // inflation array

  // number of times of decay for an occupied voxel to be decayed into empty
  // cell, 0 means no decay
  int local_decay_times_to_empty_;   // for local and storage map
  int global_decay_times_to_empty_;  // for global map

  double robot_r_, robot_h_;
  bool global_use_robot_dim_xy_;
  bool global_use_robot_dim_z_;
  double global_map_dim_d_x_, global_map_dim_d_y_, global_map_dim_d_z_;
  double local_map_dim_d_x_, local_map_dim_d_y_, local_map_dim_d_z_;

  double local_max_raycast_, global_max_raycast_;  // maximum raycasting range
  double occ_map_height_;
  Eigen::Vector3d local_ori_offset_;
  bool pub_storage_map_ =
      false;  // don't set this as true unless you're debugging, it's very slow

  int update_interval_;
  int counter_ = 0;
  int counter_clear_ = 0;

  vec_Vec3i clear_ns_;
};
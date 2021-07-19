#pragma once

#include <planning_ros_msgs/VoxelMap.h>

#include <Eigen/Geometry>
#include <boost/multi_array.hpp>

namespace mapper {

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using vec_Vec3d = AlignedVector<Eigen::Vector3d>;
using vec_Vec3i = AlignedVector<Eigen::Vector3i>;

class VoxelMapper {
 public:
  /**
   * @brief Simple constructor
   * @param origin the origin of the map (float), should be the left-most corner
   * @param dim the range of map in x-y-z axes (float)
   * @param res voxel resolution
   * @param val set default map value
   * @param decay_times_to_empty number of times of decay for an occupied voxel
   * to be decayed into empty cell, 0 means no decay
   */
  VoxelMapper(const Eigen::Vector3d& origin, const Eigen::Vector3d& dim,
              double res, int8_t val = 0, int decay_times_to_empty = 0);

  /// Set all voxels as unknown
  void setMapUnknown();
  /// Set all voxels as free
  void setMapFree();

  /// Get the occupied voxels
  vec_Vec3d getCloud();
  /// Get the inflated occupied voxels
  vec_Vec3d getInflatedCloud();

  /**
   * @brief Get the occupied voxels within a local range
   * @param pos the center of the local point cloud
   * @param dim the range of the local point cloud
   *
   * Assume the map is in a fixed frame
   */
  vec_Vec3d getLocalCloud(const Eigen::Vector3d& pos,
                          const Eigen::Vector3d& ori,
                          const Eigen::Vector3d& dim);

  /**
   * @brief crop a local voxel map from the global voxel map
   * @param ori the origin of the local voxel map
   * @param dim the range of the local voxel map
   *
   * Assume the map is in a fixed frame
   */
  planning_ros_msgs::VoxelMap getInflatedLocalMap(const Eigen::Vector3d& ori,
                                                  const Eigen::Vector3d& dim);

  /**
   * @brief Decay the occupied voxels within a local range
   * @param pos the center of the local point cloud
   * @param max_decay_range maximum range of the local region to decay
   *
   * Assume the map is in a fixed frame
   */
  void decayLocalCloud(const Eigen::Vector3d& pos, double max_decay_range);

  /**
   * @brief Get the inflated occupied voxels within a local range (local voxel
   * map is a subset of global voxel map)
   * @param pos the center of the local point cloud
   * @param dim the range of the local point cloud
   *
   * Assume the map is in a fixed frame
   */
  vec_Vec3d getInflatedLocalCloud(const Eigen::Vector3d& pos,
                                  const Eigen::Vector3d& ori,
                                  const Eigen::Vector3d& dim);

  /// Get the map
  planning_ros_msgs::VoxelMap getMap();
  /// Get the inflated map
  planning_ros_msgs::VoxelMap getInflatedMap();

  /**
   * @brief Get the 2D slice of inflated point cloud
   * @param h the height (z-axis value) to get the slice
   * @param hh the thickness of the slice, zero means one-voxel thick
   */
  planning_ros_msgs::VoxelMap getInflatedOccMap(double h, double hh = 0);

  /**
   * @brief Add point cloud to global map
   * @param pts point cloud in the sensor frame
   * @param TF transform from the sensor frame to the map frame
   * @param ns inflated voxel neighbors
   * @param ray_trace if do ray trace, default disabled
   *
   * return the new added occupied cells
   */
  void addCloud(const vec_Vec3d& pts, const Eigen::Affine3d& TF,
                const vec_Vec3i& ns, bool ray_trace = false,
                double max_range = 10);

  /**
   * @brief Add point cloud
   * @param pts point cloud in the sensor frame
   * @param TF transform from the sensor frame to the map frame
   * @param ns inflated voxel neighbors
   * @param ray_trace if do ray trace, default disabled
   * @param uh upper bound in world z axis
   * @param lh lower bound in world z axis
   *
   * return the new added occupied cells
   */
  void addCloud2D(const vec_Vec3d& pts, const Eigen::Affine3d& TF,
                  const vec_Vec3i& ns, bool ray_trace, double uh, double lh,
                  double max_range);

  /// Free voxels
  void freeVoxels(const Eigen::Vector3d& pt, const vec_Vec3i& ns);
  /// Set corresponding voxels as free
  void freeCloud(const vec_Vec3d& pts, const Eigen::Affine3d& tf);
  // /// Decay the voxel value to unknown with growing time
  // void decay();

 private:
  /// Initialize a space for the map
  bool allocate(const Eigen::Vector3d& new_dim_d,
                const Eigen::Vector3d& new_ori_d);
  /// Raytrace from p1 to p2
  vec_Vec3i rayTrace(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2);
  /// Convert the float point into the int coordinate
  Eigen::Vector3i floatToInt(const Eigen::Vector3d& pt);
  /// Convert the int coordinate into the float point
  Eigen::Vector3d intToFloat(const Eigen::Vector3i& pn);
  /// Check if the coordinate is outside or not
  bool isOutSide(const Eigen::Vector3i& pn);

  /// Map dimension: number of voxels in each axis
  Eigen::Vector3i dim_;
  /// Map origin coordinate
  Eigen::Vector3i origin_;
  /// Map origin point
  Eigen::Vector3d origin_d_;

  Eigen::Affine3d lidar_rot_;

  /// Map resolution
  float res_;
  /// Map object, it is a 3D array
  boost::multi_array<int8_t, 3> map_;
  /// Inflated map object, it is a 3D array
  boost::multi_array<int8_t, 3> inflated_map_;

  /// Value free
  int8_t val_free = 0;
  /// Value occupied
  int8_t val_occ = 100;  // DON'T CHANGE THIS! This value is hard-coded in the
                         // planner. (TODO: remove the hard coding in planner)
  /// Value unknown
  int8_t val_unknown = -1;
  /// Value even
  int8_t val_even = 50;
  /// Value decay (voxels will disappear if unobserved for (val_occ - val_even)
  /// / val_decay times)
  int decay_times_to_empty;
  int8_t val_decay;

  // be careful of overflow (should always be within -128 and 128 range)
  // Add val_add to the voxel whenever a point lies in it. Voxel will be
  // occupied after (val_occ - val_free) / val_add times of such addition.
  int8_t val_add = 20;  // should always be less than 27 to avoid overflow
                        // (should always be within -128 and 128 range)
  /// Default map value
  int8_t val_default = 0;
};

}  // namespace mapper

#ifndef VOXEL_MAPPER_H
#define VOXEL_MAPPER_H

#include <mpl_basis/data_type.h>
#include <planning_ros_msgs/VoxelMap.h>
#include <ros/ros.h>

#include <boost/multi_array.hpp>

class VoxelMapper {
 public:
  /**
   * @brief Simple constructor
   * @param origin the origin of the map (float), should be the left-most corner
   * @param dim the range of map in x-y-z axes (float)
   * @param res voxel resolution
   * @param val set default map value
   */
  VoxelMapper(Vec3f origin, Vec3f dim, decimal_t res, int8_t val = 0);

  /// Set all voxels as unknown
  void setMapUnknown();
  /// Set all voxels as free
  void setMapFree();

  /// Get the occupied voxels
  vec_Vec3f getCloud();
  /// Get the inflated occupied voxels
  vec_Vec3f getInflatedCloud();

  /**
   * @brief Get the occupied voxels within a local range
   * @param pos the center of the local point cloud
   * @param dim the range of the local point cloud
   *
   * Assume the map is in a fixed frame
   */
  vec_Vec3f getLocalCloud(const Vec3f& pos, const Vec3f& ori, const Vec3f& dim);


  /**
   * @brief crop a local voxel map from the global voxel map 
   * @param ori the origin of the local voxel map
   * @param dim the range of the local voxel map
   *
   * Assume the map is in a fixed frame
   */
  planning_ros_msgs::VoxelMap getInflatedLocalMap(const Vec3f& ori, const Vec3f& dim);

  /**
   * @brief Decay the occupied voxels within a local range 
   * @param pos the center of the local point cloud
   * @param max_decay_range maximum range of the local region to decay
   *
   * Assume the map is in a fixed frame
   */
  void decayLocalCloud(const Vec3f& pos, double max_decay_range);

  /**
   * @brief Get the inflated occupied voxels within a local range (local voxel map is a subset of global voxel map)
   * @param pos the center of the local point cloud
   * @param dim the range of the local point cloud
   *
   * Assume the map is in a fixed frame
   */
  vec_Vec3f getInflatedLocalCloud(const Vec3f& pos, const Vec3f& ori,
                                  const Vec3f& dim);

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
  void addCloud(const vec_Vec3f& pts, const Aff3f& TF, const vec_Vec3i& ns,
                bool ray_trace = false, double max_range = 10);
  
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
  void addCloud2D(const vec_Vec3f& pts, const Aff3f& TF, const vec_Vec3i& ns,
                  bool ray_trace, double uh, double lh, double max_range);

  /// Free voxels
  void freeVoxels(const Vec3f& pt, const vec_Vec3i ns);
  /// Set corresponding voxels as free
  void freeCloud(const vec_Vec3f& pts, const Aff3f& TF);
  // /// Decay the voxel value to unknown with growing time
  // void decay();

 private:
  /// Initialize a space for the map
  bool allocate(const Vec3f& new_dim_d, const Vec3f& new_ori_d);
  /// Raytrace from p1 to p2
  vec_Vec3i rayTrace(const Vec3f& pt1, const Vec3f& pt2);
  /// Convert the float point into the int coordinate
  Vec3i floatToInt(const Vec3f& pt);
  /// Convert the int coordinate into the float point
  Vec3f intToFloat(const Vec3i& pn);
  /// Check if the coordinate is outside or not
  bool isOutSide(const Vec3i& pn);

  /// Map dimension: number of voxels in each axis
  Vec3i dim_;
  /// Map origin coordinate
  Vec3i origin_;
  /// Map origin point
  Vec3f origin_d_;

  Aff3f lidar_rot_;

  /// Map resolution
  float res_;
  /// Map object, it is a 3D array
  boost::multi_array<int8_t, 3> map_;
  /// Inflated map object, it is a 3D array
  boost::multi_array<int8_t, 3> inflated_map_;

  /// Value free
  int8_t val_free = 0;
  /// Value occupied
  int8_t val_occ = 100; // DON'T CHANGE THIS! This value is hard-coded in the planner. (TODO: remove the hard coding in planner)
  /// Value unknown
  int8_t val_unknown = -1;
  /// Value even
  int8_t val_even = 50;
  /// Value decay (voxels will disappear if unobserved for (val_occ - val_even)
  /// / val_decay times)
  int8_t val_decay = 0;
  // be careful of overflow (should always be within -128 and 128 range)
  // Add val_add to the voxel whenever a point lies in it. Voxel will be occupied after (val_occ - val_free) / val_add times of such addition.
  int8_t val_add = 20;  // should always be less than 27 to avoid overflow (should always be within
                        // -128 and 128 range)
  /// Default map value
  int8_t val_default = 0;
};

#endif

#pragma once

#include <kr_planning_msgs/VoxelMap.h>

#include <Eigen/Geometry>
#include <boost/multi_array.hpp>
#include <gtest/gtest_prod.h>
#include <mpl_collision/map_util.h>
#include <vector>

namespace mapper {

// Create a derived class to be able to directly access elements of the map
// declared in MapUtil
class VoxelMap : public MPL::VoxelMapUtil {
 public:
  VoxelMap() = default;
  signed char& operator[](int index) { return map_[index]; }
};

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using vec_Vec3d = AlignedVector<Eigen::Vector3d>;
using vec_Vec3i = AlignedVector<Eigen::Vector3i>;

/**
 * @brief The VoxelMapper class contains two maps denoted as map_ and
 * inflated_map_ where inflated_map_ is the inflated versions of map_. These two
 * objects are instances of the mapper::VoxelMap class which is derived from
 * MapUtil. Naturally, the maps share the same origin, dimensions and resolution
 * IMPORTANT note: They use axis-aligned voxels
 */
class VoxelMapper {
  // Making test class and tests friends of VoxelMapper class to be able to
  // access private methods and members
  friend class VoxelMapperTest;
  FRIEND_TEST(VoxelMapperTest, TestAllocateRelocate);

 public:
  /**
   * @brief Simple constructor
   * @param origin The origin of the map (float), should be the most negative
   * corner
   * @param dim The range of map in x-y-z axes (float in world units)
   * @param res Voxel resolution in world units
   * @param default_val The default map value
   * @param decay_times_to_empty Number of times of decay for an occupied voxel
   * to be decayed into empty cell. 0 means no decay
   */
  VoxelMapper(const Eigen::Vector3d& origin,
              const Eigen::Vector3d& dim,
              double res,
              int8_t default_val = 0,
              int decay_times_to_empty = 0);

  /**
   * @brief Set the map with some predefined values 
   * @param ori Origin position in world units (most negative corner)
   * @param dim Number of cells in each dimension
   * @param map Vector of cell values
   * @param res Map resolution
   */
  void setMap(const Eigen::Vector3d &ori, const Eigen::Vector3i &dim,
              const std::vector<signed char> &map, double res);

  /**
   * @brief Set the inflated map with some predefined values 
   * @param ori Origin position in world units (most negative corner)
   * @param dim Number of cells in each dimension
   * @param map Vector of cell values
   * @param res Map resolution
   */
  void setInflatedMap(const Eigen::Vector3d &ori, const Eigen::Vector3i &dim,
                      const std::vector<signed char> &map, double res);

  /**
   * @brief Set all voxels as unknown in both the map and inflated map 
   */
  void setMapUnknown();

  /**
   * @brief Set all voxels as free in both the map and the inflated map
   */
  void setMapFree();

  /**
   * @brief Get the Map object
   * @return kr_planning_msgs::VoxelMap 
   */
  kr_planning_msgs::VoxelMap getMap();

  /**
   * @brief Get the Inflated Map object
   * @return kr_planning_msgs::VoxelMap 
   */
  kr_planning_msgs::VoxelMap getInflatedMap();

  /**
   * @brief Crop a local voxel map from the global inflated voxel map
   * @param ori The origin of the local voxel map (most negative corner)
   * @param dim the range of the local voxel map in world units
   */
  kr_planning_msgs::VoxelMap getInflatedLocalMap(const Eigen::Vector3d& ori,
                                                  const Eigen::Vector3d& dim);

  /**
   * @brief Get a 2D slice of the inflated map
   * @param h The height (z-axis value) to get the slice
   * @param hh The thickness of the slice, zero means one-voxel thick. Thickness
   * refers to how much space (in either direction) along the z-axis is going to
   * be considered for the 2D slice.
   */
  kr_planning_msgs::VoxelMap getInflatedOccMap(double h, double hh = 0);

  /**
   * @brief Add point cloud to map and inflated map
   * @param pts Point cloud in the sensor frame
   * @param TF Transformation representing the pose of the sensor frame with
   * respect to the map frame
   * @param ns Relative neighboring voxels to consider for the inflated map
   * @param ray_trace If do ray trace, default disabled
   */
  void addCloud(const vec_Vec3d& pts,
                const Eigen::Affine3d& TF,
                const vec_Vec3i& ns,
                bool ray_trace = false,
                double max_range = 10);

  /**
   * @brief Get a vector of points that are located at the center of occupied
   * voxels in the map
   */
  vec_Vec3d getCloud();

  /**
   * @brief Get a vector of points that are located at the center of occupied
   * voxels in the inflated map
   */
  vec_Vec3d getInflatedCloud();

  /**
   * @brief Get a vector of points that are located at the center of occupied
   * voxels in the map within a local range
   * @param ori The origin of the local cloud in world units (most negative
   * corner)
   * @param dim the range of the local point cloud in world units
   */
  vec_Vec3d getLocalCloud(const Eigen::Vector3d& ori,
                          const Eigen::Vector3d& dim);

  /**
   * @brief Get a vector of points that are located at the center of occupied
   * voxels in the inflated map within a local range
   * @param ori The origin of the local cloud in world units (most negative
   * corner)
   * @param dim the range of the local point cloud in world units
   */
  vec_Vec3d getInflatedLocalCloud(const Eigen::Vector3d& ori,
                                  const Eigen::Vector3d& dim);

  /**
   * @brief Decay the occupied voxels in the map and the inflated map within a
   * local range
   * @param pos The center of the local point cloud
   * @param max_decay_range Maximum range of the local region to decay
   */
  void decayLocalCloud(const Eigen::Vector3d& pos, double max_decay_range);

  /**
   * @brief Free the voxel in which the provided point is located in as well as
   * the voxel's neighbors. Any voxel in the inflated map (be it a neighbor or
   * not) will not be freed if the corresponding voxel in the map is free.
   * @param pt Point in world units
   * @param ns Relative neighboring voxels
   */
  void freeVoxels(const Eigen::Vector3d& pt, const vec_Vec3i& ns);

  /**
   * @brief This method takes a point cloud and performs ray tracing with every
   * point to free the corresponding voxels if they are previously unknown. This
   * is performed both for the map and the inflated map.
   * @param pts Vector of points to perform ray tracing with
   * @param tf The pose of the frame that the points are relative to 
   */
  void freeCloud(const vec_Vec3d& pts, const Eigen::Affine3d& tf);

 private:
  /**
   * @brief Initialize a space for the map. If this method is called after
   * initialization, then it can rellocate the map and retain the value of any
   * overlapping voxels
   * @param new_dim_d New dimensions in world units
   * @param new_ori_d New origin in world units
   * @return Boolean value where true means that the reallocation occurred
   */
  bool allocate(const Eigen::Vector3d& new_ori_d,
                const Eigen::Vector3d& new_dim_d);

  mapper::VoxelMap map_;            // Map object
  mapper::VoxelMap inflated_map_;   // Inflated map object
  Eigen::Vector3d origin_d_;        // Origin for both maps,most negative corner
  Eigen::Vector3i dim_;             // Number of voxels along each axis
  double res_;                      // Resolution used for both maps

  // Possible voxel values taken from VoxelMap.msg
  int8_t val_free_    = kr_planning_msgs::VoxelMap::val_free;
  int8_t val_occ_     = kr_planning_msgs::VoxelMap::val_occ;
  int8_t val_unknown_ = kr_planning_msgs::VoxelMap::val_unknown;
  int8_t val_even_    = kr_planning_msgs::VoxelMap::val_even;
  int8_t val_default_ = kr_planning_msgs::VoxelMap::val_default;

  // Be careful of overflow (should always be within -128 and 128 range)
  // Add val_add to the voxel whenever a point lies in it. Should always be less
  // than 27 to avoid overflow (should always be within -128 and 128 range)
  int8_t val_add_ = kr_planning_msgs::VoxelMap::val_add;

  // Value decay (voxels will disappear if unobserved for
  // ((val_occ - val_even) / val_decay times)
  int8_t val_decay_;
};

inline void VoxelMapper::setMap(const Eigen::Vector3d &ori,
                                const Eigen::Vector3i &dim,
                                const std::vector<signed char> &map,
                                double res) {
  map_.setMap(ori, dim, map, res);
}

inline void VoxelMapper::setInflatedMap(const Eigen::Vector3d &ori,
                                        const Eigen::Vector3i &dim,
                                        const std::vector<signed char> &map,
                                        double res) {
  inflated_map_.setMap(ori, dim, map, res);
}

}  // namespace mapper

/**
 * @file map_planner.h
 * @brief motion planning using voxel map for collision checking
 */

#ifndef MPL_MAP_PLANNER_H
#define MPL_MAP_PLANNER_H

#include <mpl_planner/common/planner_base.h>
#include <mpl_planner/env/env_map.h>

namespace MPL {

template <int Dim>
using linkedHashMap =
    std::unordered_map<int, std::vector<std::pair<Waypoint<Dim>, int>>>;
/**
 * @brief Motion primitive planner in voxel map
 */
template <int Dim>
class MapPlanner : public PlannerBase<Dim, Waypoint<Dim>> {
 public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug messages
   */
  MapPlanner(bool verbose);
  /// Set map util
  virtual void setMapUtil(const std::shared_ptr<MapUtil<Dim>>& map_util);
  /**
   * @brief Set search region
   * @param path a sequence of waypoints from a path or trajectory
   * @param dense if true, do ray cast between two consecutive points in path
   */
  void setSearchRegion(const vec_Vecf<Dim>& path, bool dense = false);
  /// Set search radius (tunnel radius)
  void setSearchRadius(const Vecf<Dim>& radius);

  /// Set potential radius
  void setPotentialRadius(const Vecf<Dim>& radius);
  /// Set potential map size
  void setPotentialMapRange(const Vecf<Dim>& range);
  /// Set gradient weight
  void setGradientWeight(decimal_t w);
  /// Set potential weight
  void setPotentialWeight(decimal_t w);

  /// Get the potential cloud, works for 2D and 3D
  vec_Vec3f getPotentialCloud(decimal_t h_max = 1.0);
  /// Get the gradient cloud, works for 2D
  vec_Vec3f getGradientCloud(decimal_t h_max = 1.0, int i = 0);

  /// Get search region
  vec_Vecf<Dim> getSearchRegion() const;
  /// Get linked voxels
  vec_Vecf<Dim> getLinkedNodes() const;
  /**
   * @brief Update edge costs according to the new blocked nodes
   * @param pns the new occupied voxels
   *
   * The function returns affected primitives for debug purpose
   */
  void updateBlockedNodes(const vec_Veci<Dim>& pns);
  /**
   * @brief Update edge costs according to the new cleared nodes
   * @param pns the new cleared voxels
   *
   * The function returns affected primitives for debug purpose
   */
  void updateClearedNodes(const vec_Veci<Dim>& pns);

  /**
   * @brief Generate potential map
   * @param pos center of the potential map range is zero, do global generation
   * @param pow power of potential field
   */
  void updatePotentialMap(const Vecf<Dim>& pos);

  /**
   * @brief Iterative trajectory planning with APFs
   * @param start start waypoint
   * @param goal goal waypoint
   * @param raw_traj the trajectory to be perturbed
   * @param max_iter_num number of max iterations, default value is 3
   */
  bool iterativePlan(const Waypoint<Dim>& start, const Waypoint<Dim>& goal,
                     const Trajectory<Dim>& raw_traj, int max_iter_num = 3);

 protected:
  /// Create mask for potential
  void createMask();

  /// Calculate local gradient map
  vec_E<Vecf<Dim>> calculateGradient(const Veci<Dim>& coord1,
                                     const Veci<Dim>& coord2);
  /// Map util
  std::shared_ptr<MapUtil<Dim>> map_util_;

  /// Linked table that records voxel and corresponding primitives passed
  /// through
  mutable linkedHashMap<Dim> lhm_;

  /// Max value for potential
  int8_t H_MAX{100};

  /// Radius of potential for each voxel
  Vecf<Dim> potential_radius_{Vecf<Dim>::Zero()};
  /// Potential map size, centered at the given pos
  Vecf<Dim> potential_map_range_{Vecf<Dim>::Zero()};
  /// Mask for generating potential field around obstacle
  vec_E<std::pair<Veci<Dim>, int8_t>> potential_mask_;
  /// Power of potential value
  decimal_t pow_{1.0};
  /// Gradient map
  vec_E<Vecf<Dim>> gradient_map_;

  /// Search radius (tunnel size)
  Vecf<Dim> search_radius_{Vecf<Dim>::Zero()};
};

/// Planner for 2D OccMap
typedef MapPlanner<2> OccMapPlanner;

/// Planner for 3D VoxelMap
typedef MapPlanner<3> VoxelMapPlanner;
}  // namespace MPL

#endif

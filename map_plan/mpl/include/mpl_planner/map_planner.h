/**
 * @file map_planner.h
 * @brief motion planning using voxel map for collision checking
 */

#pragma once
#include "mpl_planner/env_map.h"
#include "mpl_planner/planner_base.h"

namespace MPL {

template <int Dim>
using linkedHashMap =
    std::unordered_map<int, std::vector<std::pair<Waypoint<Dim>, int>>>;

/**
 * @brief Motion primitive planner in voxel map
 */
template <int Dim>
class MapPlanner final : public PlannerBase<Dim> {
 public:
  MapPlanner(bool verbose);

  /// Set map util
  void setMapUtil(const std::shared_ptr<MapUtil<Dim>>& map_util);

 protected:
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

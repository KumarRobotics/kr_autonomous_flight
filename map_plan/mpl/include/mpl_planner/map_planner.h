/**
 * @file map_planner.h
 * @brief motion planning using voxel map for collision checking
 */

#pragma once

#include "mpl_planner/env_map.h"
#include "mpl_planner/planner_base.h"

namespace MPL {

template <int Dim>
class MapPlanner final : public PlannerBase<Dim> {
 public:
  MapPlanner(bool verbose) : PlannerBase<Dim>(verbose) {}

  /// Set map util
  void setMapUtil(const std::shared_ptr<MapUtil<Dim>>& map_util);
};

/// Planner for 2D OccMap
typedef MapPlanner<2> OccMapPlanner;
/// Planner for 3D VoxelMap
typedef MapPlanner<3> VoxelMapPlanner;

}  // namespace MPL

#pragma once

#include <jps_planner/distance_map_planner/distance_map_planner.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <mapper/data_conversions.h>

template <int Dim>
class PathPlannerUtil {
 public:
  PathPlannerUtil(bool verbose) {
    map_util_ = std::make_shared<JPS::MapUtil<Dim>>();
    jps_util_ = std::make_shared<JPSPlanner<Dim>>(verbose);
    jps_util_->setMapUtil(map_util_);

    distance_map_planner_.reset(new DMPlanner<Dim>(false));
    distance_map_planner_->setMapUtil(map_util_);
    distance_map_planner_->setPow(4);
    distance_map_planner_->setCweight(10);
  }

  vec_Vecf<Dim> getPath() { return path_; }

  std::vector<std::pair<std::string, vec_Vecf<Dim>>> getPathArray() {
    return path_array_;
  }

  vec_Vec3f getCloud() { return distance_map_planner_->getCloud(); }

  vec_Vecf<Dim> getSearchRegion() {
    return distance_map_planner_->getSearchRegion();
  }

  int getPlannerStatus() { return jps_util_->status(); }

  void setMap(const planning_ros_msgs::VoxelMap& map) {
    ::setMap(map_util_, map);
  }

  bool plan(const Vecf<Dim>& start, const Vecf<Dim>& goal) {
    vec_Vecf<Dim> jps_path;
    vec_Vecf<Dim> dist_path;
    // map_util_->freeUnknown();
    jps_util_->updateMap();

    /// get 3d shortest path using jps
    bool solved = jps_util_->plan(start, goal, 1.0, true);
    if (solved) {
    
      jps_path = jps_util_->getPath();
      /// inflate the local map with 0.5m
      Vecf<Dim> potential_radius;
      Vecf<Dim> search_radius;
      potential_radius << 1.0, 1.0, 0.8;
      search_radius << 0.8, 0.8, 0.6;

      distance_map_planner_->setPotentialRadius(
          potential_radius);  // Set 3D potential field radius
      distance_map_planner_->setSearchRadius(
          search_radius);  // Set the valid search region around given path

      distance_map_planner_->computePath(start, goal, jps_path);
      /// get the path from distance map
      dist_path = distance_map_planner_->getPath();
    }

    path_array_.clear();
    path_.clear();

    if (solved) {
      // fix the start point of the path: use the actual start (i.e. robot pose upon calling the planner) instead of the center of the voxel closest to the start, much smoother motion. 
      jps_path[0] = start;
      dist_path[0] = start;

      path_ = dist_path;
      path_array_.push_back(std::make_pair("jps", jps_path));
      path_array_.push_back(std::make_pair("dist", dist_path));
    }

    return solved;
  }

 private:
  std::shared_ptr<JPS::MapUtil<Dim>> map_util_;
  std::shared_ptr<JPSPlanner<Dim>> jps_util_;
  std::shared_ptr<DMPlanner<Dim>> distance_map_planner_;

  vec_Vecf<Dim> path_;
  std::vector<std::pair<std::string, vec_Vecf<Dim>>> path_array_;
};

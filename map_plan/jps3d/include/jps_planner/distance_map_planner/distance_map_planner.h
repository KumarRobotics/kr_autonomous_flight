/**
 * @file distance_map_planner.h
 * @brief distance map planner!
 */
#ifndef DMP_LANNER_H
#define DMP_LANNER_H

#include "graph_search.h"
#include <jps_collision/map_util.h>
#include <jps_basis/data_type.h>

class GraphSearch;
/**
 * @brief Abstract base for planning
 */
template <int Dim> class DMPlanner {
public:
  /**
   * @brief Simple constructor
   * @param verbose enable debug mode
   */
  DMPlanner(bool verbose = false);

  /**
   * @brief set a prior path and get region around it
   * @param path prior path
   * @param r radius in x-y plane
   * @param h height in z axis, if 2d, it's not used
   * @param dense if true, dont need to do rayTrace
   *
   * it returns the inflated region
   */

  std::vector<bool> setPath(const vec_Vecf<Dim> &path, const Vecf<Dim>& radius,
                            bool dense);

  /// Set search radius around a prior path
  void setSearchRadius(const Vecf<Dim>& r);
  /// Set potential radius
  void setPotentialRadius(const Vecf<Dim>& r);
  /// Set the range of potential map, 0 means the whole map
  void setPotentialMapRange(const Vecf<Dim>& r);
  /// Set heuristic weight
  void setEps(double eps);
  /// Set collision cost weight
  void setCweight(double c);
  /// Set the power of potential function \f$H_{MAX}(1 - d/d_{max})^{pow}\f$
  void setPow(int pow);

  /**
   * @brief Status of the planner
   *
   * 0 --- exit normally;
   * -1 --- no path found;
   * 1, 2 --- start or goal is not free.
   */
  int status();
  /// Get the modified path
  vec_Vecf<Dim> getPath();
  /// Get the raw path
  vec_Vecf<Dim> getRawPath();
  /// Get the prior path
  vec_Vecf<Dim> getPriorPath();
  /// Get the nodes in open set
  vec_Vecf<Dim> getOpenSet() const;
  /// Get the nodes in close set
  vec_Vecf<Dim> getCloseSet() const;
  /// Get all the nodes
  vec_Vecf<Dim> getAllSet() const;
  /// Get the potential cloud
  vec_Vec3f getCloud(double h_max = 1);
  /// Get the searching region
  vec_Vecf<Dim> getSearchRegion();
  /// Get the internal map util
  std::shared_ptr<JPS::MapUtil<Dim>> getMapUtil();

  /**
   * @brief Generate distance map
   * @param map_util MapUtil that contains the map object
   * @param pos center of the distance map
   *
   * it copies the map object, thus change the original map_uitl won't affect
   * the internal map.
   */
  void setMap(const std::shared_ptr<JPS::MapUtil<Dim>> &map_util,
              const Vecf<Dim>& pos);

  /// Compute the optimal path
  bool computePath(const Vecf<Dim>& start, const Vecf<Dim>& goal, const vec_Vecf<Dim>& path);
protected:
  /// Create the mask for potential distance field
  vec_E<std::pair<Veci<Dim>, int8_t>> createMask(int pow);
  /// Need to be specified in Child class, main planning function
  bool plan(const Vecf<Dim> &start, const Vecf<Dim> &goal,
            decimal_t eps = 1, decimal_t cweight = 0.1);
  /// remove redundant points on the same line
  vec_Vecf<Dim> removeLinePts(const vec_Vecf<Dim> &path);
  /// Remove some corner waypoints
  vec_Vecf<Dim> removeCornerPts(const vec_Vecf<Dim> &path);
  /// check availability
  bool checkAvailability(const Veci<Dim> &pn);

  /// Assume using 3D voxel map for all 2d and 3d planning
  std::shared_ptr<JPS::MapUtil<Dim>> map_util_;
  /// The planner back-end
  std::shared_ptr<DMP::GraphSearch> graph_search_;
  /// tunnel for visualization
  std::vector<bool> search_region_;
  /// 1-D map array
  std::vector<int8_t> cmap_;

  /// Enabled for printing info
  bool planner_verbose_;
  /// Path cost (raw)
  double path_cost_;
  /// Prior path from planner
  vec_Vecf<Dim> prior_path_;
  /// Raw path from planner
  vec_Vecf<Dim> raw_path_;
  /// Modified path for future usage
  vec_Vecf<Dim> path_;
  /// Flag indicating the success of planning
  int status_ = 0;
  /// max potential value
  int8_t H_MAX{100};
  /// heuristic weight
  double eps_{0.0};
  /// potential weight
  double cweight_{0.1};
  /// radius of distance field
  Vecf<Dim> potential_radius_{Vecf<Dim>::Zero()};
  /// radius of searching tunnel
  Vecf<Dim> search_radius_{Vecf<Dim>::Zero()};
  /// xy range of local distance map
  Vecf<Dim> potential_map_range_{Vecf<Dim>::Zero()};
  /// power index for creating mask
  int pow_{1};

};

/// Planner for 2D OccMap
typedef DMPlanner<2> DMPlanner2D;

/// Planner for 3D VoxelMap
typedef DMPlanner<3> DMPlanner3D;


template <int Dim> class IterativeDMPlanner : public DMPlanner<Dim> {
 public:
  IterativeDMPlanner(bool verbose = false);

  /// Iteratively compute the optimal path
  bool iterativeComputePath(const Vecf<Dim> &start, const Vecf<Dim> &goal,
                            const vec_Vecf<Dim> &path, int max_iteration);
};

/// Iterative Planner for 2D OccMap
typedef IterativeDMPlanner<2> IterativeDMPlanner2D;

/// Iterative Planner for 3D VoxelMap
typedef IterativeDMPlanner<3> IterativeDMPlanner3D;



#endif

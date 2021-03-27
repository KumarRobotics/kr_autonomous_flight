#pragma once
#include <memory>
#include <unordered_map>

#include "mpl_collision/map_util.h"
#include "mpl_planner/env_base.h"

namespace MPL {
/**
 * @brief Voxel map environment
 */
template <int Dim>
class EnvMap : public EnvBase<Dim> {
 public:
  using MapUtilD = MapUtil<Dim>;
  using WaypointD = Waypoint<Dim>;
  using PrimitiveD = Primitive<Dim>;
  using TrajectoryD = Trajectory<Dim>;

  /// Constructor with map util as input
  EnvMap(std::shared_ptr<MapUtilD> map_util) : map_util_(map_util) {}

  /// Check if state hit the goal region, use L-1 norm
  bool is_goal(const WaypointD &state) const;

  /// Check if a point is in free space
  bool is_free(const Vecf<Dim> &pt) const;

  /**
   * @brief Check if the primitive is in free space
   *
   * Sample points along the primitive, and check each point for collision; the
   * number of sampling is calculated based on the maximum velocity and
   * resolution of the map.
   */
  bool is_free(const PrimitiveD &pr) const;

  /**
   * @brief Accumulate the cost along the primitive
   *
   * Sample points along the primitive, and sum up the cost of each point;
   * number of sampling is calculated based on the maximum velocity and
   * resolution of the map.
   *
   * If the potential map has been set, it also uses the potential values;
   * otherwise, the accumulated value will be zero for collision-free primitive
   * and infinity for others.
   */

  decimal_t traverse_primitive(const PrimitiveD &pr) const;

  /**
   * @brief Get successor
   *
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control input
   * for each successor
   *
   * When goal is outside, extra step is needed for finding optimal trajectory.
   * Only return the primitive satisfies valid dynamic constriants (include the
   * one hits obstacles).
   */
  void get_succ(const WaypointD &curr, vec_E<WaypointD> &succ,
                std::vector<decimal_t> &succ_cost,
                std::vector<int> &action_idx) const;

  /// Set gradient map
  void set_gradient_map(const vec_E<Vecf<Dim>> &map) { gradient_map_ = map; }

  /// Set gradient weight
  void set_gradient_weight(decimal_t w) { gradient_weight_ = w; }

  /// Set potential map
  void set_potential_map(const std::vector<int8_t> &map) {
    potential_map_ = map;
  }

  /// Set potential weight
  void set_potential_weight(decimal_t w) { potential_weight_ = w; }

  /// Set prior trajectory
  void set_prior_trajectory(const TrajectoryD &traj);

  /// traverse trajectory
  decimal_t traverse_trajectory(const TrajectoryD &traj) const;

  /// Print out params
  void info();

 protected:
  /// Collision checking util
  std::shared_ptr<MapUtilD> map_util_;
  /// Potential map, optional
  std::vector<int8_t> potential_map_;
  /// Gradient map, optional
  vec_E<Vecf<Dim>> gradient_map_;
  /// Weight of potential value
  decimal_t potential_weight_{0.1};
  /// Weight of gradient value
  decimal_t gradient_weight_{0.0};
};

}  // namespace MPL
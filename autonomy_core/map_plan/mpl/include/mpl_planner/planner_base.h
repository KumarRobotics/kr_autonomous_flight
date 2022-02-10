#pragma once

#include "mpl_planner/env_base.h"
#include "mpl_planner/graph_search.h"

namespace MPL {
/**
 * @brief Motion planning base util class
 */
template <int Dim>
class PlannerBase {
 public:
  using Coord = Waypoint<Dim>;
  using EnvBaseD = EnvBase<Dim>;
  using PrimitiveD = Primitive<Dim>;
  using TrajectoryD = Trajectory<Dim>;
  using StateSpaceD = StateSpace<Dim>;

  /// Simple constructor
  PlannerBase(bool verbose = false) : planner_verbose_(verbose) {}
  virtual ~PlannerBase() = default;

  /// Check if the planner has been initialized
  bool initialized() { return ss_ptr_ != nullptr; }

  /// Get optimal trajectory
  TrajectoryD getTraj() const { return traj_; }
  /// Get points in open set
  vec_Vecf<Dim> getOpenSet() const;
  /// Get points in close set
  vec_Vecf<Dim> getCloseSet() const;
  /// Get expanded nodes, for A* it should be the same as the close set
  vec_Vecf<Dim> getExpandedNodes() const { return env_->expanded_nodes_; }
  /// Get expanded edges, for A* it should be the same as the close set
  vec_E<PrimitiveD> getExpandedEdges() const { return env_->expanded_edges_; }
  /// Get number of expanded nodes
  int getExpandedNum() const { return ss_ptr_->expand_iteration_; }
  /// Check tree validation
  void checkValidation() { ss_ptr_->checkValidation(ss_ptr_->hm_); }

  /// Reset state space
  void reset();

  /// Set max vel in each axis
  void setLPAstar(bool use_lpastar);
  /// Set max vel in x and y axis
  void setVxy(decimal_t v);
  /// Set max vel in z axis
  void setVz(decimal_t vz);
  /// Set max acc in each axis
  void setAmax(decimal_t a);
  /// Set max jerk in each axis
  void setJmax(decimal_t j);
  /// Set max jerk in each axis
  void setYawmax(decimal_t yaw);
  /// Set vertical semi-fov
  void setVfov(decimal_t vfov);
  /// Set max time step to explore
  void setTmax(decimal_t t);
  /// Set dt for each primitive
  void setDt(decimal_t dt);
  /// Set weight for cost in time
  void setW(decimal_t w);
  /// Set weight for cost in time
  void setWyaw(decimal_t w);
  /// Set greedy searching param
  void setEpsilon(decimal_t eps);
  /// Calculate heuristic using dynamics
  void setHeurIgnoreDynamics(bool ignore);
  /// Set max number of expansion
  void setMaxNum(int num);
  /// Set U
  void setU(const vec_E<VecDf>& U);
  /// Set tolerance in geometric and dynamic spaces
  void setTol(decimal_t tol_pos,
              decimal_t tol_vel = -1,
              decimal_t tol_acc = -1);

  /**
   * @brief Planning thread
   * @param start start waypoint
   * @param goal goal waypoint
   *
   * The goal waypoint is the center of the goal region, the planner cannot find
   * the trajectory hits the exact goal state due to discretization
   */
  bool plan(const Coord& start, const Coord& goal);

 protected:
  /// Environment class
  std::shared_ptr<EnvBaseD> env_;
  /// Planner workspace
  std::shared_ptr<StateSpaceD> ss_ptr_;
  /// Optimal trajectory
  TrajectoryD traj_;
  /// Total cost of final trajectory
  decimal_t traj_cost_;
  /// Greedy searching parameter
  decimal_t epsilon_ = 1.0;
  /// Maxmum number of expansion, -1 means no limitation
  int max_num_ = -1;
  /// Enable LPAstar for planning
  bool use_lpastar_ = false;
  /// Enabled to display debug message
  bool planner_verbose_;
};

}  // namespace MPL

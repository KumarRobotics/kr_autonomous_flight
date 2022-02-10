#pragma once

#include <cmath>

#include "mpl_basis/primitive.h"
#include "mpl_basis/trajectory.h"
#include "mpl_basis/waypoint.h"
namespace MPL {

/**
 * @brief Base environment class
 */
template <int Dim>
class EnvBase {
 public:
  using WaypointD = Waypoint<Dim>;
  using PrimitiveD = Primitive<Dim>;
  using TrajectoryD = Trajectory<Dim>;

  /// Simple constructor
  EnvBase() = default;
  virtual ~EnvBase() = default;

  /// Check if state hit the goal region, use L-1 norm
  virtual bool is_goal(const WaypointD& state) const;

  /**
   * @brief Heuristic function
   * @param Waypoint current state coord
   * @param t current state time
   */
  virtual decimal_t get_heur(const WaypointD& state) const;

  /**
   * @brief Heuristic function
   * @param state current state coord
   * @param goal goal state coord
   * @param z_penalty penalty along z-direction, it should be >= 1.0
   */
  virtual decimal_t cal_heur(const WaypointD& state,
                             const WaypointD& goal,
                             const double& z_penalty = 2.0) const;

  /// Replace the original cast function
  Veci<Dim> round(const Vecf<Dim>& vec, decimal_t res) const;

  /// Convert a vec to a string
  //  std::string to_string(const Veci<Dim>& vec) const;

  /// Recover trajectory
  void forward_action(const WaypointD& curr,
                      int action_id,
                      PrimitiveD& pr) const;

  /// Set control input
  void set_u(const vec_E<VecDf>& U) { U_ = U; }

  /// Set max vel in xy axis
  void set_v_xy(decimal_t v) { v_max_ = v; }

  /// Set max vel in z axis
  void set_v_z(decimal_t vz) { v_max_z_ = vz; }

  /// Set max acc in each axis
  void set_a_max(decimal_t a) { a_max_ = a; }

  /// Set max acc in each axis
  void set_j_max(decimal_t j) { j_max_ = j; }

  /// Set max acc in each axis
  void set_yaw_max(decimal_t yaw) { yaw_max_ = yaw; }

  /// Set vertical semi-fov
  void set_vfov(decimal_t vfov) { vfov_ = vfov; }

  /// Set prior trajectory
  virtual void set_prior_trajectory(const Trajectory<Dim>& traj);

  /// Set dt for primitive
  void set_dt(decimal_t dt) { dt_ = dt; }

  /// Set distance tolerance for goal region
  void set_tol_pos(decimal_t pos) { tol_pos_ = pos; }

  /// Set velocity tolerance for goal region
  void set_tol_vel(decimal_t vel) { tol_vel_ = vel; }

  /// Set acceleration tolerance for goal region
  void set_tol_acc(decimal_t acc) { tol_acc_ = acc; }

  /// Set acceleration tolerance for goal region
  void set_tol_yaw(decimal_t yaw) { tol_yaw_ = yaw; }

  /// set weight for cost in time, usually no need to change
  void set_w(decimal_t w) { w_ = w; }

  /// set weight for cost in yaw, usually no need to change
  void set_wyaw(decimal_t wyaw) { wyaw_ = wyaw; }

  /// Set max time
  void set_t_max(int t) { t_max_ = t; }

  /// Set goal state
  bool set_goal(const WaypointD& state);

  /// Set heur_ignore_dynamics
  void set_heur_ignore_dynamics(bool ignore) { heur_ignore_dynamics_ = ignore; }

  /// Print out params
  virtual void info();

  /// Check if a point is in free space
  virtual bool is_free(const Vecf<Dim>& pt) const = 0;

  /// Check if a primitive is in free space
  virtual bool is_free(const Primitive<Dim>& pr) const = 0;

  virtual decimal_t cal_intrinsic_cost(const PrimitiveD& pr) const;

  /// Retrieve dt
  decimal_t get_dt() const { return dt_; }

  /**
   * @brief Get successor
   * @param curr The node to expand
   * @param succ The array stores valid successors
   * @param succ_cost The array stores cost along valid edges
   * @param action_idx The array stores corresponding idx of control for each
   * successor
   */
  virtual void get_succ(const WaypointD& curr,
                        vec_E<WaypointD>& succ,
                        std::vector<decimal_t>& succ_cost,
                        std::vector<int>& action_idx) const = 0;

  /// if enabled, ignore dynamics when calculate heuristic
  bool heur_ignore_dynamics_{true};
  /// weight of time cost
  decimal_t w_{50.0};
  /// weight of yaw
  decimal_t wyaw_{1.0};
  /// tolerance of position for goal region, 0.5 is the default
  decimal_t tol_pos_{0.5};
  /// tolerance of velocity for goal region, 0 means no tolerance
  decimal_t tol_vel_{-1.0};
  /// tolerance of acceleration for goal region, 0 means no tolerance
  decimal_t tol_acc_{-1.0};
  /// tolerance of yaw for goal region, 0 means no tolerance
  decimal_t tol_yaw_{-1.0};
  /// max velocity along x and y
  decimal_t v_max_{-1.0};
  /// max velocity along z
  decimal_t v_max_z_{-1.0};
  /// max acceleration
  decimal_t a_max_{-1.0};
  /// max jerk
  decimal_t j_max_{-1.0};
  /// max yaw
  decimal_t yaw_max_{-1.0};
  /// vertical semi-fov
  decimal_t vfov_{-1.0};
  /// max time
  decimal_t t_max_{std::numeric_limits<decimal_t>::infinity()};
  /// duration of primitive
  decimal_t dt_{1.0};
  /// Array of constant control input
  vec_E<VecDf> U_;
  /// Goal node
  Waypoint<Dim> goal_node_;
  /// Prior trajectory
  vec_E<std::pair<WaypointD, decimal_t>> prior_traj_;
  /// Valid search region (tunnel constraint)
  /// TODO: change to std::vector<uchar> ?
  std::vector<bool> search_region_;
  /// expanded nodes for debug
  mutable vec_Vecf<Dim> expanded_nodes_;
  /// expanded edges for debug
  mutable vec_E<Primitive<Dim>> expanded_edges_;
};
}  // namespace MPL

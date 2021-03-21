/**
 * @file poly_traj.h
 * @brief Trajectory object for PolySolver
 */
#ifndef MPL_POLY_TRAJ_H
#define MPL_POLY_TRAJ_H

#include <mpl_basis/primitive.h>

#include <deque>
#include <memory>

/**
 * @brief Trajectory class for solving n-th polynomial with PolySolver
 */
template <int Dim>
class PolyTraj {
 public:
  /// Simple constructor
  PolyTraj();
  /// Clear
  void clear();
  /// Set waypoints
  void setWaypoints(const vec_E<Waypoint<Dim>>& ws);
  /// Set time allocation
  void setTime(const std::vector<decimal_t>& dts);
  /// Add coefficients
  void addCoeff(const MatDNf<Dim>& coeff);
  /// Convert to Primitive class
  vec_E<Primitive<Dim>> toPrimitives() const;
  /// Evaluate the waypoint at t
  Waypoint<Dim> evaluate(decimal_t t) const;
  /// Get the total time for the trajectory
  decimal_t getTotalTime() const;
  /// Get the p
  MatDNf<Dim> p();

 private:
  std::vector<decimal_t> waypoint_times_;
  std::vector<decimal_t> dts_;
  vec_E<Waypoint<Dim>> waypoints_;
  std::deque<MatDNf<Dim>, Eigen::aligned_allocator<MatDNf<Dim>>> coefficients_;
};

/// PolyTraj in 2D
typedef PolyTraj<2> PolyTraj2D;

/// PolyTraj in 3D
typedef PolyTraj<3> PolyTraj3D;
#endif

/**
 * @file poly_solver.h
 * @brief Trajectory generator from given waypoints
 */
#ifndef MPL_POLY_SOLVER_H
#define MPL_POLY_SOLVER_H

#include <mpl_traj_solver/poly_traj.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/StdVector>

/**
 * @brief Trajectory generator back-end class
 *
 * Given intermediate waypoints and associated time allocation, generate the
 * n-th order polynomials
 */
template <int Dim>
class PolySolver {
 public:
  /**
   * @brief Simple constructor
   * @param smooth_derivative_order The max derivative we want continuous
   * @param minimize_derivative The derivative to minimize
   */
  PolySolver(unsigned int smooth_derivative_order,
             unsigned int minimize_derivative, bool debug = false);
  /**
   * @brief Solve the trajector as defined in constructor
   * @param waypoints Intermediate waypoints that the trajectory pass through
   * @param dts Time allocation for each segment
   *
   * Note that the element in dts is the time for that segment
   */
  bool solve(const vec_E<Waypoint<Dim>>& waypoints,
             const std::vector<decimal_t>& dts);

  /// Get the solved trajectory
  std::shared_ptr<PolyTraj<Dim>> getTrajectory();

 protected:
  /// Number of coefficients of a polynomial
  unsigned int N_;
  /// Order of derivative to optimize
  unsigned int R_;
  MatDf A_;
  MatDf Q_;
  /// Enble output on screen
  bool debug_;
  /// Solved trajectory
  std::shared_ptr<PolyTraj<Dim>> ptraj_;
};

/// PolySolver for 2D
typedef PolySolver<2> PolySolver2D;

/// PolySolver for 3D
typedef PolySolver<3> PolySolver3D;
#endif

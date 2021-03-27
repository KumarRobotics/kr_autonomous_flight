/**
 * @file trajectory.h
 * @brief Trajectory class
 */

#pragma once

#include "mpl_basis/lambda.h"
#include "mpl_basis/primitive.h"

namespace MPL {

/**
 * @brief Command class
 *
 * State includes position, velocity, acceleration and jerk in \f$R^n\f$, where
 * the dimension \f$n\f$ can be either 2 or 3. Yaw and yaw_dot are also
 * contained.
 */
template <int Dim>
struct Command {
  Vecf<Dim> pos;      ///< position in \f$R^{Dim}\f$
  Vecf<Dim> vel;      ///< velocity in \f$R^{Dim}\f$
  Vecf<Dim> acc;      ///< acceleration in \f$R^{Dim}\f$
  Vecf<Dim> jrk;      ///< jerk in \f$R^{Dim}\f$
  decimal_t yaw;      ///< yaw
  decimal_t yaw_dot;  ///< yaw velocity
  decimal_t t;        /// Time \f$t\f$ wrt when evaluate
};

/// Command 2D
typedef Command<2> Command2D;

/// Command 3D
typedef Command<3> Command3D;

/**
 * @brief Trajectory class
 *
 * A trajectory is composed by multiple end-to-end connected primitives,
 * so-called piece-wise polynomials
 */
template <int Dim>
class Trajectory {
 public:
  using CommandD = Command<Dim>;
  using WaypointD = Waypoint<Dim>;
  using PrimitiveD = Primitive<Dim>;

  Trajectory() = default;

  /**
   * @brief Construct from multiple primitives
   */
  Trajectory(const vec_E<PrimitiveD> &prs);

  /**
   * @brief Evaluate Waypoint at time \f$t\f$
   *
   * If t is out of scope, we set t to be the closer bound (0 or total_t_) and
   * return the evaluation; The failure case is when lambda is ill-posed such
   * that \f$t = \lambda(\tau)^{-1}\f$ has no solution, in which a null Waypoint
   * is returned
   */
  WaypointD evaluate(decimal_t time) const;

  /**
   * @brief Evaluate Command at \f$t\f$, return false if fails to evaluate
   *
   * If \f$t\f$ is out of scope, we set \f$t\f$ to be the closer bound (0 or
   * total_t_) and return the evaluation; The failure case is when lambda is
   * ill-posed such that \f$t = \lambda(\tau)^{-1}\f$ has no solution.
   */
  bool evaluate(decimal_t time, CommandD &p) const;

  /**
   * @brief Scale according to ratio at start and end (velocity only)
   */
  bool scale(decimal_t ri, decimal_t rf);

  /**
   * @brief Sample N+1 Command along trajectory using uniformed time
   */
  vec_E<CommandD> sample(int N) const;

  /**
   * @brief Return total efforts of Primitive for the given duration: \f$J(i) =
   * \int_0^T |p^{i}(t)|^2dt\f$
   * @param control Flag that indicates the order of derivative \f$i\f$
   *
   * Return J is the summation of efforts in all three dimensions.
   * `Control::VEL` or `Control::VELxYAW` corresponds to \f$i = 1\f$;
   * `Control::ACC` or `Control::ACCxYAW` corresponds to \f$i = 2\f$;
   * `Control::JRK` or `Control::JRKxYAW` corresponds to \f$i = 3\f$;
   * `Control::SNP` or `Control::SNPxYAW` corresponds to \f$i = 4\f$.
   */
  decimal_t J(const MPL::Control &control) const;

  /**
   * @brief Return total yaw efforts for the given duration: \f$J_{yaw} =
   * \int_0^T |\dot{\phi}(t)|^2dt\f$
   *
   * By default, the `Jyaw` is using `Control::VEL` to calculate.
   */
  decimal_t Jyaw() const;

  /// Get time of each segment
  std::vector<decimal_t> getSegmentTimes() const;

  /// Get intermediate Waypoint on the trajectory
  vec_E<WaypointD> getWaypoints() const;

  /// Get Primitive array
  vec_E<PrimitiveD> getPrimitives() const { return segs; }

  /**
   * @brief Get the scaling factor Lambda
   */
  Lambda lambda() const { return lambda_; }

  /// Get the total duration of Trajectory
  decimal_t getTotalTime() const { return total_t_; }

  /// Segments of primitives
  vec_E<PrimitiveD> segs;
  /// Time in virtual domain
  std::vector<decimal_t> taus;
  /// Time in actual domain
  std::vector<decimal_t> Ts;
  /// Total time of the trajectory
  decimal_t total_t_{0};
  /// Scaling object
  Lambda lambda_;
};

/// Trajectory in 2D
typedef Trajectory<2> Trajectory2D;

/// Trajectory in 3D
typedef Trajectory<3> Trajectory3D;

}  // namespace MPL

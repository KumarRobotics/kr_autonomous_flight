/**
 * @file primitive.h
 * @brief Primitive classes
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>

#include "mpl_basis/data_type.h"
#include "mpl_basis/waypoint.h"

namespace MPL {

/**
 * @brief Primitive1D class
 *
 * Assume the 1D primitive is the n-th order polynomial with n = 5 as
 * \f$p(t) =
 * \frac{c(0)}{120}t^5+\frac{c(1)}{24}t^4+\frac{c(2)}{6}t^3+\frac{c(3)}{2}t^2+c(4)t+c(5)
 * = 0\f$
 */
class Primitive1D {
 public:
  /************************* Constructors *************************/
  /// Empty constructor
  Primitive1D() {}

  /**
   * @brief Construct from known coefficients
   * @param coeff[0] is the coefficient of the highest order
   */
  Primitive1D(const Vec6f& coeff) : c(coeff) {}

  /// Construct 1D primitive from an initial state (p) and an input control (u)
  Primitive1D(decimal_t p, decimal_t u) { c << 0, 0, 0, 0, u, p; }

  /// Construct 1D primitive from an initial state (p, v) and an input control
  /// (u)
  Primitive1D(Vec2f state, decimal_t u) { c << 0, 0, 0, u, state(1), state(0); }

  /// Construct 1D primitive from an initial state (p, v, a) and an input
  /// control (u)
  Primitive1D(Vec3f state, decimal_t u) {
    c << 0, 0, u, state(2), state(1), state(0);
  }

  /// Construct 1D primitive from an initial state (p, v, a, j) and an input
  /// control (u)
  Primitive1D(Vec4f state, decimal_t u) {
    c << 0, u, state(3), state(2), state(1), state(0);
  }

  /// Construct 1D primitive from an initial state (p1) to a goal state (p2),
  /// given duration t
  Primitive1D(decimal_t p1, decimal_t p2, decimal_t t) {
    c << 0, 0, 0, 0, (p2 - p1) / t, p1;
  }

  /// Construct 1D primitive from an initial state (p1, v1) to a goal state (p2,
  /// v2), given duration t
  Primitive1D(
      decimal_t p1, decimal_t v1, decimal_t p2, decimal_t v2, decimal_t t);

  /// Construct 1D primitive from an initial state (p1, v1, a1) to a goal state
  /// (p2, v2, a2), given duration t
  Primitive1D(decimal_t p1,
              decimal_t v1,
              decimal_t a1,
              decimal_t p2,
              decimal_t v2,
              decimal_t a2,
              decimal_t t);

  /*************************** Member functions **************************/
  /**
   * @brief Return total efforts of 1D primitive for the given duration: \f$J(t,
   * i) = \int_0^t |p^{i}(t)|^2dt\f$
   * @param t assume the duration is from 0 to t
   * @param control effort is defined as \f$i\f$-th derivative of polynomial
   */
  decimal_t J(decimal_t t, const MPL::Control& control) const;

  /// Return coffecients
  Vec6f coeff() const { return c; }

  /// Return \f$p\f$ at time \f$t\f$
  decimal_t p(decimal_t t) const;

  /// Return \f$v\f$ at time \f$t\f$
  decimal_t v(decimal_t t) const;

  /// Return \f$a\f$ at time \f$t\f$
  decimal_t a(decimal_t t) const;

  /// Return \f$j\f$ at time \f$t\f$
  decimal_t j(decimal_t t) const { return c(0) / 2 * t * t + c(1) * t + c(2); }

  /**
   * @brief Return vector of time \f$t\f$ for velocity extrema
   *
   * Velocities at both ends (0, t) are not considered
   */
  std::vector<decimal_t> extrema_v(decimal_t t) const;

  /**
   * @brief Return vector of time \f$t\f$ for acceleration extrema
   *
   * Accelerations at both ends (0, t) are not considered
   */
  std::vector<decimal_t> extrema_a(decimal_t t) const;

  /**
   * @brief Return vector of time \f$t\f$ for jerk extrema
   *
   * Jerks at both ends (0, t) are not considered
   */
  std::vector<decimal_t> extrema_j(decimal_t t) const;

 public:
  /// Coefficients
  Vec6f c{Vec6f::Zero()};
};

/**
 * @brief Primitive class
 *
 * Contains \f$n\f$ 1D primitives corresponding to each axis individually.
 */
template <int Dim>
class Primitive {
 public:
  using WaypointD = Waypoint<Dim>;

  Primitive() = default;

  /**
   * @brief Construct from an initial state p and an input control u for a given
   * duration t
   *
   * if the dimension of u is greater than p, use the additional value for yaw
   * control
   */
  Primitive(const WaypointD& p, const VecDf& u, decimal_t t);

  /**
   * @brief Construct from an initial state p1 and a goal state p2 for a given
   * duration t
   */
  Primitive(const WaypointD& p1, const WaypointD& p2, decimal_t t);

  /**
   * @brief Construct from given coefficients and duration\f$t\f$
   *
   * Note: flag `use_xxx` is not set in this constructor
   */
  Primitive(const vec_E<Vec6f>& cs, decimal_t t, MPL::Control control);

  /**
   * @brief Return Waypoint at time \f$t\f$
   *
   * Note: flag `use_xxx` is set in the return value and it is equal
   * to the first given Waypoint
   */
  WaypointD evaluate(decimal_t t) const;

  /**
   * @brief Return duration \f$t\f$
   */
  decimal_t t() const { return t_; }

  /// Get the control indicator
  MPL::Control control() const { return control_; }

  /**
   * @brief Get the 1D primitive
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  Primitive1D pr(int k) const { return prs_[k]; }
  /// Get the yaw primitive
  Primitive1D pr_yaw() const { return pr_yaw_; }

  /**
   * @brief Return max velocity along one axis
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  decimal_t max_vel(int k) const;

  /**
   * @brief Return max accleration along one axis
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  decimal_t max_acc(int k) const;

  /**
   * @brief Return max jerk along k-th dimension
   */
  decimal_t max_jrk(int k) const;

  /**
   * @brief Return total efforts for the given duration
   * @param control effort is defined as \f$i\f$-th derivative of polynomial
   *
   * Return J is the summation of efforts in all three dimensions and
   * \f$J(i) = \int_0^t |p^{i}(t)|^2dt\f$
   */
  decimal_t J(const MPL::Control& control) const;

  /// Return total yaw efforts for the given duration
  decimal_t Jyaw() const { return pr_yaw_.J(t_, MPL::VEL); }

  /**
   * @brief Sample N+1 Waypoints using uniformed time
   */
  vec_E<WaypointD> sample(int N) const;

  /************************** Public members ************************/
  /// Duration
  decimal_t t_;
  /// Control
  MPL::Control control_;
  /// By default, primitive class contains `Dim` 1D primitive
  std::array<Primitive1D, Dim> prs_;
  /// Primitive for yaw
  Primitive1D pr_yaw_;
};

/// Primitive for 2D
typedef Primitive<2> Primitive2D;

/// Primitive for 3D
typedef Primitive<3> Primitive3D;

/************************* Utils ******************************/
/**
 * @brief Check if the max velocity magnitude is within the threshold
 * @param mv is the max threshold for velocity
 * @param ma is the max threshold for acceleration
 * @param mj is the max threshold for jerk
 * @param myaw is the max threshold for yaw
 * @param vfov is sensor vertical semi-fov in rad
 *
 * Use L1 norm for the maximum
 */
template <int Dim>
bool validate_primitive(const Primitive<Dim>& pr,
                        decimal_t mv = 0,
                        decimal_t ma = 0,
                        decimal_t mj = 0,
                        decimal_t myaw = 0,
                        decimal_t vfov = 0) {
  if (pr.control() == MPL::ACC)
    return validate_xxx(pr, mv, MPL::VEL);
  else if (pr.control() == MPL::JRK)
    return validate_xxx(pr, mv, MPL::VEL) && validate_xxx(pr, ma, MPL::ACC) &&
           validate_vel_dir(pr, vfov);
  else if (pr.control() == MPL::SNP)
    return validate_xxx(pr, mv, MPL::VEL) && validate_xxx(pr, ma, MPL::ACC) &&
           validate_xxx(pr, mj, MPL::JRK);
  else if (pr.control() == MPL::VELxYAW)
    return validate_yaw(pr, myaw);
  else if (pr.control() == MPL::ACCxYAW)
    return validate_yaw(pr, myaw) && validate_xxx(pr, mv, MPL::VEL);
  else if (pr.control() == MPL::JRKxYAW)
    return validate_yaw(pr, myaw) && validate_xxx(pr, mv, MPL::VEL) &&
           validate_xxx(pr, ma, MPL::ACC);
  else if (pr.control() == MPL::SNPxYAW)
    return validate_yaw(pr, myaw) && validate_xxx(pr, mv, MPL::VEL) &&
           validate_xxx(pr, ma, MPL::ACC) && validate_xxx(pr, mj, MPL::JRK);
  else
    return true;
}
/**
 * @brief Check if the max velocity magnitude is within the threshold
 * @param mv is the max threshold
 *
 * Use L1 norm for the maximum
 */
template <int Dim>
bool validate_xxx(const Primitive<Dim>& pr, decimal_t max, MPL::Control xxx) {
  if (max <= 0) return true;
  // check if max vel is violating the constraint
  for (int i = 0; i < Dim; i++) {
    if (xxx == MPL::VEL && pr.max_vel(i) > max)
      return false;
    else if (xxx == MPL::ACC && pr.max_acc(i) > max)
      return false;
    else if (xxx == MPL::JRK && pr.max_jrk(i) > max)
      return false;
  }
  return true;
}

/**
 * @brief Check if the successor goes outside of the fov
 * @param my is the value of semi-fov
 *
 */
template <int Dim>
bool validate_yaw(const Primitive<Dim>& pr, decimal_t my) {
  // ignore negative threshold
  if (my <= 0) return true;
  // check velocity angle at two ends, compare with my
  vec_E<Waypoint<Dim>> ws(2);
  ws[0] = pr.evaluate(0);
  ws[1] = pr.evaluate(pr.t());
  for (const auto& w : ws) {
    const auto v = w.vel.template topRows<2>();
    if (v(0) != 0 || v(1) != 0) {  // if v is not zero
      decimal_t d = v.normalized().dot(Vec2f(cos(w.yaw), sin(w.yaw)));
      if (d < cos(my)) return false;
    }
  }
  return true;
}

/**
 * @brief Check if the velocity direction is within sensor vertical FOV
 * @param fov is sensor vertical semi-fov
 *
 */
template <int Dim>
bool validate_vel_dir(const Primitive<Dim>& pr, decimal_t vfov) {
  // ignore non-positive threshold
  if ((vfov <= 0) || (Dim < 3)) {
    return true;
  }

  // clip value to lie in 0 ~ pi/2
  vfov = std::clamp(vfov, 0.00, 1.57);

  // check max vertical velocity angle, compare with vfov
  decimal_t vx_max, vy_max, vz_max;
  vx_max = std::abs(pr.max_vel(0));
  vy_max = std::abs(pr.max_vel(1));
  vz_max = std::abs(pr.max_vel(2));

  decimal_t v_horizontal = sqrt(pow(vx_max, 2) + pow(vy_max, 2));
  if ((vz_max / v_horizontal) <= std::tan(vfov)) {
    return true;
  } else {
    return false;
  }
}

}  // namespace MPL

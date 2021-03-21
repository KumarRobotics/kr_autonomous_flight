/**
 * @file primitive.h
 * @brief Primitive classes
 */

#ifndef MPL_PRIMITIVE_H
#define MPL_PRIMITIVE_H
#include <mpl_basis/data_type.h>
#include <mpl_basis/waypoint.h>

#include "math.h"

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
  Primitive1D(decimal_t p1, decimal_t v1, decimal_t p2, decimal_t v2,
              decimal_t t) {
    Mat4f A;
    A << 0, 0, 0, 1, 0, 0, 1, 0, power(t, 3) / 6, t * t / 2, t, 1, t * t / 2, t,
        1, 0;
    Vec4f b;
    b << p1, v1, p2, v2;
    Vec4f cc = A.inverse() * b;
    c << 0, 0, cc(0), cc(1), cc(2), cc(3);
  }

  /// Construct 1D primitive from an initial state (p1, v1, a1) to a goal state
  /// (p2, v2, a2), given duration t
  Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1, decimal_t p2,
              decimal_t v2, decimal_t a2, decimal_t t) {
    Mat6f A;
    A << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,
        power(t, 5) / 120, power(t, 4) / 24, power(t, 3) / 6, t * t / 2, t, 1,
        power(t, 4) / 24, power(t, 3) / 6, t * t / 2, t, 1, 0, power(t, 3) / 6,
        t * t / 2, t, 1, 0, 0;
    Vec6f b;
    b << p1, v1, a1, p2, v2, a2;
    c = A.inverse() * b;
  }

  /*************************** Member functions **************************/
  /**
   * @brief Return total efforts of 1D primitive for the given duration: \f$J(t,
   * i) = \int_0^t |p^{i}(t)|^2dt\f$
   * @param t assume the duration is from 0 to t
   * @param control effort is defined as \f$i\f$-th derivative of polynomial
   */
  decimal_t J(decimal_t t, const Control::Control& control) const {
    // i = 1, return integration of square of vel
    if (control == Control::VEL || control == Control::VELxYAW)
      return c(0) * c(0) / 5184 * power(t, 9) +
             c(0) * c(1) / 576 * power(t, 8) +
             (c(1) * c(1) / 252 + c(0) * c(2) / 168) * power(t, 7) +
             (c(0) * c(3) / 72 + c(1) * c(2) / 36) * power(t, 6) +
             (c(2) * c(2) / 20 + c(0) * c(4) / 60 + c(1) * c(3) / 15) *
                 power(t, 5) +
             (c(2) * c(3) / 4 + c(1) * c(4) / 12) * power(t, 4) +
             (c(3) * c(3) / 3 + c(2) * c(4) / 3) * power(t, 3) +
             c(3) * c(4) * t * t + c(4) * c(4) * t;
    // i = 2, return integration of square of acc
    else if (control == Control::ACC || control == Control::ACCxYAW)
      return c(0) * c(0) / 252 * power(t, 7) + c(0) * c(1) / 36 * power(t, 6) +
             (c(1) * c(1) / 20 + c(0) * c(2) / 15) * power(t, 5) +
             (c(0) * c(3) / 12 + c(1) * c(2) / 4) * power(t, 4) +
             (c(2) * c(2) / 3 + c(1) * c(3) / 3) * power(t, 3) +
             c(2) * c(3) * t * t + c(3) * c(3) * t;
    // i = 3, return integration of square of jerk
    else if (control == Control::JRK || control == Control::JRKxYAW)
      return c(0) * c(0) / 20 * power(t, 5) + c(0) * c(1) / 4 * power(t, 4) +
             (c(1) * c(1) + c(0) * c(2)) / 3 * power(t, 3) +
             c(1) * c(2) * t * t + c(2) * c(2) * t;
    // i = 4, return integration of square of snap
    else if (control == Control::SNP || control == Control::SNPxYAW)
      return c(0) * c(0) / 3 * power(t, 3) + c(0) * c(1) * t * t +
             c(1) * c(1) * t;
    else
      return 0;
  }

  /// Return coffecients
  Vec6f coeff() const { return c; }

  /// Return \f$p\f$ at time \f$t\f$
  decimal_t p(decimal_t t) const {
    return c(0) / 120 * power(t, 5) + c(1) / 24 * power(t, 4) +
           c(2) / 6 * power(t, 3) + c(3) / 2 * t * t + c(4) * t + c(5);
  }

  /// Return \f$v\f$ at time \f$t\f$
  decimal_t v(decimal_t t) const {
    return c(0) / 24 * power(t, 4) + c(1) / 6 * power(t, 3) + c(2) / 2 * t * t +
           c(3) * t + c(4);
  }

  /// Return \f$a\f$ at time \f$t\f$
  decimal_t a(decimal_t t) const {
    return c(0) / 6 * power(t, 3) + c(1) / 2 * t * t + c(2) * t + c(3);
  }

  /// Return \f$j\f$ at time \f$t\f$
  decimal_t j(decimal_t t) const { return c(0) / 2 * t * t + c(1) * t + c(2); }

  /**
   * @brief Return vector of time \f$t\f$ for velocity extrema
   *
   * Velocities at both ends (0, t) are not considered
   */
  std::vector<decimal_t> extrema_v(decimal_t t) const {
    std::vector<decimal_t> roots = solve(0, c(0) / 6, c(1) / 2, c(2), c(3));
    std::vector<decimal_t> ts;
    for (const auto& it : roots) {
      if (it > 0 && it < t)
        ts.push_back(it);
      else if (it >= t)
        break;
    }
    return ts;
  }

  /**
   * @brief Return vector of time \f$t\f$ for acceleration extrema
   *
   * Accelerations at both ends (0, t) are not considered
   */
  std::vector<decimal_t> extrema_a(decimal_t t) const {
    std::vector<decimal_t> roots = solve(0, 0, c(0) / 2, c(1), c(2));
    std::vector<decimal_t> ts;
    for (const auto& it : roots) {
      if (it > 0 && it < t)
        ts.push_back(it);
      else if (it >= t)
        break;
    }
    return ts;
  }

  /**
   * @brief Return vector of time \f$t\f$ for jerk extrema
   *
   * Jerks at both ends (0, t) are not considered
   */
  std::vector<decimal_t> extrema_j(decimal_t t) const {
    std::vector<decimal_t> ts;
    if (c(0) != 0) {
      decimal_t t_sol = -c(1) * 2 / c(0);
      if (t_sol > 0 && t_sol < t) ts.push_back(t_sol);
    }
    return ts;
  }

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
  /**
   * @brief Empty constructor
   */
  Primitive() {}

  /**
   * @brief Construct from an initial state p and an input control u for a given
   * duration t
   *
   * if the dimension of u is greater than p, use the additional value for yaw
   * control
   */
  Primitive(const Waypoint<Dim>& p, const VecDf& u, decimal_t t)
      : t_(t), control_(p.control) {
    if (control_ == Control::SNP) {
      for (int i = 0; i < Dim; i++) {
        Vec4f vec;
        vec << p.pos(i), p.vel(i), p.acc(i), p.jrk(i);
        prs_[i] = Primitive1D(vec, u(i));
      }
    } else if (control_ == Control::JRK) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
    } else if (control_ == Control::ACC) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
    } else if (control_ == Control::VEL) {
      for (int i = 0; i < Dim; i++) prs_[i] = Primitive1D(p.pos(i), u(i));
    } else if (control_ == Control::SNPxYAW) {
      for (int i = 0; i < Dim; i++) {
        Vec4f vec;
        vec << p.pos(i), p.vel(i), p.acc(i), p.jrk(i);
        prs_[i] = Primitive1D(vec, u(i));
      }
      pr_yaw_ = Primitive1D(p.yaw, u(Dim));
    } else if (control_ == Control::JRKxYAW) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
      pr_yaw_ = Primitive1D(p.yaw, u(Dim));
    } else if (control_ == Control::ACCxYAW) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
      pr_yaw_ = Primitive1D(p.yaw, u(Dim));
    } else if (control_ == Control::VELxYAW) {
      for (int i = 0; i < Dim; i++) prs_[i] = Primitive1D(p.pos(i), u(i));
      pr_yaw_ = Primitive1D(p.yaw, u(Dim));
    } else
      printf("Null Primitive, check the control set-up of the Waypoint!\n");
  }

  /**
   * @brief Construct from an initial state p1 and a goal state p2 for a given
   * duration t
   */
  Primitive(const Waypoint<Dim>& p1, const Waypoint<Dim>& p2, decimal_t t)
      : t_(t), control_(p1.control) {
    // Use jrk control
    if (p1.control == Control::JRK && p2.control == Control::JRK) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i), p2.pos(i),
                              p2.vel(i), p2.acc(i), t_);
    }
    // Use acc control
    else if (p1.control == Control::ACC && p2.control == Control::ACC) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p2.pos(i), p2.vel(i), t_);
    }
    // Use vel control
    else if (p1.control == Control::VEL && p2.control == Control::VEL) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p2.pos(i), t_);
    }
    // Use jrk & yaw control
    else if (p1.control == Control::JRKxYAW && p2.control == Control::JRKxYAW) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i), p2.pos(i),
                              p2.vel(i), p2.acc(i), t_);
      pr_yaw_ = Primitive1D(p1.yaw, p2.yaw, t_);
    }
    // Use acc & yaw control
    else if (p1.control == Control::ACCxYAW && p2.control == Control::ACCxYAW) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p2.pos(i), p2.vel(i), t_);
      pr_yaw_ = Primitive1D(p1.yaw, p2.yaw, t_);
    }
    // Use vel & yaw control
    else if (p1.control == Control::VELxYAW && p2.control == Control::VELxYAW) {
      for (int i = 0; i < Dim; i++)
        prs_[i] = Primitive1D(p1.pos(i), p2.pos(i), t_);
      pr_yaw_ = Primitive1D(p1.yaw, p2.yaw, t_);
    }
    // Null
    else
      printf("Null Primitive, check the control set-up of the Waypoint!\n");
  }

  /**
   * @brief Construct from given coefficients and duration\f$t\f$
   *
   * Note: flag `use_xxx` is not set in this constructor
   */
  Primitive(const vec_E<Vec6f>& cs, decimal_t t, Control::Control control)
      : t_(t), control_(control) {
    for (int i = 0; i < Dim; i++) prs_[i] = Primitive1D(cs[i]);
    if (cs.size() == Dim + 1) pr_yaw_ = Primitive1D(cs[Dim]);
  }

  /**
   * @brief Return Waypoint at time \f$t\f$
   *
   * Note: flag `use_xxx` is set in the return value and it is equal
   * to the first given Waypoint
   */
  Waypoint<Dim> evaluate(decimal_t t) const {
    Waypoint<Dim> p(control_);
    for (int k = 0; k < Dim; k++) {
      p.pos(k) = prs_[k].p(t);
      p.vel(k) = prs_[k].v(t);
      p.acc(k) = prs_[k].a(t);
      p.jrk(k) = prs_[k].j(t);
      if (p.use_yaw) p.yaw = normalize_angle(pr_yaw_.p(t));
    }
    return p;
  }

  /**
   * @brief Return duration \f$t\f$
   */
  decimal_t t() const { return t_; }

  /// Get the control indicator
  Control::Control control() const { return control_; }

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
  decimal_t max_vel(int k) const {
    std::vector<decimal_t> ts = prs_[k].extrema_v(t_);
    decimal_t max_v = std::max(std::abs(prs_[k].v(0)), std::abs(prs_[k].v(t_)));
    for (const auto& it : ts) {
      if (it > 0 && it < t_) {
        decimal_t v = std::abs(prs_[k].v(it));
        max_v = v > max_v ? v : max_v;
      }
    }
    return max_v;
  }

  /**
   * @brief Return max accleration along one axis
   * @param k indicates the corresponding axis: 0-x, 1-y, 2-z
   */
  decimal_t max_acc(int k) const {
    std::vector<decimal_t> ts = prs_[k].extrema_a(t_);
    decimal_t max_a = std::max(std::abs(prs_[k].a(0)), std::abs(prs_[k].a(t_)));
    for (const auto& it : ts) {
      if (it > 0 && it < t_) {
        decimal_t a = std::abs(prs_[k].a(it));
        max_a = a > max_a ? a : max_a;
      }
    }
    return max_a;
  }

  /**
   * @brief Return max jerk along k-th dimension
   */
  decimal_t max_jrk(int k) const {
    std::vector<decimal_t> ts = prs_[k].extrema_j(t_);
    decimal_t max_j = std::max(std::abs(prs_[k].j(0)), std::abs(prs_[k].j(t_)));
    for (const auto& it : ts) {
      if (it > 0 && it < t_) {
        decimal_t j = std::abs(prs_[k].j(it));
        max_j = j > max_j ? j : max_j;
      }
    }
    return max_j;
  }

  /**
   * @brief Return total efforts for the given duration
   * @param control effort is defined as \f$i\f$-th derivative of polynomial
   *
   * Return J is the summation of efforts in all three dimensions and
   * \f$J(i) = \int_0^t |p^{i}(t)|^2dt\f$
   */
  decimal_t J(const Control::Control& control) const {
    decimal_t j = 0;
    for (const auto& pr : prs_) j += pr.J(t_, control);
    return j;
  }

  /// Return total yaw efforts for the given duration
  decimal_t Jyaw() const { return pr_yaw_.J(t_, Control::VEL); }

  /**
   * @brief Sample N+1 Waypoints using uniformed time
   */
  vec_E<Waypoint<Dim>> sample(int N) const {
    vec_E<Waypoint<Dim>> ps(N + 1);
    decimal_t dt = t_ / N;
    for (int i = 0; i <= N; i++) ps[i] = evaluate(i * dt);
    return ps;
  }

  /************************** Public members ************************/
  /// Duration
  decimal_t t_;
  /// Control
  Control::Control control_;
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
 *
 * Use L1 norm for the maximum
 */
template <int Dim>
bool validate_primitive(const Primitive<Dim>& pr, decimal_t mv = 0,
                        decimal_t ma = 0, decimal_t mj = 0,
                        decimal_t myaw = 0) {
  if (pr.control() == Control::ACC)
    return validate_xxx(pr, mv, Control::VEL);
  else if (pr.control() == Control::JRK)
    return validate_xxx(pr, mv, Control::VEL) &&
           validate_xxx(pr, ma, Control::ACC);
  else if (pr.control() == Control::SNP)
    return validate_xxx(pr, mv, Control::VEL) &&
           validate_xxx(pr, ma, Control::ACC) &&
           validate_xxx(pr, mj, Control::JRK);
  else if (pr.control() == Control::VELxYAW)
    return validate_yaw(pr, myaw);
  else if (pr.control() == Control::ACCxYAW)
    return validate_yaw(pr, myaw) && validate_xxx(pr, mv, Control::VEL);
  else if (pr.control() == Control::JRKxYAW)
    return validate_yaw(pr, myaw) && validate_xxx(pr, mv, Control::VEL) &&
           validate_xxx(pr, ma, Control::ACC);
  else if (pr.control() == Control::SNPxYAW)
    return validate_yaw(pr, myaw) && validate_xxx(pr, mv, Control::VEL) &&
           validate_xxx(pr, ma, Control::ACC) &&
           validate_xxx(pr, mj, Control::JRK);
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
bool validate_xxx(const Primitive<Dim>& pr, decimal_t max,
                  Control::Control xxx) {
  if (max <= 0) return true;
  // check if max vel is violating the constraint
  for (int i = 0; i < Dim; i++) {
    if (xxx == Control::VEL && pr.max_vel(i) > max)
      return false;
    else if (xxx == Control::ACC && pr.max_acc(i) > max)
      return false;
    else if (xxx == Control::JRK && pr.max_jrk(i) > max)
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
      /*
      decimal_t vyaw = std::atan2(v(1), v(0));
      decimal_t dyaw = normalize_angle(vyaw - w.yaw);
      if(std::abs(dyaw) > my) // if exceed the threshold
        return false;
        */
      decimal_t d = v.normalized().dot(Vec2f(cos(w.yaw), sin(w.yaw)));
      if (d < cos(my)) return false;
    }
  }
  return true;
}

/// Print all coefficients in primitive p
template <int Dim>
void print(const Primitive<Dim>& p) {
  std::cout << "Primitive: " << std::endl;
  std::cout << "t: " << p.t() << std::endl;
  for (int i = 0; i < Dim; i++)
    std::cout << "dim[" << i << "]:     " << p.pr(i).coeff().transpose()
              << std::endl;
  std::cout << "yaw: " << p.pr_yaw().coeff().transpose() << std::endl;
}

/// Print max dynamic infomation in primitive p
template <int Dim>
void print_max(const Primitive<Dim>& p) {
  Vecf<Dim> max_v, max_a, max_j;
  for (int i = 0; i < Dim; i++) {
    max_v(i) = p.max_vel(i);
    max_a(i) = p.max_acc(i);
    max_j(i) = p.max_jrk(i);
  }
  std::cout << "max_vel: " << max_v.transpose() << std::endl;
  std::cout << "max_acc: " << max_a.transpose() << std::endl;
  std::cout << "max_jrk: " << max_j.transpose() << std::endl;
}

#endif

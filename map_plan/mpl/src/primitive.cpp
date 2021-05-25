#include "mpl_basis/primitive.h"

#include "mpl_basis/math.h"

namespace MPL {

Primitive1D::Primitive1D(decimal_t p1, decimal_t v1, decimal_t p2, decimal_t v2,
                         decimal_t t) {
  Mat4f A;
  A << 0, 0, 0, 1, 0, 0, 1, 0, power(t, 3) / 6, t * t / 2, t, 1, t * t / 2, t,
      1, 0;
  Vec4f b;
  b << p1, v1, p2, v2;
  Vec4f cc = A.inverse() * b;
  c << 0, 0, cc(0), cc(1), cc(2), cc(3);
}

Primitive1D::Primitive1D(decimal_t p1, decimal_t v1, decimal_t a1, decimal_t p2,
                         decimal_t v2, decimal_t a2, decimal_t t) {
  Mat6f A;
  A << 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, power(t, 5) / 120,
      power(t, 4) / 24, power(t, 3) / 6, t * t / 2, t, 1, power(t, 4) / 24,
      power(t, 3) / 6, t * t / 2, t, 1, 0, power(t, 3) / 6, t * t / 2, t, 1, 0,
      0;
  Vec6f b;
  b << p1, v1, a1, p2, v2, a2;
  c = A.inverse() * b;
}

decimal_t Primitive1D::J(decimal_t t, const Control& control) const {
  // i = 1, return integration of square of vel
  if (control == MPL::VEL || control == MPL::VELxYAW)
    return c(0) * c(0) / 5184 * power(t, 9) + c(0) * c(1) / 576 * power(t, 8) +
           (c(1) * c(1) / 252 + c(0) * c(2) / 168) * power(t, 7) +
           (c(0) * c(3) / 72 + c(1) * c(2) / 36) * power(t, 6) +
           (c(2) * c(2) / 20 + c(0) * c(4) / 60 + c(1) * c(3) / 15) *
               power(t, 5) +
           (c(2) * c(3) / 4 + c(1) * c(4) / 12) * power(t, 4) +
           (c(3) * c(3) / 3 + c(2) * c(4) / 3) * power(t, 3) +
           c(3) * c(4) * t * t + c(4) * c(4) * t;
  // i = 2, return integration of square of acc
  else if (control == MPL::ACC || control == MPL::ACCxYAW)
    return c(0) * c(0) / 252 * power(t, 7) + c(0) * c(1) / 36 * power(t, 6) +
           (c(1) * c(1) / 20 + c(0) * c(2) / 15) * power(t, 5) +
           (c(0) * c(3) / 12 + c(1) * c(2) / 4) * power(t, 4) +
           (c(2) * c(2) / 3 + c(1) * c(3) / 3) * power(t, 3) +
           c(2) * c(3) * t * t + c(3) * c(3) * t;
  // i = 3, return integration of square of jerk
  else if (control == MPL::JRK || control == MPL::JRKxYAW)
    return c(0) * c(0) / 20 * power(t, 5) + c(0) * c(1) / 4 * power(t, 4) +
           (c(1) * c(1) + c(0) * c(2)) / 3 * power(t, 3) + c(1) * c(2) * t * t +
           c(2) * c(2) * t;
  // i = 4, return integration of square of snap
  else if (control == MPL::SNP || control == MPL::SNPxYAW)
    return c(0) * c(0) / 3 * power(t, 3) + c(0) * c(1) * t * t +
           c(1) * c(1) * t;
  else
    return 0;
}

decimal_t Primitive1D::p(decimal_t t) const {
  return c(0) / 120 * power(t, 5) + c(1) / 24 * power(t, 4) +
         c(2) / 6 * power(t, 3) + c(3) / 2 * t * t + c(4) * t + c(5);
}

decimal_t Primitive1D::v(decimal_t t) const {
  return c(0) / 24 * power(t, 4) + c(1) / 6 * power(t, 3) + c(2) / 2 * t * t +
         c(3) * t + c(4);
}

decimal_t Primitive1D::a(decimal_t t) const {
  return c(0) / 6 * power(t, 3) + c(1) / 2 * t * t + c(2) * t + c(3);
}

std::vector<decimal_t> Primitive1D::extrema_v(decimal_t t) const {
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

std::vector<decimal_t> Primitive1D::extrema_a(decimal_t t) const {
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

std::vector<decimal_t> Primitive1D::extrema_j(decimal_t t) const {
  std::vector<decimal_t> ts;
  if (c(0) != 0) {
    decimal_t t_sol = -c(1) * 2 / c(0);
    if (t_sol > 0 && t_sol < t) ts.push_back(t_sol);
  }
  return ts;
}

template <int Dim>
MPL::Primitive<Dim>::Primitive(const WaypointD& p, const VecDf& u, decimal_t t)
    : t_(t), control_(p.control) {
  if (control_ == MPL::SNP) {
    for (int i = 0; i < Dim; i++) {
      Vec4f vec;
      vec << p.pos(i), p.vel(i), p.acc(i), p.jrk(i);
      prs_[i] = Primitive1D(vec, u(i));
    }
  } else if (control_ == MPL::JRK) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
  } else if (control_ == MPL::ACC) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
  } else if (control_ == MPL::VEL) {
    for (int i = 0; i < Dim; i++) prs_[i] = Primitive1D(p.pos(i), u(i));
  } else if (control_ == MPL::SNPxYAW) {
    for (int i = 0; i < Dim; i++) {
      Vec4f vec;
      vec << p.pos(i), p.vel(i), p.acc(i), p.jrk(i);
      prs_[i] = Primitive1D(vec, u(i));
    }
    pr_yaw_ = Primitive1D(p.yaw, u(Dim));
  } else if (control_ == MPL::JRKxYAW) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(Vec3f(p.pos(i), p.vel(i), p.acc(i)), u(i));
    pr_yaw_ = Primitive1D(p.yaw, u(Dim));
  } else if (control_ == MPL::ACCxYAW) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(Vec2f(p.pos(i), p.vel(i)), u(i));
    pr_yaw_ = Primitive1D(p.yaw, u(Dim));
  } else if (control_ == MPL::VELxYAW) {
    for (int i = 0; i < Dim; i++) prs_[i] = Primitive1D(p.pos(i), u(i));
    pr_yaw_ = Primitive1D(p.yaw, u(Dim));
  } else
    printf("Null Primitive, check the control set-up of the Waypoint!\n");
}

template <int Dim>
Primitive<Dim>::Primitive(const WaypointD& p1, const WaypointD& p2, decimal_t t)
    : t_(t), control_(p1.control) {
  // Use jrk control
  if (p1.control == MPL::JRK && p2.control == MPL::JRK) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i), p2.pos(i),
                            p2.vel(i), p2.acc(i), t_);
  }
  // Use acc control
  else if (p1.control == MPL::ACC && p2.control == MPL::ACC) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p2.pos(i), p2.vel(i), t_);
  }
  // Use vel control
  else if (p1.control == MPL::VEL && p2.control == MPL::VEL) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p2.pos(i), t_);
  }
  // Use jrk & yaw control
  else if (p1.control == MPL::JRKxYAW && p2.control == MPL::JRKxYAW) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p1.acc(i), p2.pos(i),
                            p2.vel(i), p2.acc(i), t_);
    pr_yaw_ = Primitive1D(p1.yaw, p2.yaw, t_);
  }
  // Use acc & yaw control
  else if (p1.control == MPL::ACCxYAW && p2.control == MPL::ACCxYAW) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p1.vel(i), p2.pos(i), p2.vel(i), t_);
    pr_yaw_ = Primitive1D(p1.yaw, p2.yaw, t_);
  }
  // Use vel & yaw control
  else if (p1.control == MPL::VELxYAW && p2.control == MPL::VELxYAW) {
    for (int i = 0; i < Dim; i++)
      prs_[i] = Primitive1D(p1.pos(i), p2.pos(i), t_);
    pr_yaw_ = Primitive1D(p1.yaw, p2.yaw, t_);
  }
  // Null
  else
    printf("Null Primitive, check the control set-up of the Waypoint!\n");
}

template <int Dim>
Primitive<Dim>::Primitive(const vec_E<Vec6f>& cs, decimal_t t, Control control)
    : t_(t), control_(control) {
  for (int i = 0; i < Dim; i++) prs_[i] = Primitive1D(cs[i]);
  if (cs.size() == Dim + 1) pr_yaw_ = Primitive1D(cs[Dim]);
}

template <int Dim>
Waypoint<Dim> Primitive<Dim>::evaluate(decimal_t t) const {
  WaypointD p(control_);
  for (int k = 0; k < Dim; k++) {
    p.pos(k) = prs_[k].p(t);
    p.vel(k) = prs_[k].v(t);
    p.acc(k) = prs_[k].a(t);
    p.jrk(k) = prs_[k].j(t);
    if (p.use_yaw) p.yaw = normalize_angle(pr_yaw_.p(t));
  }
  return p;
}

template <int Dim>
decimal_t Primitive<Dim>::max_vel(int k) const {
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

template <int Dim>
decimal_t Primitive<Dim>::max_acc(int k) const {
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

template <int Dim>
decimal_t Primitive<Dim>::max_jrk(int k) const {
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

template <int Dim>
decimal_t Primitive<Dim>::J(const Control& control) const {
  decimal_t j = 0;
  for (const auto& pr : prs_) j += pr.J(t_, control);
  return j;
}

template <int Dim>
vec_E<Waypoint<Dim>> Primitive<Dim>::sample(int N) const {
  vec_E<WaypointD> ps(N + 1);
  decimal_t dt = t_ / N;
  for (int i = 0; i <= N; i++) ps[i] = evaluate(i * dt);
  return ps;
}

// Explicit instantiations
template class Primitive<2>;
template class Primitive<3>;

}  // namespace MPL

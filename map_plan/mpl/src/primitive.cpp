#include "mpl_basis/primitive.h"

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

}  // namespace MPL

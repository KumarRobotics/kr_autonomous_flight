#include "mpl_basis/lambda.h"

#include "mpl_basis/math.h"

namespace MPL {

LambdaSeg::LambdaSeg(const VirtualPoint &v1, const VirtualPoint &v2) {
  Mat4f A;
  A << power(v1.t, 3), v1.t * v1.t, v1.t, 1, 3 * v1.t * v1.t, 2 * v1.t, 1, 0,
      power(v2.t, 3), v2.t * v2.t, v2.t, 1, 3 * v2.t * v2.t, 2 * v2.t, 1, 0;
  Vec4f b;
  b << v1.p, v1.v, v2.p, v2.v;

  a = A.inverse() * b;

  if (std::abs(a(0)) < 1e-5) a(0) = 0;
  if (std::abs(a(1)) < 1e-5) a(1) = 0;
  if (std::abs(a(2)) < 1e-5) a(2) = 0;
  if (std::abs(a(3)) < 1e-5) a(3) = 0;

  ti = v1.t;
  tf = v2.t;

  dT = getT(tf) - getT(ti);
}

VirtualPoint LambdaSeg::evaluate(decimal_t tau) const {
  VirtualPoint vt;
  vt.t = tau;
  vt.p = a(0) * power(tau, 3) + a(1) * tau * tau + a(2) * tau + a(3);
  vt.v = 3 * a(0) * tau * tau + 2 * a(1) * tau + a(2);
  return vt;
}

decimal_t LambdaSeg::getT(decimal_t t) const {
  return a(0) / 4 * power(t, 4) + a(1) / 3 * power(t, 3) + a(2) / 2 * t * t +
         a(3) * t;
}

Lambda::Lambda(const std::vector<VirtualPoint> &vs) {
  for (int i = 0; i < (int)vs.size() - 1; i++) {
    LambdaSeg seg(vs[i], vs[i + 1]);
    segs.push_back(seg);
  }
}

std::vector<VirtualPoint> Lambda::sample(int N) {
  // sample N points
  std::vector<VirtualPoint> vs;
  if (segs.empty()) return vs;
  decimal_t ti = segs.front().ti;
  decimal_t tf = segs.back().tf;
  decimal_t dt = (tf - ti) / N;
  for (decimal_t t = ti; t <= tf; t += dt) {
    for (const auto &seg : segs) {
      if (t >= seg.ti && t < seg.tf) {
        vs.push_back(seg.evaluate(t));
        break;
      }
    }
  }

  return vs;
}

vec_Vec3f Lambda::sampleT(int N) {
  vec_Vec3f ts;
  decimal_t ti = segs.front().ti;
  decimal_t tf = segs.back().tf;
  decimal_t dt = (tf - ti) / N;

  for (decimal_t t = ti; t <= tf; t += dt) {
    ts.push_back(Vec3f(t, getT(t), 0));
  }

  return ts;
}

VirtualPoint Lambda::evaluate(decimal_t tau) const {
  VirtualPoint vt;
  for (const auto &seg : segs) {
    if (tau >= seg.ti && tau < seg.tf) {
      vt = seg.evaluate(tau);
      break;
    }
  }
  return vt;
}

decimal_t Lambda::getT(decimal_t tau) const {
  if (segs.empty()) return tau;
  decimal_t T = 0;
  for (const auto &seg : segs) {
    if (tau >= seg.ti && tau <= seg.tf) {
      T += seg.getT(tau) - seg.getT(seg.ti);
      return T;
    }
    T += seg.dT;
  }
  return T;
}

decimal_t Lambda::getTau(decimal_t t) const {
  if (!exist()) return t;
  decimal_t T = 0;
  for (const auto &seg : segs) {
    if (t >= T && t <= T + seg.dT) {
      decimal_t a = seg.a(0) / 4;
      decimal_t b = seg.a(1) / 3;
      decimal_t c = seg.a(2) / 2;
      decimal_t d = seg.a(3);
      decimal_t e = T - t - seg.getT(seg.ti);

      std::vector<decimal_t> ts = solve(a, b, c, d, e);
      for (const auto &it : ts) {
        if (it >= seg.ti && it <= seg.tf) return it;
      }
    }
    T += seg.dT;
  }

  printf("error: cannot find tau, t = %f\n", t);
  return -1;
}

decimal_t Lambda::getTotalTime() const {
  decimal_t t = 0;
  for (const auto &seg : segs) t += seg.dT;

  return t;
}

}  // namespace MPL

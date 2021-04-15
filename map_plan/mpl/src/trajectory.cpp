#include "mpl_basis/trajectory.h"

#include "mpl_basis/math.h"

namespace MPL {

template <int Dim>
Trajectory<Dim>::Trajectory(const vec_E<PrimitiveD> &prs) : segs(prs) {
  taus.push_back(0);
  for (const auto &pr : prs) taus.push_back(pr.t() + taus.back());
  Ts = taus;
  total_t_ = taus.back();
}

template <int Dim>
typename Trajectory<Dim>::WaypointD Trajectory<Dim>::evaluate(
    decimal_t time) const {
  decimal_t tau = lambda_.getTau(time);
  if (tau < 0) tau = 0;
  if (tau > total_t_) tau = total_t_;

  for (size_t id = 0; id < segs.size(); id++) {
    if ((tau >= taus[id] && tau < taus[id + 1]) || id == segs.size() - 1) {
      tau -= taus[id];
      Waypoint<Dim> p(segs[id].control());
      for (int j = 0; j < Dim; j++) {
        const auto pr = segs[id].pr(j);
        p.pos(j) = pr.p(tau);
        p.vel(j) = pr.v(tau);
        p.acc(j) = pr.a(tau);
        p.jrk(j) = pr.j(tau);
        p.yaw = normalize_angle(segs[id].pr_yaw().p(tau));
      }
      return p;
    }
  }

  printf("cannot find tau according to time: %f\n", time);
  return Waypoint<Dim>();
}

template <int Dim>
bool Trajectory<Dim>::evaluate(decimal_t time, CommandD &p) const {
  decimal_t tau = lambda_.getTau(time);
  if (tau < 0) tau = 0;
  if (tau > total_t_) tau = total_t_;

  decimal_t lambda = 1;
  decimal_t lambda_dot = 0;

  if (lambda_.exist()) {
    VirtualPoint vt = lambda_.evaluate(tau);
    lambda = vt.p;
    lambda_dot = vt.v;
  }

  for (size_t id = 0; id < segs.size(); id++) {
    if (tau >= taus[id] && tau <= taus[id + 1]) {
      tau -= taus[id];
      for (int j = 0; j < Dim; j++) {
        const auto pr = segs[id].pr(j);
        p.pos(j) = pr.p(tau);
        p.vel(j) = pr.v(tau) / lambda;
        p.acc(j) = pr.a(tau) / lambda / lambda -
                   p.vel(j) * lambda_dot / lambda / lambda / lambda;
        p.jrk(j) = pr.j(tau) / lambda / lambda -
                   3 / power(lambda, 3) * p.acc(j) * p.acc(j) * lambda_dot +
                   3 / power(lambda, 4) * p.vel(j) * lambda_dot * lambda_dot;
        p.yaw = normalize_angle(segs[id].pr_yaw().p(tau));
        p.yaw_dot = normalize_angle(segs[id].pr_yaw().v(tau));
        p.t = time;
      }
      return true;
    }
  }

  printf("cannot find tau according to time: %f\n", time);
  return false;
}

template <int Dim>
bool Trajectory<Dim>::scale(decimal_t ri, decimal_t rf) {
  std::vector<VirtualPoint> vs;
  VirtualPoint vi, vf;
  vi.p = 1.0 / ri;
  vi.v = 0;
  vi.t = 0;

  vf.p = 1.0 / rf;
  vf.v = 0;
  vf.t = taus.back();

  vs.push_back(vi);
  vs.push_back(vf);
  Lambda ls(vs);
  lambda_ = ls;
  std::vector<decimal_t> ts;
  for (const auto &tau : taus) ts.push_back(lambda_.getT(tau));
  Ts = ts;
  total_t_ = Ts.back();
  return true;
}

template <int Dim>
vec_E<typename Trajectory<Dim>::CommandD> Trajectory<Dim>::sample(int N) const {
  vec_E<CommandD> ps(N + 1);

  decimal_t dt = total_t_ / N;
  for (int i = 0; i <= N; i++) evaluate(i * dt, ps[i]);

  return ps;
}

template <int Dim>
decimal_t Trajectory<Dim>::J(const Control &control) const {
  decimal_t j = 0;
  for (const auto &seg : segs) j += seg.J(control);
  return j;
}

template <int Dim>
decimal_t Trajectory<Dim>::Jyaw() const {
  decimal_t j = 0;
  for (const auto &seg : segs) j += seg.Jyaw();
  return j;
}

template <int Dim>
std::vector<decimal_t> Trajectory<Dim>::getSegmentTimes() const {
  std::vector<decimal_t> dts;
  for (int i = 0; i < (int)Ts.size() - 1; i++) dts.push_back(Ts[i + 1] - Ts[i]);
  return dts;
}

template <int Dim>
vec_E<typename Trajectory<Dim>::WaypointD> Trajectory<Dim>::getWaypoints()
    const {
  vec_E<WaypointD> ws;
  if (segs.empty()) return ws;
  decimal_t t = 0;
  for (const auto &seg : segs) {
    ws.push_back(seg.evaluate(0));
    ws.back().t = t;
    t += seg.t();
  }
  ws.push_back(segs.back().evaluate(segs.back().t()));
  ws.back().t = t;
  return ws;
}

// explicit instantiation
template class Trajectory<2>;
template class Trajectory<3>;

}  // namespace MPL

#include <mpl_traj_solver/poly_traj.h>

namespace MPL {

template <int Dim>
PolyTraj<Dim>::PolyTraj() {}

template <int Dim>
Waypoint<Dim> PolyTraj<Dim>::evaluate(decimal_t time) const {
  if (time < 0)
    time = 0;
  else if (time > waypoint_times_.back())
    time = waypoint_times_.back();

  unsigned int cur_idx = 0;
  for (unsigned int i = 1; i < waypoint_times_.size(); i++) {
    if (time <= waypoint_times_[i]) {
      cur_idx = i - 1;
      break;
    }
  }

  const decimal_t t_traj = time - waypoint_times_[cur_idx];
  const MatDNf<Dim> &p = coefficients_[cur_idx];
  Waypoint<Dim> waypoint;
  for (int derr = 0; derr < Dim; derr++) {
    Vecf<Dim> vec = Vecf<Dim>::Zero();
    for (unsigned int i = derr; i < p.rows(); i++) {
      unsigned int c = 1;
      for (int j = 0; j < derr; j++) c *= (i - j);
      vec += p.row(i).transpose() * (c * std::pow(t_traj, i - derr));
    }
    if (derr == 0)
      waypoint.pos = vec;
    else if (derr == 1)
      waypoint.vel = vec;
    else if (derr == 2)
      waypoint.acc = vec;
  }

  return waypoint;
}

template <int Dim>
decimal_t PolyTraj<Dim>::getTotalTime() const {
  return waypoint_times_.back();
}

template <int Dim>
void PolyTraj<Dim>::clear() {
  waypoint_times_.clear();
  coefficients_.clear();
}

template <int Dim>
void PolyTraj<Dim>::addCoeff(const MatDNf<Dim> &coeff) {
  coefficients_.push_back(coeff);
}

template <int Dim>
void PolyTraj<Dim>::setWaypoints(const vec_E<Waypoint<Dim>> &ws) {
  waypoints_ = ws;
}

template <int Dim>
void PolyTraj<Dim>::setTime(const std::vector<decimal_t> &dts) {
  waypoint_times_.clear();
  waypoint_times_.push_back(0);
  for (auto t : dts) waypoint_times_.push_back(waypoint_times_.back() + t);
  dts_ = dts;
}

template <int Dim>
vec_E<Primitive<Dim>> PolyTraj<Dim>::toPrimitives() const {
  vec_E<Primitive<Dim>> trajs;
  trajs.resize(coefficients_.size());
  for (unsigned int i = 0; i < coefficients_.size(); i++) {
    vec_E<Vec6f> coeffs;
    const MatDNf<Dim> &p = coefficients_[i];
    for (unsigned int j = 0; j < p.cols(); j++) {
      Vec6f coeff(Vec6f::Zero());
      for (unsigned int k = 0; k < p.rows(); k++)
        coeff(k) = p(k, j) * factorial(k);
      coeffs.push_back(coeff.reverse());
    }

    trajs[i] = Primitive<Dim>(coeffs, dts_[i], waypoints_.front().control);
  }
  return trajs;
}

template <int Dim>
MatDNf<Dim> PolyTraj<Dim>::p() {
  MatDNf<Dim> p;
  if (coefficients_.empty()) return p;
  unsigned int N = coefficients_.front().rows();
  p = MatDNf<Dim>(N * coefficients_.size(), Dim);
  int i = 0;
  for (const auto &it : coefficients_) {
    p.block(i * N, 0, N, Dim) = it;
    i++;
  }
  return p;
}

template class PolyTraj<1>;

template class PolyTraj<2>;

template class PolyTraj<3>;

}  // namespace MPL

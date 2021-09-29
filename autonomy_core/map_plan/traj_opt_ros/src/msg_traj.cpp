// Copyright 2015 Michael Watterson
#include "traj_opt_ros/msg_traj.h"

#include <boost/shared_ptr.hpp>

namespace traj_opt {

MsgTrajectory::MsgTrajectory(const TrajData &traj) : traj_(traj) {
  num_secs_ = traj_.data.front().segments;
  dim_ = traj_.dimensions;

  for (auto &poly : traj_.data.front().segs) dts.push_back(poly.dt);
  exec_t = traj.data.front().t_total;

  deg_ = traj_.data.front().segs.front().degree;

  polyies_.reserve(num_secs_);
  // unpack coefficents
  //  auto it = traj.coefficents.begin();
  for (uint i = 0; i < num_secs_; i++) {
    std::vector<boost::shared_ptr<Poly>> polys;
    polys.clear();
    polys.reserve(dim_);
    for (int j = 0; j < dim_; j++) {
      polys.push_back(boost::make_shared<Poly>(
          traj.data.at(j).segs.at(i).coeffs.data(), deg_));
    }
    polyies_.push_back(polys);
  }
  // take derrivatives
  derrives_.push_back(polyies_);
  for (int i = 0; i < 3; i++) {
    for (auto &p : polyies_)
      for (auto &q : p)
        q = boost::make_shared<Poly>(PolyCalculus::differentiate(*q));

    derrives_.push_back(polyies_);
  }
}

bool MsgTrajectory::evaluate(double t, uint derr,
                             VecD &out) const {  // returns false when out
  out = VecD::Zero(dim_, 1);
  //  out << 0.0,0.0,0.0,0.0;
  bool success = false;

  double dt, dx;

  // of time range, but still
  // sets out to endpoint
  std::vector<boost::shared_ptr<Poly>> const *poly;
  if (t < 0) {
    poly = &derrives_.at(derr).front();
    dt = dts.front();
    dx = 0.0;
    success = false;
  } else {
    // find appropriate section
    auto dt_it = dts.begin();
    for (auto &it : derrives_.at(derr)) {
      if (t < *dt_it) {
        poly = &(it);
        dt = *dt_it;
        dx = t / (*dt_it);
        success = true;
        break;
      }
      t -= *dt_it;

      ++dt_it;
    }
    if (!success) {
      poly = &derrives_.at(derr).back();
      dt = dts.back();
      dx = 1.0;
    }
    success = false;
  }
  double ratio = std::pow(1 / dt, double(derr));
  for (int i = 0; i < dim_; i++) out(i) = ratio * poly->at(i)->evaluate(dx);

  return success;
}

double MsgTrajectory::getTotalTime() const {
  double tt = 0.0;
  for (auto &t : dts) tt += t;
  return tt;
}

TrajData MsgTrajectory::serialize() { return traj_; }
double MsgTrajectory::getCost() { return NAN; }
bool MsgTrajectory::evaluateS(double t, VecD &out) {
  assert(dim_ == 9);
  out = VecD::Zero(6, 1);
  VecD r5;
  evaluate(t, 0, r5);
  Quat q(r5(5), r5(6), r5(7), r5(8));
  Mat3 chart = q.matrix();

  Vec3 xi = r5(3) * chart * Vec3::UnitY() + r5(4) * chart * Vec3::UnitZ() -
            chart * Vec3::UnitX();
  Vec3 P =
      2.0 / (r5(3) * r5(3) + r5(4) * r5(4) + 1.0) * xi + chart * Vec3::UnitX();
  out.block<3, 1>(0, 0) = r5.block<3, 1>(0, 0);
  out.block<3, 1>(3, 0) = P;

  //  std::cout << "Chart of trajm " << q.w() << ", "<< q.x() << ", "<< q.y() <<
  //  ", "<< q.z()  << std::endl;
  return true;
}
bool MsgTrajectory::evaluateST(double t, VecD &out) {
  // plots in tangent space
  assert(dim_ == 9);
  out = VecD::Zero(6, 1);
  VecD r5;
  evaluate(t, 0, r5);
  Quat q(r5(5), r5(6), r5(7), r5(8));
  Mat3 chart = q.matrix();

  Vec3 xi(r5(3), r5(4), t);
  out.block<3, 1>(0, 0) = xi;
  out.block<3, 1>(3, 0) = chart * Vec3::UnitX();

  return true;
}

}  // namespace traj_opt

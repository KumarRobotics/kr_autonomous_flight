// Copyright 2015 Michael Watterson
#include "traj_opt_ros/msg_traj.h"

#include <boost/shared_ptr.hpp>

namespace traj_opt {

MsgTrajectory::MsgTrajectory(const TrajData &traj) : traj_(traj) {
  // TODO(laura) fails if each dimension of the traj has a different number of
  // segments

  num_segs_ = traj_.data.front().segments;
  dim_ = traj_.dimensions;

  for (int dim = 0; dim < dim_; dim++) {
    std::vector<double> dt;
    for (auto &poly : traj_.data.at(dim).segs) dt.push_back(poly.dt);
    dts_.push_back(dt);
  }

  exec_t = traj.data.front().t_total;

  deg_ = traj_.data.front().segs.front().degree;
  polys_.reserve(num_segs_);
  // unpack coefficients
  for (uint i = 0; i < num_segs_; i++) {
    std::vector<boost::shared_ptr<Poly>> polys;
    polys.clear();
    polys.reserve(dim_);
    for (int j = 0; j < dim_; j++) {
      polys.push_back(boost::make_shared<Poly>(
          traj.data.at(j).segs.at(i).coeffs.data(), deg_));
    }
    polys_.push_back(polys);
  }
  // take derivatives
  derivatives_.push_back(polys_);
  for (int i = 0; i < 3; i++) {
    for (auto &p : polys_)
      for (auto &q : p)
        q = boost::make_shared<Poly>(PolyCalculus::differentiate(*q));
    derivatives_.push_back(polys_);
  }
}

bool MsgTrajectory::evaluate(double t, uint deriv,
                             VecD &out) {  // returns false when out
  out = VecD::Zero(dim_, 1);
  //  out << 0.0,0.0,0.0,0.0;
  bool success = false;

  double dt, dx;

  // of time range, but still
  // sets out to endpoint
  std::vector<boost::shared_ptr<Poly>> const *poly;
  int success_counter = 0;
  for (int dim = 0; dim < dim_; dim++) {
    if (t < 0) {
      poly = &derivatives_.at(deriv).front();
      dt = dts_.at(dim).front();
      dx = 0.0;
    } else {
      // find appropriate section
      auto dt_it = dts_.at(dim).begin();
      for (auto &it : derivatives_.at(deriv)) {
        if (t < *dt_it) {
          poly = &(it);
          dt = *dt_it;
          dx = t / (*dt_it);
          success_counter++;
          break;
        }
        t -= *dt_it;

        ++dt_it;
      }
      if (success_counter < dim + 1) {
        poly = &derivatives_.at(deriv).back();
        dt = dts_.at(dim).back();
        dx = 1.0;
      }
    }
    double ratio = std::pow(1 / dt, double(deriv));
    out(dim) = ratio * poly->at(dim)->evaluate(dx);
  }
  if (success_counter == dim_) success = true;

  return success;
}

double MsgTrajectory::getTotalTime() const {
  double tt = 0.0;
  for (auto &t : dts_[0]) tt += t;
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

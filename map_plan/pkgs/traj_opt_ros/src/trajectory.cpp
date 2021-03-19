// Copyright 2015 Michael Watterson
#include "traj_opt_ros/trajectory.h"

namespace traj_opt {

double Trajectory::getExecuteTime() const {
  if (exec_t <= 0)
    return getTotalTime();
  else
    return exec_t;
}

bool Trajectory::getCommand(double t, uint num_derivatives, MatD &data) {
  // check input
  if (dim_ < 1) return false;

  // allocate data
  data = MatD::Zero(dim_, num_derivatives + 1);
  // evaluate and pack
  for (uint i = 0; i <= num_derivatives; i++) {
    VecD block;
    this->evaluate(t, i, block);
    data.block(0, i, dim_, 1) = block;
  }
  return true;
}

bool Trajectory::getHopfChart(double t) {
  VecD val;
  evaluate(t, 0, val);
  if (val.rows() < 5) return true;

  return val(4) < 0.5;
}

std::pair<double, double> Trajectory::getYaws(double t) {
  VecD pos, vel;
  evaluate(t, 0, pos);
  evaluate(t, 1, vel);

  if (pos.rows() < 4)
    return std::make_pair(0.0, 0.0);
  else
    return std::make_pair(pos(3), vel(3));
}
std::pair<Vec3, Vec3> Trajectory::getXi(double t, double g) {
  VecD gv = VecD::Zero(3, 1);
  gv(2) = g;
  VecD evalv, evald;
  evaluate(t, 2, evalv);
  evaluate(t, 3, evald);
  VecD eval3 = evalv.block<3, 1>(0, 0);
  VecD evald3 = evald.block<3, 1>(0, 0);

  VecD xi = eval3 + gv;
  xi.normalize();
  // calc differential
  VecD xid = xi.dot(xi) * evald3;
  xid -= xi.dot(evald3) * xi;
  xid /= std::pow(xi.norm(), 3.0);

  Vec3 xi3, xid3;
  xi3 = eval3.head(3) + gv.head(3);
  xid3 = evald3.head(3);
  return std::make_pair(xi3, xid);
}

}  // namespace traj_opt

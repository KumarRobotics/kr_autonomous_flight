// Copyright 2015 Michael Watterson
#pragma once

#include <utility>  // pair

#include "traj_opt_ros/traj_data.h"
#include "traj_opt_ros/types.h"

namespace traj_opt {

class Trajectory {
 public:
  virtual ~Trajectory() {}
  virtual bool evaluate(double t, uint derr, VecD &out) const = 0;
  virtual double getTotalTime() const = 0;
  virtual double getCost() = 0;
  // execute time
  double getExecuteTime() const;
  std::pair<Vec3, Vec3> getXi(double t, double g = 9.81);
  std::pair<double, double> getYaws(double t);
  bool getHopfChart(double t);

  // returns a matrix (dim X num_derivatives + 1) of the trajectory evalutated
  // at time t
  bool getCommand(double t, uint num_derivatives, MatD &data);
  void setDim(uint ndim) { dim_ = ndim; }
  void setExecuteTime(double t) { exec_t = t; }

  int getDim() { return dim_; }
  virtual TrajData serialize() = 0;

 protected:
  double exec_t{-1.0};  // duration of execute time
  int dim_{0};
};

}  // namespace traj_opt

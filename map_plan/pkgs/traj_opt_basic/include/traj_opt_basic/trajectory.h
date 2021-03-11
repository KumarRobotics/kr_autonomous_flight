// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_BASIC_TRAJECTORY_H_
#define TRAJ_OPT_BASIC_TRAJECTORY_H_

#include "traj_opt_basic/traj_data.h"
#include "traj_opt_basic/types.h"

namespace traj_opt {

class Trajectory {
 public:
  virtual ~Trajectory() {}
  virtual bool evaluate(decimal_t t, uint derr, VecD &out) const = 0;
  virtual decimal_t getTotalTime() const = 0;
  virtual decimal_t getCost() = 0;
  // execute time
  decimal_t getExecuteTime() const;
  std::pair<Vec3, Vec3> getXi(decimal_t t, decimal_t g = 9.81);
  std::pair<decimal_t, decimal_t> getYaws(decimal_t t);
  bool getHopfChart(decimal_t t);

  // returns a matrix (dim X num_derivatives + 1) of the trajectory evalutated
  // at time t
  bool getCommand(decimal_t t, uint num_derivatives, MatD &data);

  void setDim(uint ndim) { dim_ = ndim; }
  void setExecuteTime(decimal_t t) { exec_t = t; }

  int getDim() { return dim_; }
  virtual TrajData serialize() = 0;

 protected:
  decimal_t exec_t{-1.0};  // duration of execute time
  int dim_{0};
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_TRAJECTORY_H_

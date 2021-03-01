// Copyright 2015 Michael Watterson
#ifndef MAP_PLAN_PKGS_TRAJ_OPT_BASIC_TRAJECTORY_H_
#define MAP_PLAN_PKGS_TRAJ_OPT_BASIC_TRAJECTORY_H_

// package includes
#include <traj_opt_basic/traj_data.h>
#include <traj_opt_basic/types.h>

// library includes
#include <boost/smart_ptr/shared_ptr.hpp>

// STL includes
#include <iostream>
#include <vector>

namespace traj_opt {

class Trajectory {
 public:
  virtual bool evaluate(decimal_t t, uint derr, VecD *out) const = 0;
  virtual decimal_t getTotalTime() const = 0;
  virtual decimal_t getCost() = 0;
  // execute time
  decimal_t getExecuteTime() const;

  // returns a matrix (dim X num_derivatives + 1) of the trajectory evalutated
  // at time t
  bool getCommand(decimal_t t, uint num_derivatives, MatD *data);

  void setDim(uint ndim) { dim_ = ndim; }
  uint getDim() { return dim_; }
  void setExecuteTime(decimal_t t) { exec_t = t; }

  virtual TrajData serialize() = 0;

 protected:
  decimal_t exec_t{-1.0};  // duration of execute time
  uint dim_{0};
};
}  // namespace traj_opt
#endif  // MAP_PLAN_PKGS_TRAJ_OPT_BASIC_TRAJECTORY_H_

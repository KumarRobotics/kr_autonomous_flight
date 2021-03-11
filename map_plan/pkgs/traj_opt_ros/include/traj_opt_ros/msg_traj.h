// Copyright 2015 Michael Watterson
#pragma once
#include <vector>

#include "traj_opt_ros/polynomial_basis.h"
#include "traj_opt_ros/traj_data.h"
#include "traj_opt_ros/trajectory.h"

namespace traj_opt {

class MsgTrajectory : public Trajectory {
 public:
  explicit MsgTrajectory(const TrajData &traj);

  bool evaluate(decimal_t t, uint derr, VecD &out) const override;
  //  bool evaluate(decimal_t t, VecD &out);
  bool evaluateS(decimal_t t, VecD &out);
  bool evaluateST(decimal_t t, VecD &out);
  decimal_t getTotalTime() const override;
  decimal_t getCost() override;
  TrajData serialize() override;

 protected:
  TrajData traj_;
  std::vector<std::vector<boost::shared_ptr<Poly>>> polyies_;
  std::vector<std::vector<std::vector<boost::shared_ptr<Poly>>>> derrives_;
  uint num_secs_;
  std::vector<decimal_t> dts;
  uint deg_;
  //  int seg;
};
}  // namespace traj_opt

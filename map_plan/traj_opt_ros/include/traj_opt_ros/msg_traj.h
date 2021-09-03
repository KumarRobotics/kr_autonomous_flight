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

  bool evaluate(double t, uint derr, VecD &out) override;
  //  bool evaluate(double t, VecD &out);
  bool evaluateS(double t, VecD &out);
  bool evaluateST(double t, VecD &out);
  double getTotalTime() const override;
  double getCost() override;
  TrajData serialize() override;

 protected:
  TrajData traj_;
  std::vector<std::vector<boost::shared_ptr<Poly>>> polys_;
  std::vector<std::vector<std::vector<boost::shared_ptr<Poly>>>> derivatives_;
  uint num_segs_;
  std::vector<std::vector<double>> dts_;
  uint deg_;
};

}  // namespace traj_opt

// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_BASIC_MSG_TRAJ_H_
#define TRAJ_OPT_BASIC_MSG_TRAJ_H_

#include <traj_opt_basic/polynomial_basis.h>
#include <traj_opt_basic/traj_data.h>
#include <traj_opt_basic/trajectory.h>
#include <vector>

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
#endif  // TRAJ_OPT_BASIC_MSG_TRAJ_H_

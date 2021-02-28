// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_PRO_TRAJECTORY_SOLVER_H_
#define TRAJ_OPT_PRO_TRAJECTORY_SOLVER_H_

#include <traj_opt_basic/trajectory.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <utility>
#include <vector>

namespace traj_opt {

struct Waypoint {
  int knot_id{0};  // 0: first, 1: second, -1: last -2: second to last, etc
  VecD pos{VecD::Zero(4, 1)};
  VecD vel{VecD::Zero(4, 1)};
  VecD acc{VecD::Zero(4, 1)};
  VecD jrk{VecD::Zero(4, 1)};
  bool use_pos{false};
  bool use_vel{false};
  bool use_acc{false};
  bool use_jrk{false};

  // stack constraint into matrix form
  std::pair<Eigen::VectorXi, MatD> getIndexForm() const;
  void setDefaultIC(uint dim, int id = 0) {
    pos = VecD::Zero(dim, 1);
    vel = VecD::Zero(dim, 1);
    acc = VecD::Zero(dim, 1);
    jrk = VecD::Zero(dim, 1);
    use_pos = true;
    use_vel = true;
    use_acc = true;
    use_jrk = true;
    knot_id = id;
  }
  void use_all() {
    use_pos = true;
    use_vel = true;
    use_acc = true;
    use_jrk = true;
  }
};

struct WaypointHopf : public Waypoint {
  VecD xi_hat{VecD::Zero(3, 1)};  // direction of acceleration
  int chart{0};                   // 0 hover, 1 inverted
  static Waypoint cast(const WaypointHopf &val) {
    Waypoint w;
    w.pos = val.pos;
    w.vel = val.vel;
    w.acc = val.acc;
    w.jrk = val.jrk;
    w.use_pos = val.use_pos;
    w.use_vel = val.use_vel;
    w.use_acc = val.use_acc;
    w.use_jrk = val.use_jrk;
    w.knot_id = val.knot_id;
    return w;
  }
};

class TrajectorySolver {
 public:
  TrajectorySolver();
  virtual bool solveTrajectory(
      const std::vector<Waypoint> &waypnts, const std::vector<MatD> &A,
      const std::vector<VecD> &b, const std::vector<decimal_t> &ds,
      decimal_t epsilon = 0,
      boost::shared_ptr<Vec3Vec> points = boost::shared_ptr<Vec3Vec>(),
      decimal_t upsilon = 0);

  virtual bool trajectoryStatus();
  virtual boost::shared_ptr<Trajectory> getTrajectory();

  virtual bool adjustTimes(decimal_t epsilon);
  virtual void setParams(decimal_t v_max, decimal_t a_max, decimal_t j_max,
                         int time_its, decimal_t time_eps);

  virtual bool checkMax(decimal_t r) { return false; }

 protected:
  std::vector<Waypoint> waypoints_;
  bool trajectory_solved_;
  boost::shared_ptr<Trajectory> traj_;

  // parameters
  decimal_t v_max_, a_max_, j_max_, time_eps_;
  int time_its_;
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_TRAJECTORY_SOLVER_H_

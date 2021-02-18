// Copyright 2015 Michael Watterson
#ifndef TRAJ_OPT_PRO_GUROBI_SOLVER_H_
#define TRAJ_OPT_PRO_GUROBI_SOLVER_H_

#include <traj_opt_pro/gurobi_trajectory.h>

#include <vector>

class GRBEnv;

namespace traj_opt {

class GurobiSolver : public TrajectorySolver {
 public:
  GurobiSolver();
  bool solveTrajectory(
      const std::vector<Waypoint> &waypnts, const std::vector<MatD> &A,
      const std::vector<VecD> &b, const std::vector<decimal_t> &ds,
      decimal_t epsilon = 0,
      boost::shared_ptr<Vec3Vec> points = boost::shared_ptr<Vec3Vec>(),
      decimal_t upsilon = 0);
  boost::shared_ptr<Trajectory> getTrajectory();
  void addPathCost(const std::vector<MatD> &A, const std::vector<VecD> &b,
                   double epsilon);
  bool solveConvexifyed(const std::vector<Waypoint> &waypnts, const MatD &Aobs,
                        const VecD &bobs, const MatD &Aenv, const VecD &benv,
                        const std::vector<decimal_t> &ds);
  void seedDs(boost::shared_ptr<Vec3Vec> points, std::vector<decimal_t> *dsp);
  void sikangSeed(boost::shared_ptr<Vec3Vec> points,
                  std::vector<decimal_t> *dsp, decimal_t v0,
                  decimal_t ratio = 1.0);
  bool use_lp_{false};
  void setPolyParams(int degree, PolyType type, int order);
  void setConParam(ConstraintMode mode, int num) {
    con_mode_ = mode;
    num_sample_ = num;
  }
  decimal_t getCost() { return cost_; }

  bool checkMax(decimal_t r);

 private:
  boost::shared_ptr<LegendreTrajectory> ltraj_;
  decimal_t cost_;
  // gurobi stuff
  boost::shared_ptr<GRBEnv> grb_env_;
  bool initializeGurobi();

  void addLineCost(boost::shared_ptr<Vec3Vec> points, double upsilon);
  int degree_, order_;
  PolyType polytype_;
  ConstraintMode con_mode_;
  int num_sample_;
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_GUROBI_SOLVER_H_

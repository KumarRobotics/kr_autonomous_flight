#pragma once

#include <traj_opt_pro/nonlinear_solver.h>
#include <traj_opt_pro/nonlinear_trajectory.h>
#include <traj_opt_pro/weights.h>

namespace traj_opt {

// class InformationCost : public CostFunction {
// public:
//  PolyCost(const NLTraj &traj, const NLTimes &times, BasisBundlePro &basis,
//           int min_dim);
//  decimal_t evaluate();
//  ETV gradient();
//  ETV hessian();

// private:
//  std::vector<SymbolicPoly> poly;

// public:
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//};

class NestedExpression;

class SphereCost : public CostFunction {
 public:
  // only pass s2 part of traj here
  SphereCost(const NLTraj &traj, const NLTimes &times, BasisBundlePro &basis,
             int min_dim,
             const std::vector<Quat, Eigen::aligned_allocator<Quat> > &charts);
  decimal_t evaluate() override;
  ETV gradient() override;
  ETV hessian() override;

  void testGrad();

 private:
  // for stereographic coordinates this returns the x_k in the numerator or -1
  std::pair<int, decimal_t> christoffel_symbol(int i, int j, int k);
  std::vector<RationalPoly> cost_parts;
  boost::shared_ptr<NestedExpression> s2cost;
  SymbolicPoly poly;

  NLTraj traj_;
  NLTimes times_;
  int segs_;
  int deg_;
  // for calculation, we have cost in the form:
  // p1 + p2/den^2 + p2/den^4;
  MatD cost_v_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cost_n_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace traj_opt

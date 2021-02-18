#include <traj_opt_pro/information_cost.h>
#include <traj_opt_pro/nested.h>
#include <traj_opt_pro/nonlinear_trajectory.h>
#include <traj_opt_pro/timers.h>

namespace traj_opt {
std::pair<int, decimal_t> SphereCost::christoffel_symbol(int i, int j, int k) {
  if (i == k) {
    return std::make_pair(j, -2.0);
  } else if (j == k) {
    return std::make_pair(i, -2.0);
  } else if (i == j) {
    return std::make_pair(k, 2.0);
  } else {
    return std::make_pair(0, 0.0);
  }
}

// store stuff as R3 part then S2 part
SphereCost::SphereCost(
    const NLTraj &traj, const NLTimes &times, BasisBundlePro &basis,
    int min_dim,
    const std::vector<Quat, Eigen::aligned_allocator<Quat>> &charts)
    : traj_(traj), times_(times) {
  // get r3 part
  segs_ = traj.front().size();
  deg_ = traj.front().front().size() - 1;

  cost_n_ = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Zero(8, 8);
  cost_v_ = MatD::Zero(8, 8);

  // LOL, I guess I should document these. In the mean time, enjoy decyphering
  // them :P
  if (min_dim == 3) {
    cost_n_ << -5, -5, -4, -4, -3, -3, -2, -2, -5, -5, -4, -4, -3, -3, -2, -2,
        -4, -4, -3, -3, -2, -2, -1, -1, -4, -4, -3, -3, -2, -2, -1, -1, -3, -3,
        -2, -2, -1, -1, 0, 0, -3, -3, -2, -2, -1, -1, 0, 0, -2, -2, -1, -1, 0,
        0, 1, 1, -2, -2, -1, -1, 0, 0, 1, 1;
    cost_v_ << 1.120000000000000, -1.120000000000000, 0.560000000000000,
        0.560000000000000, 0.100000000000000, -0.100000000000000,
        0.003333333333333, 0.003333333333333, -1.120000000000000,
        1.120000000000000, -0.560000000000000, -0.560000000000000,
        -0.100000000000000, 0.100000000000000, -0.003333333333333,
        -0.003333333333333, 0.560000000000000, -0.560000000000000,
        0.297142857142857, 0.262857142857143, 0.058571428571429,
        -0.041428571428571, 0.002095238095238, 0.001238095238095,
        0.560000000000000, -0.560000000000000, 0.262857142857143,
        0.297142857142857, 0.041428571428571, -0.058571428571429,
        0.001238095238095, 0.002095238095238, 0.100000000000000,
        -0.100000000000000, 0.058571428571429, 0.041428571428571,
        0.014285714285714, -0.005714285714286, 0.000547619047619,
        0.000119047619048, -0.100000000000000, 0.100000000000000,
        -0.041428571428571, -0.058571428571429, -0.005714285714286,
        0.014285714285714, -0.000119047619048, -0.000547619047619,
        0.003333333333333, -0.003333333333333, 0.002095238095238,
        0.001238095238095, 0.000547619047619, -0.000119047619048,
        0.000063492063492, -0.000007936507937, 0.003333333333333,
        -0.003333333333333, 0.001238095238095, 0.002095238095238,
        0.000119047619048, -0.000547619047619, -0.000007936507937,
        0.000063492063492;
    cost_v_ *= 1000.0;
  }

  //  for (int d = 0; d < 3; d++) {
  //    for (int s = 0; s < segs_; s++) {
  //      for (int i = 0; i <= deg_; i++) {
  //        for (int j = 0; j <= i; j++) {
  //          if (i == j)
  //            poly += SymbolicPoly(traj.at(d).at(s).at(i),
  //            traj.at(d).at(s).at(j),
  //                                 times.at(s), cost_n_(i, j), cost_v_(i, j));
  //          else  // add symetric elements properly
  //            poly +=
  //                SymbolicPoly(traj.at(d).at(s).at(i), traj.at(d).at(s).at(j),
  //                             times.at(s), cost_n_(i, j), 2.0 * cost_v_(i,
  //                             j));
  //        }
  //      }
  //    }
  //  }

  //  std::vector<Quat, Eigen::aligned_allocator<Quat> > charts =
  //  std::vector<Quat, Eigen::aligned_allocator<Quat>
  //  >(segs_,Quat::Identity());

  // get s2 part
  int n = 11;

  for (int k = 0; k < n; k++) {
    decimal_t wi = QuadWeights::getWeight(k, n);
    decimal_t pi = QuadWeights::getPoint(k, n);
    std::cout << "pi: " << pi << " wi: " << wi << std::endl;
  }

  std::vector<boost::shared_ptr<NestedExpression>> total_parts;
  std::vector<decimal_t> weights;

  for (int j = 0; j < segs_; j++) {
    // get monomial expressions
    std::vector<RationalPoly> coe1,
        coe2;  // expressions for first and second variables on sphere
    // chart transition
    Quat dchart;
    if (j > 0) {
      dchart = charts.at(j).inverse() * charts.at(j - 1);
    }
    for (int i = 0; i <= deg_; i++) {
      if (j > 0 && i == 0) {
        //              ScopedTimer tm("phiphi");
        auto phiphi = S2R3Traj::getPhiPhi(dchart, traj.at(0).at(j - 1).at(1),
                                          traj.at(1).at(j - 1).at(1));
        coe1.push_back(phiphi.front());
        coe2.push_back(phiphi.back());
      } else if (j > 0 && i == 2) {
        //              ScopedTimer tm("dphiphi");
        auto dphiphi = S2R3Traj::getdPhiPhi(
            dchart, traj.at(0).at(j - 1).at(1), traj.at(1).at(j - 1).at(1),
            traj.at(0).at(j - 1).at(3), traj.at(1).at(j - 1).at(3));
        coe1.push_back(dphiphi.front());
        coe2.push_back(dphiphi.back());
      } else {
        //              ScopedTimer tm("trivial");
        coe1.push_back(RationalPoly(SymbolicPoly(
            SymbolicPoly::PolyArbitrary(
                1, SymbolicPoly::PolyPair(traj.at(0).at(j).at(i), 1)),
            1.0)));
        coe2.push_back(RationalPoly(SymbolicPoly(
            SymbolicPoly::PolyArbitrary(
                1, SymbolicPoly::PolyPair(traj.at(1).at(j).at(i), 1)),
            1.0)));
      }
    }
    std::vector<boost::shared_ptr<NestedExpression>> sphere_parts;
    for (auto &co : coe1)
      sphere_parts.push_back(boost::make_shared<RationalNested>(co));
    for (auto &co : coe2)
      sphere_parts.push_back(boost::make_shared<RationalNested>(co));

    decimal_t dt = times.at(j)->getVal();
    for (int k = 0; k < n; k++) {
      // std::cout << "j, k: " << j << " , " << k << std::endl;
      decimal_t wi = QuadWeights::getWeight(k, n);
      decimal_t pi = QuadWeights::getPoint(k, n);
      weights.push_back(wi);
      total_parts.push_back(
          boost::make_shared<NestedS2CostPart>(sphere_parts, dt * pi, dt));
    }
  }
  s2cost = boost::make_shared<NestedSum>(total_parts, weights);

  //  for(auto &wi:weights)
  //    std::cout << "w: " << wi << std::endl;

  evaluate();
}
decimal_t SphereCost::evaluate() {
  //  decimal_t result=poly.evaluate();
  //  std::cout << "r3 cost "  << result << std::endl;
  //  decimal_t r3 = result;
  decimal_t result = s2cost->evaluate();
  //  std::cout << "s2 cost "  << result -r3 << std::endl;
  return result;
}
ETV SphereCost::gradient() {
  ETV result = s2cost->gradient();
  //  for(auto &p:cost_parts) {
  //    ETV grad = NonlinearSolver::transpose(p.gradient2(0));
  //    result.insert(result.end(),grad.begin(),grad.end());
  //    //        std::cout << "cost part size " << grad.size() << std::endl;
  //    //        for(auto &pi:grad)
  //    //          std::cout << "sphere gradient " << pi.row() << " " <<
  //    pi.col() << " " << pi.value() << std::endl;
  //  }
  return result;
}
ETV SphereCost::hessian() {
  ETV result = poly.quad_hessian();
  for (auto &p : cost_parts) {
    ETV hess = p.hessian2();
    result.insert(result.end(), hess.begin(), hess.end());
    //        for(auto &pi:hess)
    //          std::cout << "sphere hess " << pi.row() << " " << pi.col() << "
    //          " << pi.value() << std::endl;
    //        std::cout << "cost hess size " << hess.size() << std::endl;
  }
  return result;
}

S2R3Traj::S2R3Traj(
    const std::vector<std::pair<MatD, VecD>> &cons, Vec5 testn, Vec5 testf,
    const std::vector<Quat, Eigen::aligned_allocator<Quat>> &chartss,
    const std::vector<decimal_t> &ds) {
  auto endpoint_basis = boost::make_shared<EndPointBasis>(7);
  auto bezier_basis = boost::make_shared<BezierBasis>(7);

  //  {
  //    ScopedTimer tm("Problem Formulation");

  basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);

  //    std::vector<decimal_t> ds(cons.size(),1.0);

  seg_ = ds.size();
  deg_ = 7;
  dim_ = 5;

  charts = chartss;

  allocate_poly(testn, testf, ds);
  add_Axb(cons);

  auto cost = boost::make_shared<PolyCost>(traj, times, basis, 3, 0, 3);
  solver.setCost(cost);
  //    add_Axb(cons, true);

  solved_ = solver.solve(true, 1e-10);  // solve r3
  scost = boost::make_shared<SphereCost>(trajs2, times, basis, 3, charts);
  solvers2.setCost(scost);

  // call solver
  //  }
  //      solved_ = solver.solve(true);

  solved_ &= solvers2.solve_nlopt();  // solve
}
void S2R3Traj::allocate_poly(const Vec5 &pn, const Vec5 &pf,
                             const std::vector<decimal_t> &ds) {
  // assumes deg = 7 and cont = 3
  traj.clear();
  NonlinearSolver *solver_ref;
  for (int i = 0; i < 5; i++) {
    if (i < 3)
      solver_ref = &solver;
    else
      solver_ref = &solvers2;
    NLSpline spline;
    for (int j = 0; j < seg_; j++) {
      NLPoly pi;
      for (int k = 0; k <= deg_; k++) {
        if (j == 0 && k == 0) {
          pi.push_back(solver_ref->addConstVar(pn(i)));
        } else if (j == 0 && k % 2 == 0) {
          pi.push_back(solver_ref->addConstVar(0.0));
        } else if (j == (seg_ - 1) && k == 1) {
          pi.push_back(solver_ref->addConstVar(pf(i)));
        } else if (j == (seg_ - 1) && k % 2 == 1) {
          pi.push_back(solver_ref->addConstVar(0.0));
        } else if (j > 0 && k % 2 == 0) {
          pi.push_back(spline.at(j - 1).at(
              k + 1));  // this is only valid for assumptions
        } else {
          pi.push_back(solver_ref->addVar(0.25));
        }
      }
      spline.push_back(pi);
    }
    if (i < 3)
      traj.push_back(spline);
    else
      trajs2.push_back(spline);
  }
  for (int j = 0; j < seg_; j++) {
    times.push_back(solver.addConstVar(ds.at(j)));
  }
}
//    bool S2R3Traj::evaluate(decimal_t t, uint derr, VecD &out) const {
//      // TODO, fix this
//      return false;
//    }
#include "constraints.cpp"
void S2R3Traj::add_Axb(const std::vector<std::pair<MatD, VecD>> &cons,
                       bool r3) {
  std::vector<Variable *> svars;
  for (int s = 0; s < cons.size(); s++) {
    for (int i = 0; i <= deg_; i++) {
      if (!trajs2.at(0).at(s).at(i)->isConstant())
        svars.push_back(trajs2.at(0).at(s).at(i));
      if (!trajs2.at(1).at(s).at(i)->isConstant())
        svars.push_back(trajs2.at(1).at(s).at(i));
    }
  }

  int j = 0;
  for (auto &pair : cons) {
    assert(pair.first.rows() == pair.second.rows());

    std::vector<RationalPoly> coe1,
        coe2;  // expressions for first and second variables on sphere
    Quat dchart;
    if (j > 0) {
      dchart = charts.at(j).inverse() * charts.at(j - 1);
    }

    for (int i = 0; i <= deg_; i++) {
      if (j > 0 && i == 0) {
        //              ScopedTimer tm("phiphi");
        auto phiphi = S2R3Traj::getPhiPhi(dchart, trajs2.at(0).at(j - 1).at(1),
                                          trajs2.at(1).at(j - 1).at(1));
        coe1.push_back(phiphi.front());
        coe2.push_back(phiphi.back());
      } else if (j > 0 && i == 2) {
        //              ScopedTimer tm("dphiphi");
        auto dphiphi = S2R3Traj::getdPhiPhi(
            dchart, trajs2.at(0).at(j - 1).at(1), trajs2.at(1).at(j - 1).at(1),
            trajs2.at(0).at(j - 1).at(3), trajs2.at(1).at(j - 1).at(3));
        coe1.push_back(dphiphi.front());
        coe2.push_back(dphiphi.back());
      } else {
        //              ScopedTimer tm("trivial");
        coe1.push_back(RationalPoly(SymbolicPoly(
            SymbolicPoly::PolyArbitrary(
                1, SymbolicPoly::PolyPair(trajs2.at(0).at(j).at(i), 1)),
            1.0)));
        coe2.push_back(RationalPoly(SymbolicPoly(
            SymbolicPoly::PolyArbitrary(
                1, SymbolicPoly::PolyPair(trajs2.at(1).at(j).at(i), 1)),
            1.0)));
      }
    }
    std::vector<boost::shared_ptr<NestedExpression>> sphere_parts1,
        sphere_parts2;
    for (auto &co : coe1)
      sphere_parts1.push_back(boost::make_shared<RationalNested>(co));
    for (auto &co : coe2)
      sphere_parts2.push_back(boost::make_shared<RationalNested>(co));

    for (int i = 0; i < pair.first.rows(); i++) {
      VecD ai = pair.first.block(i, 0, 1, pair.first.cols()).transpose();
      VecD ai_sub = ai.block(0, 0, 3, 1);
      for (uint k = 0; k < traj.at(0).at(j).size(); k++) {
        boost::shared_ptr<AxbConstraint> con =
            boost::make_shared<AxbConstraint>(this, j, k, ai_sub,
                                              pair.second(i));
        //        boost::shared_ptr<AxbConstraint> con =
        //            boost::make_shared<AxbConstraint>(this, j, k, ai,
        //            pair.second(i));
        //        if(r3)
        solver.addConstraint(con);
        boost::shared_ptr<NestedAxbConstraint> con2 =
            boost::make_shared<NestedAxbConstraint>(
                sphere_parts1, sphere_parts2, j, k, ai, pair.second(i),
                times.at(j)->getVal());
        //                std::cout  << "con " << con->id << " : " << con->poly
        //                << std::endl;
        //        if(!r3)
        con2->vars = svars;
        solvers2.addConstraint(con2);
      }
    }
    j++;
  }
}
TrajData S2R3Traj::serialize() {
  // just copy variable feilds to struct
  TrajData trajd(9, seg_, deg_);
  for (int d = 0; d < dim_; d++) {
    for (int j = 0; j < seg_; j++) {
      Poly poly;
      for (int k = 0; k <= deg_; k++) {
        boost::shared_ptr<StandardBasis> ptr =
            boost::static_pointer_cast<StandardBasis>(basis.getBasis(0));
        //                 poly +=
        //                 traj.at(d).at(j).at(k)->getVal()/std::pow(times.at(j)->getVal(),int(k/2))*ptr->getPoly(k);
        if (j > 0 && d > 2 && k == 0) {
          Quat dchart = charts.at(j).inverse() * charts.at(j - 1);
          auto phiphi =
              S2R3Traj::getPhiPhi(dchart, trajs2.at(0).at(j - 1).at(1),
                                  trajs2.at(1).at(j - 1).at(1));
          poly += phiphi.at(d - 3).evaluate() *
                  std::pow(times.at(j)->getVal(), k / 2) * ptr->getPoly(k);
        } else if (j > 0 && d > 2 && k == 2) {
          Quat dchart = charts.at(j).inverse() * charts.at(j - 1);
          auto dphiphi = S2R3Traj::getdPhiPhi(
              dchart, trajs2.at(0).at(j - 1).at(1),
              trajs2.at(1).at(j - 1).at(1), trajs2.at(0).at(j - 1).at(3),
              trajs2.at(1).at(j - 1).at(3));
          poly += dphiphi.at(d - 3).evaluate() *
                  std::pow(times.at(j)->getVal(), k / 2) * ptr->getPoly(k);
        } else if (d < 3) {
          poly += traj.at(d).at(j).at(k)->getVal() *
                  std::pow(times.at(j)->getVal(), k / 2) * ptr->getPoly(k);
        } else {
          poly += trajs2.at(d - 3).at(j).at(k)->getVal() *
                  std::pow(times.at(j)->getVal(), k / 2) * ptr->getPoly(k);
        }
      }
      for (int k = 0; k <= deg_; k++) {
        trajd.data.at(d).segs.at(j).coeffs.at(k) = poly[k];
      }
      //            std::cout << "Serialized poly " << poly << std::endl;
      trajd.data.at(d).segs.at(j).dt = times.at(j)->getVal();
      trajd.data.at(d).segs.at(j).basis = PolyType::ENDPOINT;
    }
  }
  // add in quaternion
  for (int d = dim_; d < 9; d++) {
    for (int j = 0; j < seg_; j++) {
      if (d == dim_) {
        trajd.data.at(d).segs.at(j).coeffs.at(0) = charts.at(j).w();
        //            trajd.data.at(d).segs.at(j).coeffs.at(1) =
        //            charts.at(j).w();
      } else if (d == dim_ + 1) {
        trajd.data.at(d).segs.at(j).coeffs.at(0) = charts.at(j).x();
        //            trajd.data.at(d).segs.at(j).coeffs.at(1) =
        //            charts.at(j).x();
      } else if (d == dim_ + 2) {
        trajd.data.at(d).segs.at(j).coeffs.at(0) = charts.at(j).y();
        //            trajd.data.at(d).segs.at(j).coeffs.at(1) =
        //            charts.at(j).y();
      } else if (d == dim_ + 3) {
        trajd.data.at(d).segs.at(j).coeffs.at(0) = charts.at(j).z();
        //            trajd.data.at(d).segs.at(j).coeffs.at(1) =
        //            charts.at(j).z();
      }
    }
  }

  return trajd;
}

void SphereCost::testGrad() {
  std::set<Variable *> myvars;
  for (auto &di : traj_)
    for (auto &si : di)
      for (auto &vi : si)
        if (vi->getId() >= 0) myvars.insert(vi);

  int i = 0;
  for (auto p : cost_parts) {
    //
    decimal_t c0 = std::pow(p.evaluate(), 2.0);
    decimal_t h = 0.001;
    ETV grad = NonlinearSolver::transpose(p.gradient2(0));
    SpMat gradv(myvars.size(), 1);
    gradv.setFromTriplets(grad.begin(), grad.end());
    VecD gradvd(gradv);

    VecD numgrad = VecD::Zero(myvars.size());
    for (auto &var : myvars) {
      var->val += h;
      decimal_t cv = std::pow(p.evaluate(), 2.0);
      numgrad(var->id) = (cv - c0) / h;
      var->val -= h;
    }
    std::cout << "i " << i++ << std::endl;
    std::cout << "grad " << gradvd.transpose() << std::endl;
    std::cout << "gradn " << numgrad.transpose() << std::endl;
    if ((gradvd - numgrad).norm() > 0.5) {
      std::cout << "High error " << p << "val " << p.evaluate() << std::endl;
      ETV grad2 = p.gradient2(0);
      ETV gradn = p.gradient(0);
      for (auto &gi : grad2)
        std::cout << gi.row() << " " << gi.col() << " " << gi.value()
                  << std::endl;
      std::cout << std::endl;
      for (auto &gi : gradn)
        std::cout << gi.row() << " " << gi.col() << " " << gi.value()
                  << std::endl;
      std::cout << std::endl;
    }
  }
}

}  // namespace traj_opt

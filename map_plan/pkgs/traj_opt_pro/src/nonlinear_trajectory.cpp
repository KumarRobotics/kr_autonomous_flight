// Copyright 2017 Michael Watterson
// #include <hopf_control/hopf_helper.h>
#include <traj_opt_pro/cheby_fit.h>
#include <traj_opt_pro/nonlinear_trajectory.h>

#include <utility>
#include <vector>

namespace traj_opt {

void PolyCost::init_constants() {
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
  } else if (min_dim == 1) {
    cost_n_ << -1, -1, 0, 0, 1, 1, 2, 2, -1, -1, 0, 0, 1, 1, 2, 2, 0, 0, 1, 1,
        2, 2, 3, 3, 0, 0, 1, 1, 2, 2, 3, 3, 1, 1, 2, 2, 3, 3, 4, 4, 1, 1, 2, 2,
        3, 3, 4, 4, 2, 2, 3, 3, 4, 4, 5, 5, 2, 2, 3, 3, 4, 4, 5, 5;
    cost_v_ << 1.631701631701632, -1.631701631701632, 0.315850815850816,
        0.315850815850816, 0.026806526806527, -0.026806526806527,
        0.000971250971251, 0.000971250971251, -1.631701631701632,
        1.631701631701632, -0.315850815850816, -0.315850815850816,
        -0.026806526806527, 0.026806526806527, -0.000971250971251,
        -0.000971250971251, 0.315850815850816, -0.315850815850816,
        0.299700299700300, 0.016150516150516, 0.030719280719281,
        0.003912753912754, 0.001387501387501, -0.000416250416250,
        0.315850815850816, -0.315850815850816, 0.016150516150516,
        0.299700299700300, -0.003912753912754, -0.030719280719281,
        -0.000416250416250, 0.001387501387501, 0.026806526806527,
        -0.026806526806527, 0.030719280719281, -0.003912753912754,
        0.004051504051504, 0.001359751359751, 0.000205350205350,
        -0.000101287601288, -0.026806526806527, 0.026806526806527,
        0.003912753912754, -0.030719280719281, 0.001359751359751,
        0.004051504051504, 0.000101287601288, -0.000205350205350,
        0.000971250971251, -0.000971250971251, 0.001387501387501,
        -0.000416250416250, 0.000205350205350, 0.000101287601288,
        0.000011100011100, -0.000006937506938, 0.000971250971251,
        -0.000971250971251, -0.000416250416250, 0.001387501387501,
        -0.000101287601288, -0.000205350205350, -0.000006937506938,
        0.000011100011100;
  } else {
    throw 2;  // unsupported
  }
}

// standard formulation
NonlinearTrajectory::NonlinearTrajectory(
    const std::vector<Waypoint> &waypoints,
    const std::vector<std::pair<MatD, VecD>> &cons, int deg, int min_dim,
    boost::shared_ptr<std::vector<decimal_t>> ds,
    boost::shared_ptr<VecDVec> path)
    : seg_(cons.size()), deg_(deg), basis(PolyType::ENDPOINT, deg_, min_dim) {
  dim_ = waypoints.front().pos.rows();
  assert(dim_ == cons.front().first.cols());

  auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
  auto bezier_basis = boost::make_shared<BezierBasis>(deg_);
  basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);
  basisStandard = boost::make_shared<BasisTransformer>(endpoint_basis);
  waypoints_ = waypoints;

  // if (ds != NULL) std::cout << "ds " << ds->size() << std::endl;
  // if (path != NULL) std::cout << "path " << path->size() << std::endl;

  // std::cout << "con " << cons.size() << std::endl;
  bool marginalize = true;
  bool use_knots = waypoints.size() > 2;
  bool use_all = false;
  if (waypoints.size() > 2 && waypoints.at(1).use_vel) use_all = true;
  if (waypoints.size() > 1 && !waypoints.at(1).use_pos) use_all = true;
  // setup poly
  allocate_poly(ds, path, marginalize, use_knots, use_all, true);
  // equality
  add_boundary(waypoints, marginalize);
  if (!marginalize) link_sections();
  // ineq
  if (solver.num_vars() == 0) {
    if (solver.num_const_vars() == 0)
      solved_ = false;
    else
      solved_ = true;
    return;
  }

  add_Axb(cons);
  decimal_t alpha = 1.0;
  cost = boost::make_shared<PolyCost>(traj, times, basis, min_dim, alpha);

  solver.setCost(cost);
  solved_ = solver.solve(false);
}
// point cloud formulation
NonlinearTrajectory::NonlinearTrajectory(const std::vector<Waypoint> &waypoints,
                                         const Vec3Vec &points, int segs,
                                         decimal_t dt)
    : seg_(segs), deg_(7), basis(PolyType::ENDPOINT, deg_, 3) {
  dim_ = waypoints.front().pos.rows();
  auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
  auto bezier_basis = boost::make_shared<BezierBasis>(deg_);
  basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);

  boost::shared_ptr<std::vector<decimal_t>> ds =
      boost::make_shared<std::vector<decimal_t>>(segs, dt);

  // setup poly
  allocate_poly(ds);
  allocate_beads();
  // equality
  add_boundary(waypoints);
  link_sections();
  // ineq
  addCloudConstraint(points);
  addPosTime();
  // bound time total
  //      addTimeBound(ds);
  make_convex(ds);
  // cost
  cost = boost::make_shared<PolyCost>(traj, times, basis, 3);

  solver.setCost(cost);
  // call solver
  // solved_ = solver.solve(true);
  solved_ = solver.solve(false);
}
// vision based formulation
NonlinearTrajectory::NonlinearTrajectory(
    const std::vector<Waypoint> &waypoints,
    const std::vector<std::pair<MatD, VecD>> &con, const Vec3Vec &points,
    decimal_t dt)
    : seg_(con.size()), deg_(7), basis(PolyType::ENDPOINT, deg_, 3) {
  dim_ = waypoints.front().pos.rows();
  auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
  auto bezier_basis = boost::make_shared<BezierBasis>(deg_);
  basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);
  basisStandard = boost::make_shared<BasisTransformer>(endpoint_basis);

  boost::shared_ptr<std::vector<decimal_t>> ds =
      boost::make_shared<std::vector<decimal_t>>(seg_, dt);

  // setup poly
  allocate_poly(ds);
  allocate_evaluations(points.size());
  // equality
  add_boundary(waypoints);
  link_sections();
  // ineq
  add_Axb(con);
  addVisibility(points, 0.5);
  addPosTime();
  // bound time total
  //      addTimeBound(ds);
  make_convex(ds);
  // cost
  cost = boost::make_shared<PolyCost>(traj, times, basis, 3);

  solver.setCost(cost);
  // call solver
  // solved_ = solver.solve(true);
  solved_ = solver.solve(false);
}

decimal_t NonlinearTrajectory::getCost() { return cost->evaluate(); }

void NonlinearTrajectory::allocate_poly(
    boost::shared_ptr<std::vector<decimal_t>> ds,
    boost::shared_ptr<VecDVec> path, bool marginalize, bool use_knots,
    bool use_all, bool convex) {
  traj.clear();
  //  std::cout << "Use knots " << use_knots << std::endl;
  for (int i = 0; i < dim_; i++) {
    NLSpline spline;
    for (int j = 0; j < seg_; j++) {
      NLPoly pi;
      for (int k = 0; k <= deg_; k++) {
        if (marginalize && j == 0 && k % 2 == 0)
          pi.push_back(solver.addConstVar(2.0));
        else if (marginalize && j == (seg_ - 1) && k % 2 == 1 && !use_all)
          pi.push_back(solver.addConstVar(2.0));
        else if (marginalize && j > 0 && k % 2 == 0) {
          pi.push_back(spline.at(j - 1).at(k + 1));  // this is key
          pi.back()->markDuplicate();
        } else if (marginalize && use_all && k % 2 == 1 && k != 1)
          pi.push_back(solver.addConstVar(2.0));
        else if (marginalize && use_all && k == 1)
          pi.push_back(solver.addVar(2.0));
        else if (marginalize && use_knots && k == 1)
          pi.push_back(solver.addConstVar(2.0));
        else if (k == 1 && j < (seg_ - 1) && path != NULL)
          pi.push_back(solver.addConstVar(path->at(j + 1)(i)));
        else
          pi.push_back(solver.addVar(2.0));
      }
      spline.push_back(pi);
      // std::cout << "pi " << pi.size() << std::endl;
    }
    traj.push_back(spline);
    // std::cout << "spline " << spline.size() << std::endl;
  }

  for (int j = 0; j < seg_; j++) {
    if (ds != NULL) {
      if (convex)
        times.push_back(solver.addConstVar(ds->at(j)));
      else
        times.push_back(solver.addVar(ds->at(j), false, true));
    } else {
      if (convex)
        times.push_back(solver.addConstVar());
      else
        times.push_back(solver.addVar(10.0, false, true));
    }
  }
}

void NonlinearTrajectory::allocate_beads() {
  beads.clear();
  for (int i = 0; i <= dim_; i++) {
    NLChain chain;
    for (int j = 0; j < seg_; j++) chain.push_back(solver.addVar(2.0));
    beads.push_back(chain);
  }
}

void NonlinearTrajectory::link_sections() {
  for (int d = 0; d < dim_; d++) {
    for (int q = 0; q <= 3; q++) {
      for (int j = 0; j < (seg_ - 1); j++) {
        std::vector<EqConstraint::EqPair> con;
        con.push_back(
            EqConstraint::EqPair(traj.at(d).at(j).at(2 * q + 1), 1.0));
        con.push_back(
            EqConstraint::EqPair(traj.at(d).at(j + 1).at(2 * q), -1.0));
        solver.addConstraint(con, 0.0);
      }
    }
  }
}

void NonlinearTrajectory::make_convex(
    boost::shared_ptr<std::vector<decimal_t>> ds) {
  int i = 0;
  for (auto t : times) {
    std::vector<EqConstraint::EqPair> con;
    con.push_back(EqConstraint::EqPair(t, 1.0));
    if (ds != NULL)
      solver.addConstraint(con, ds->at(i));
    else
      solver.addConstraint(con, 1.0);
    i++;
  }
}

void NonlinearTrajectory::add_boundary(const std::vector<Waypoint> &waypoints,
                                       bool marginalize) {
  for (auto &point : waypoints) {
    int id = point.knot_id;
    std::pair<Eigen::VectorXi, MatD> pair = point.getIndexForm();
    // note we use start of each segment, unless id == seg_
    if (id < 0) id += seg_ + 1;

    for (int c = 0; c < pair.first.rows(); c++) {
      int co = 2 * pair.first(c);
      int idi = id;
      if (id == seg_) {
        co += 1;
        idi--;
      }

      for (int d = 0; d < dim_; d++) {
        if (marginalize) {
          //           std::cout << "traj size " << traj.size() << std::endl;
          //           std::cout << "spline size " << traj.at(d).size() <<
          //           std::endl; std::cout << "poly size " <<
          //           traj.at(d).at(idi).size() << std::endl;
          traj.at(d).at(idi).at(co)->val = pair.second(d, c);
          //          std::cout << "setting constant " <<
          //          traj.at(d).at(idi).at(co)->isConstant() << std::endl;
          //          std::cout << "d: "  << d << " idi: " << idi << " co: " <<
          //          co << std::endl;
        } else {
          EqConstraint::EqPair p(traj.at(d).at(idi).at(co), 1);
          std::vector<EqConstraint::EqPair> con(1, p);
          solver.addConstraint(con, pair.second(d, c));
        }
      }
    }
  }
}

#include "constraints.cpp"

void NonlinearTrajectory::addPosTime() {
  for (auto &v : times) {
    boost::shared_ptr<IneqConstraint> con =
        boost::make_shared<PosTimeConstraint>(v);
    solver.addConstraint(con);
  }
}
void NonlinearTrajectory::addTimeBound(
    boost::shared_ptr<std::vector<decimal_t>> ds) {
  boost::shared_ptr<IneqConstraint> con;
  if (ds == NULL) {
    con = boost::make_shared<TimeBound>(times, 10.0);
  } else {
    decimal_t T = 0.0;
    for (auto t : *ds) T += t;
    con = boost::make_shared<TimeBound>(times, T);
  }

  solver.addConstraint(con);
}

void NonlinearTrajectory::add_Axb(
    const std::vector<std::pair<MatD, VecD>> &cons) {
  int j = 0;
  for (auto &pair : cons) {
    assert(pair.first.rows() == pair.second.rows());
    for (int i = 0; i < pair.first.rows(); i++) {
      VecD ai = pair.first.block(i, 0, 1, pair.first.cols()).transpose();
      for (uint k = 0; k < traj.at(0).at(j).size(); k++) {
        boost::shared_ptr<AxbConstraint> con =
            boost::make_shared<AxbConstraint>(this, j, k, ai, pair.second(i));
        solver.addConstraint(con);
        //                std::cout  << "con " << con->id << " : " << con->poly
        //                << std::endl;
      }
    }
    j++;
  }
}

decimal_t NonlinearTrajectory::getTotalTime() const {
  decimal_t T = 0;
  for (auto &v : times) T += v->getVal();
  return T;
}

bool NonlinearTrajectory::evaluate(decimal_t t, uint derr, VecD &out) const {
  Vec3 g;
  g << 0.0, 0.0, 9.81;
  t = boost::algorithm::clamp(t, 0.0, getTotalTime());
  int j = 0;
  while (j < seg_ && t > times.at(j)->getVal()) {
    t -= times.at(j)->getVal();
    j++;
  }
  j = boost::algorithm::clamp(j, 0, seg_ - 1);
  // decimal_t val=0;
  decimal_t dt = times.at(j)->getVal();
  decimal_t s = t / dt;
  out = VecD::Zero(traj.size());
  for (int d = 0; d < traj.size(); d++) {
    for (int i = 0; i <= deg_; i++) {
      Variable *vk = traj.at(d).at(j).at(i);
      if (vk != NULL) {
        out(d) +=
            basis.getVal(s, 1.0, i, derr) * vk->getVal() * std::pow(dt, i / 2);
      } else {
        decimal_t val =
            (lamdas.at(j - !(i % 2))->getVal() * xi_hats.at(j - !(i % 2))(d) -
             g(d));
        out(d) += basis.getVal(s, 1.0, i, derr) * val * std::pow(dt, i / 2);
      }
    }
  }
  out /= std::pow(times.at(j)->getVal(), derr);
  //  std::cout << "eval derr " << derr << " val " << out.transpose() <<
  //  std::endl;
  return true;
}

TrajData NonlinearTrajectory::serialize() {
  // just copy variable feilds to struct
  Vec3 g;
  g << 0.0, 0.0, 9.81;
  TrajData trajd(dim_, seg_, deg_);
  for (int d = 0; d < dim_; d++) {
    for (int j = 0; j < seg_; j++) {
      Poly poly;
      for (int k = 0; k <= deg_; k++) {
        boost::shared_ptr<StandardBasis> ptr =
            boost::static_pointer_cast<StandardBasis>(basis.getBasis(0));
        Variable *vk = traj.at(d).at(j).at(k);
        if (vk != NULL)
          poly += vk->getVal() * std::pow(times.at(j)->getVal(), k / 2) *
                  ptr->getPoly(k);
        else
          poly +=
              (lamdas.at(j - !(k % 2))->getVal() * xi_hats.at(j - !(k % 2))(d) -
               g(d)) *
              std::pow(times.at(j)->getVal(), k / 2) * ptr->getPoly(k);
      }
      for (int k = 0; k <= deg_; k++) {
        trajd.data.at(d).segs.at(j).coeffs.at(k) = poly[k];
      }
      //            std::cout << "Serialized poly " << poly << std::endl;
      trajd.data.at(d).segs.at(j).dt = times.at(j)->getVal();
      trajd.data.at(d).segs.at(j).basis = PolyType::ENDPOINT;
    }
  }

  return trajd;
}

// TrajData NonlinearTrajectory::serialize_hopf(
//     const boost::shared_ptr<Trajectory> &traj) {
//   TrajData trajd(5, yaw_seg_, deg_);
//   for (int d = 0; d < 5; d++) {
//     decimal_t t_total = 0.0;
//     for (int j = 0; j < yaw_seg_; j++) {
//       decimal_t dt = times_yaw.at(j)->val;
//       Poly poly;
//       if (d < 3) {
//         for (int k = 0; k <= deg_; k++) {
//           boost::shared_ptr<StandardBasis> ptr =
//               boost::static_pointer_cast<StandardBasis>(basis.getBasis(0));
//           int derr = k / 2;
//           decimal_t point = t_total + (k % 2) * dt;
//           VecD out;
//           traj->evaluate(point, derr, out);
//           decimal_t val = out(d);
//           poly += val * std::pow(times_yaw.at(j)->getVal(), k / 2) *
//                   ptr->getPoly(k);
//         }
//       } else if (d < 4) {
//         // yaw
//         for (int k = 0; k <= deg_; k++) {
//           boost::shared_ptr<StandardBasis> ptr =
//               boost::static_pointer_cast<StandardBasis>(basis.getBasis(0));
//           decimal_t val = traj_yaw.at(0).at(j).at(k)->val;
//           poly += val * std::pow(times_yaw.at(j)->getVal(), k / 2) *
//                   ptr->getPoly(k);
//         }
//       } else {
//         // chart
//         std::vector<decimal_t> coe(deg_ + 1, 0.0);
//         coe.front() = !yaw_charts.at(j);
//         poly = Poly(coe.data(), deg_);
//       }
//       for (int k = 0; k <= deg_; k++) {
//         trajd.data.at(d).segs.at(j).coeffs.at(k) = poly[k];
//       }
//       trajd.data.at(d).segs.at(j).dt = dt;
//       t_total += dt;
//     }
//   }
//   return trajd;
// }

// std::ostream &operator<<(std::ostream &os, const SymbolicPoly &poly) {
//   for (auto p : poly.poly_map) {
//     Variable *coeff = std::get<0>(p.first);
//     Variable *time = std::get<1>(p.first);
//     int n = std::get<2>(p.first);
//     os << p.second << "*c" << coeff->getId() << "*t" << time->getId() << "^"
//        << n << std::endl;
//   }
//   for (auto &p : poly.quad_map) {
//     Variable *coeff0 = std::get<0>(p.first);
//     Variable *coeff1 = std::get<1>(p.first);
//     Variable *time = std::get<2>(p.first);
//     int n = std::get<3>(p.first);
//     os << p.second << "*c" << coeff0->getId() << "*c" << coeff1->getId() <<
//     "*t"
//        << time->getId() << "^" << n << std::endl;
//   }
//   return os;
// }
// polycost
PolyCost::PolyCost(const NLTraj &traj, const NLTimes &times,
                   BasisBundlePro &basis, int min_dim, decimal_t alpha,
                   int max_dim) {
  init_constants();
  int dim = traj.size();
  if (max_dim > 0) dim = std::min(dim, max_dim);
  int segs = times.size();
  int deg = traj.front().front().size();
  for (int d = 0; d < dim; d++) {
    for (int s = 0; s < segs; s++) {
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j <= i; j++) {
          if (i == j)
            poly += SymbolicPoly(traj.at(d).at(s).at(i), traj.at(d).at(s).at(j),
                                 times.at(s), cost_n_(i, j), cost_v_(i, j));
          else  // add symetric elements properly
            poly +=
                SymbolicPoly(traj.at(d).at(s).at(i), traj.at(d).at(s).at(j),
                             times.at(s), cost_n_(i, j), 2.0 * cost_v_(i, j));
        }
      }
    }
  }
  for (int s = 0; s < segs; s++) {
    poly += SymbolicPoly(times.at(s), times.at(s), 0, alpha);
  }
  //      std::cout << "Cost " << poly << std::endl;
}
// TODO(mike) fix gradients and hessions for this new form
PolyCost::PolyCost(const NLTraj &traj, const NLTimes &lamdas,
                   const NLTimes &times, BasisBundlePro &basis,
                   const VecDVec &xi_hats) {
  // lets assume traj is null if lamda, and we have a vector lamdas for each
  // lamdas(i)*xi_hat(i) - g = \ddot{r}
  Vec3 g;
  g << 0.0, 0.0, 9.81;

  init_constants();
  int dim = traj.size();
  int segs = times.size();
  int deg = traj.front().front().size();
  for (int d = 0; d < dim; d++) {
    for (int s = 0; s < segs; s++) {
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j <= i; j++) {
          Variable *vi = traj.at(d).at(s).at(i);
          Variable *vj = traj.at(d).at(s).at(j);
          decimal_t extra = 1.0;
          if (vi == NULL && vj == NULL) {
            vi = lamdas.at(s - !(i % 2));
            vj = lamdas.at(s - !(j % 2));
            decimal_t gi = g(d);
            decimal_t xi = xi_hats.at(s - !(i % 2))(d);
            decimal_t xj = xi_hats.at(s - !(j % 2))(d);
            // don't need to add gi^2 term (it is constant) // just add linear
            // terms
            poly += SymbolicPoly(vi, times.at(s), cost_n_(i, j),
                                 -gi * xi * cost_v_(i, j));
            poly += SymbolicPoly(vj, times.at(s), cost_n_(i, j),
                                 -gi * xi * cost_v_(i, j));
            extra = xi * xj;
          } else if (vi == NULL) {
            decimal_t gi = g(d);
            decimal_t xi = xi_hats.at(s - !(i % 2))(d);
            extra = xi;
            vi = lamdas.at(s - !(i % 2));
            poly += SymbolicPoly(vi, times.at(s), cost_n_(i, j),
                                 -gi * cost_v_(i, j));
          } else if (vj == NULL) {
            decimal_t gi = g(d);
            decimal_t xj = xi_hats.at(s - !(j % 2))(d);
            extra = xj;
            vj = lamdas.at(s - !(j % 2));
            poly += SymbolicPoly(vj, times.at(s), cost_n_(i, j),
                                 -gi * cost_v_(i, j));
          }
          if (vi == NULL || vj == NULL) throw 3;  // bug
          if (i == j) {
            poly += SymbolicPoly(vi, vj, times.at(s), cost_n_(i, j),
                                 extra * cost_v_(i, j));
          } else {  // add symetric elements properly
            poly += SymbolicPoly(vi, vj, times.at(s), cost_n_(i, j),
                                 2.0 * extra * cost_v_(i, j));
          }
        }
      }
    }
  }
  //    std::cout << "Cost " << poly << std::endl;
}
PolyCost::PolyCost(const NLTraj &traj, const NLTimes &times,
                   BasisBundlePro &basis, const PolyTraj &fit_yaw) {
  min_dim = 1;
  init_constants();
  int dim = traj.size();
  int segs = times.size();
  int deg = traj.front().front().size();

  for (int d = 0; d < dim; d++) {
    for (int s = 0; s < segs; s++) {
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j < deg; j++) {
          Variable *vi = traj.at(d).at(s).at(i);
          Variable *vj = traj.at(d).at(s).at(j);
          decimal_t xj = fit_yaw.at(d).at(s)(j);
          poly +=
              SymbolicPoly(vi, vj, times.at(s), cost_n_(i, j), cost_v_(i, j));
          poly += SymbolicPoly(vi, times.at(s), cost_n_(i, j),
                               -2.0 * xj * cost_v_(i, j));
          constant += std::pow(times.at(s)->val, cost_n_(i, j)) * xj * xj *
                      cost_v_(i, j);
        }
      }
    }
  }
}

decimal_t PolyCost::evaluate() {
  double val = constant + poly.evaluate() + 1e-10;
  if (val < 0) {
    std::cout << "error cost is less than zero " << std::endl;
    throw 22;
  }

  return val;
}
ETV PolyCost::gradient() {
  ETV qgrad = poly.quad_gradient();
  ETV grad = NonlinearSolver::transpose(poly.gradient(0));
  qgrad.insert(qgrad.end(), grad.begin(), grad.end());
  return qgrad;
}
ETV PolyCost::hessian() {
  ETV qhess = poly.quad_hessian();
  ETV hess = poly.hessian();
  qhess.insert(qhess.end(), hess.begin(), hess.end());
  return qhess;
}
void NonlinearTrajectory::addCloudConstraint(const Vec3Vec &points) {
  for (int j = 0; j < seg_; j++) {
    for (int k = 0; k <= deg_; k++) {
      boost::shared_ptr<BallConstraint> con =
          boost::make_shared<BallConstraint>(this, j, k);
      solver.addConstraint(con);
    }
    for (auto &p : points) {
      boost::shared_ptr<BallConstraint> con =
          boost::make_shared<BallConstraint>(this, j, p,
                                             0.8);  // hardcode robot_r for now
      solver.addConstraint(con);
    }
  }
}

Vec4Vec NonlinearTrajectory::getBeads() {
  Vec4Vec res(seg_, Vec4::Zero());
  for (int i = 0; i < seg_; i++)
    for (int d = 0; d <= dim_; d++) res.at(i)(d) = beads.at(d).at(i)->getVal();
  return res;
}

void NonlinearTrajectory::scaleTime(decimal_t ratio) {
  for (auto t : times) t->val *= ratio;

  std::list<int> duplicates;
  for (int d = 0; d < traj.size(); d++) {
    for (int j = 0; j < seg_; j++) {
      for (int k = 0; k <= deg_; k++) {
        if (traj.at(d).at(j).at(k)->isUnique())
          traj.at(d).at(j).at(k)->val /= std::pow(ratio, k / 2);
        else {
          Variable *c = traj.at(d).at(j).at(k);
          if (std::find(duplicates.begin(), duplicates.end(), c->id) ==
              duplicates.end()) {
            c->val /= std::pow(ratio, k / 2);
            duplicates.insert(duplicates.end(), c->id);
          }
        }
      }
    }
  }

  duplicates.clear();
  for (int d = 0; d < trajs2.size(); d++) {
    for (int j = 0; j < seg_; j++) {
      for (int k = 0; k <= deg_; k++) {
        if (trajs2.at(d).at(j).at(k)->isUnique())
          trajs2.at(d).at(j).at(k)->val /= std::pow(ratio, k / 2);
        else {
          Variable *c = trajs2.at(d).at(j).at(k);
          if (std::find(duplicates.begin(), duplicates.end(), c->id) ==
              duplicates.end()) {
            c->val /= std::pow(ratio, k / 2);
            duplicates.insert(duplicates.end(), c->id);
          }
        }
      }
    }
  }
}

void NonlinearTrajectory::allocate_evaluations(uint num_eval) {
  for (uint i = 0; i < num_eval; i++)
    evaluation_points.push_back(solver.addVar(2.0));
}

void NonlinearTrajectory::addVisibility(const Vec3Vec &points, decimal_t dist) {
  decimal_t T = getTotalTime();
  std::cout << "total time " << T << std::endl;
  VecD dynamic = VecD::Zero(3);
  for (uint i = 0; i < points.size(); i++) {
    boost::shared_ptr<IneqConstraint> con = boost::make_shared<TimeBound>(
        std::vector<Variable *>(1, evaluation_points.at(i)), T);
    solver.addConstraint(con);
    con = boost::make_shared<PosTimeConstraint>(evaluation_points.at(i));
    solver.addConstraint(con);
    dynamic.block<3, 1>(0, 0) = points.at(i);
    con = boost::make_shared<MinDist>(this, evaluation_points.at(i), dynamic,
                                      dist);
    solver.addConstraint(con);
  }
}

void NonlinearTrajectory::adjustTimeToMax(decimal_t max_v, decimal_t max_a,
                                          decimal_t max_j) {
  decimal_t nv = 0, na = 0, nj = 0;
  std::cout << "Adjusting from time " << getTotalTime() << std::endl;
  for (decimal_t t = 0; t < getTotalTime(); t += 0.01) {
    VecD val;
    evaluate(t, 1, val);
    nv = std::max(nv, val.norm());
    evaluate(t, 2, val);
    na = std::max(na, val.norm());
    evaluate(t, 3, val);
    nj = std::max(nj, val.norm());
  }
  decimal_t rv = nv / max_v;
  decimal_t ra = std::sqrt(na / max_a);
  decimal_t rj = std::pow(nj / max_j, 0.33333333333);
  decimal_t T = std::max(std::max(rv, ra), rj);
  std::cout << "Vmax: " << nv << " Amax: " << na << " Jmax " << nj << std::endl;
  std::cout << "Scaling to " << T * getTotalTime() << std::endl;
  std::cout << "Vmax: " << nv / T << " Amax: " << na / T / T << " Jmax "
            << nj / T / T / T << std::endl;
  if (T > 1000) {
    solved_ = false;
    return;
  }
  scaleTime(T);

  if (na > 2.5 * max_a || nj > 1.5 * max_j) {
    std::cout << "Resolve is wonky, forcing fail. " << std::endl;
    solved_ = false;
  }
}

// void NonlinearTrajectory::allocate_poly_hopf(const std::vector<decimal_t>
// &ds) {
//   traj.clear();
//   //  std::cout << "Use knots " << use_knots << std::endl;
//   for (int i = 0; i < dim_; i++) {
//     NLSpline spline;
//     for (int j = 0; j < seg_; j++) {
//       NLPoly pi;
//       for (int k = 0; k <= deg_; k++) {
//         if (j == 0 && k % 2 == 0)
//           pi.push_back(solver.addConstVar(2.0));
//         else if (j == (seg_ - 1) && k % 2 == 1)
//           pi.push_back(solver.addConstVar(2.0));
//         else if (j < (seg_ - 1) && k == 5)
//           pi.push_back(NULL);
//         //        else if (j > 0 && j < (seg_ - 1) && k == 3 || k == 4)
//         //          pi.push_back(NULL);
//         else if (j > 0 && k % 2 == 0) {
//           pi.push_back(spline.at(j - 1).at(k + 1));  // this is key
//           if (pi.back() != NULL) pi.back()->markDuplicate();
//         } else if (k == 1)
//           pi.push_back(solver.addConstVar(2.0));
//         else
//           pi.push_back(solver.addVar(2.0));
//       }
//       spline.push_back(pi);
//       // std::cout << "pi " << pi.size() << std::endl;
//     }
//     traj.push_back(spline);
//     // std::cout << "spline " << spline.size() << std::endl;
//   }

//   for (int j = 0; j < xi_hats.size(); j++)
//   lamdas.push_back(solver.addVar(2.0));

//   for (int j = 0; j < seg_; j++) times.push_back(solver.addVar(ds.at(j)));
// }

// NonlinearTrajectory::NonlinearTrajectory(
//     const std::vector<WaypointHopf> &waypoints,
//     const std::vector<std::pair<MatD, VecD>> &con, decimal_t dt)
//     : seg_(con.size()), deg_(7), basis(PolyType::ENDPOINT, deg_, 3) {
//   dim_ = 3;
//   auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
//   auto bezier_basis = boost::make_shared<BezierBasis>(deg_);
//   basisT = boost::make_shared<BasisTransformer>(endpoint_basis,
//   bezier_basis);

//   xi_hats.clear();
//   for (int i = 1; i < (waypoints.size() - 1); i++) {
//     xi_hats.push_back(waypoints.at(i).xi_hat);
//   }
//   std::vector<decimal_t> dts(waypoints.size() - 1, dt);
//   allocate_poly_hopf(dts);

//   std::vector<Waypoint> waynormal;
//   for (auto &w : waypoints) waynormal.push_back(WaypointHopf::cast(w));
//   add_boundary(waynormal, true);

//   add_Axb(con);
//   addPosLamda();
//   addPosTime();

//   cost = boost::make_shared<PolyCost>(traj, lamdas, times, basis, xi_hats);

//   solver.setCost(cost);
//   // call solver
//   // solved_ = solver.solve(true);
//   solved_ = solver.solve_nlopt();

//   if (solved_) {
//     find_charts();
//     for (auto c : yaw_charts) std::cout << "chart " << c << std::endl;

//     // fit yaw
//     PolyTraj yaw = fitHopfYaw();
//     allocate_yaw(yaws);
//     cost_yaw = boost::make_shared<PolyCost>(traj_yaw, times_yaw, basis, yaw);
//     solver_yaw.setCost(cost_yaw);

//     //    solved_ = solver_yaw.solve(true);

//     // paste yaw onto trajectory
//     traj.push_back(traj_yaw.front());
//     dim_ = 4;
//   }
//   //  for(auto &v:lamdas) {
//   //    std::cout << "lamda i: " << v->val << std::endl;
//   //  }
// }

void NonlinearTrajectory::addPosLamda() {
  for (auto &v : lamdas) {
    boost::shared_ptr<IneqConstraint> con =
        boost::make_shared<PosTimeConstraint>(v, 1.0);
    solver.addConstraint(con);
  }
}

// std::pair<Vec3, Vec3> NonlinearTrajectory::getHopf(decimal_t t) {
//   VecD g = VecD::Zero(3, 1);
//   g(2) = 9.81;
//   VecD evalv, evald;
//   evaluate(t, 2, evalv);
//   evaluate(t, 3, evald);
//   VecD eval3 = evalv.block<3, 1>(0, 0);
//   VecD evald3 = evald.block<3, 1>(0, 0);

//   VecD xi = eval3 + g;
//   xi.normalize();
//   // calc differential
//   VecD xid = xi.dot(xi) * evald3;
//   xid -= xi.dot(evald3) * xi;
//   xid /= std::pow(xi.norm(), 3.0);

//   Vec3 xi3, xid3;
//   xi3 = eval3.head(3) + g.head(3);
//   xid3 = xid.head(3);
//   return std::make_pair(eval3, xid3);
// }

// PolyTraj NonlinearTrajectory::fitHopfYaw() {
//   PolyTraj result;
//   auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
//   auto cheby_basis = boost::make_shared<StandardBasis>(deg_);
//   auto bt = boost::make_shared<BasisTransformer>(cheby_basis,
//   endpoint_basis); int res = 100;  // res per segment

//   HopfHelper hh;
//   Vec3 xi, xid;
//   decimal_t T_base = 0.0;
//   PolySpline spline;
//   decimal_t yaw0 = 0.0;  // TODO fix this
//   for (int s = 0; s < yaw_seg_; s++) {
//     decimal_t dt = times_yaw.at(s)->getVal();
//     decimal_t ddt = dt / decimal_t(res - 1);
//     std::vector<decimal_t> eval(res, yaw0);
//     bool hover = yaw_charts.at(s);
//     if (s != 0) {
//       bool changed = yaw_charts.at(s) != yaw_charts.at(s - 1);  // nxor
//       if (changed) {
//         std::tie(xi, xid) = getHopf(T_base);
//         hh.setDes(xi, xid);
//         decimal_t yt, ytd;
//         std::tie(yt, ytd) = hh.calculateYawTransition(yaw0, 0, !hover);
//         yaw_jumps.push_back(yt);

//         std::cout << "yaw jump " << yt << std::endl;
//         yaw0 = yt;
//         eval.front() = yaw0;
//       }
//     }
//     yaws.push_back(yaw0);
//     for (int i = 0; i < (res - 1); i++) {
//       decimal_t ti = T_base + decimal_t(i) * ddt;
//       std::tie(xi, xid) = getHopf(ti);
//       hh.setDes(xi, xid);
//       hh.calculateControl(yaw0, 0, hover);
//       // hopf stuff
//       Vec3 hopf_omega, hopf_force;
//       Quat hopf_orin;
//       hh.getControl(hopf_force, hopf_orin, hopf_omega);

//       decimal_t yawd = -hopf_omega(2);

//       eval.at(i + 1) = eval.at(i) + ddt * yawd;
//     }
//     VecD co = FitCheby::fit_poly(eval, bt);
//     spline.push_back(co);
//     T_base += dt;
//     //    yaw0 = eval.back(); // for haxy thing TODO(mike) fix this
//     //    for(auto &p:eval)
//     //      std::cout << p << std::endl;
//     //    std::cout << "co " << co.transpose() << std::endl;
//   }
//   for (auto &p : yaws) std::cout << "yaws " << p << std::endl;

//   result.push_back(spline);
//   return result;
// }

void NonlinearTrajectory::allocate_yaw(const std::vector<decimal_t> &yaws) {
  // for now, allocate as constant
  // TODO (mike)
  NLSpline spline;
  for (int j = 0; j < yaw_seg_; j++) {
    NLPoly pi;
    for (int k = 0; k <= deg_; k++) {
      if (k == 0)
        pi.push_back(solver_yaw.addConstVar(yaws.at(j)));
      else if (j == 0 && k == 2)
        pi.push_back(solver_yaw.addConstVar(0.0));
      else if (k == 1)
        pi.push_back(solver_yaw.addConstVar(yaws.at(j)));
      else if (j == (yaw_seg_ - 1) && k == 3)
        pi.push_back(solver_yaw.addConstVar(0.0));
      else
        pi.push_back(solver_yaw.addVar(0.0));
    }
    spline.push_back(pi);
    // std::cout << "pi " << pi.size() << std::endl;
  }
  traj_yaw.push_back(spline);
  // std::cout << "spline " << spline.size() << std::endl;
}

void NonlinearTrajectory::find_charts() {
  uint samples = 100 * seg_;
  decimal_t T = getTotalTime();
  decimal_t dt = T / decimal_t(samples - 1);
  // assume start and end are hover for now
  std::vector<decimal_t> dts;
  std::vector<bool> charts;

  VecD up = VecD::Zero(3, 1);
  up << 0., 0., 1.;
  VecD g = VecD::Zero(3, 1);
  g << 0., 0., 9.81;
  VecD temp;

  bool hover = true;
  //  charts.push_back(hover);

  // find critical points
  for (int i = 0; i < samples; i++) {
    decimal_t dti = dt * decimal_t(i);
    evaluate(dti, 2, temp);
    temp += g;
    bool hover_new = up.dot(temp) >= 0.0;
    if (hover_new != hover) {
      dts.push_back(dti);
      hover = hover_new;
      charts.push_back(hover_new);
    }
  }
  assert(hover == true);  // should end in hover
  // merge stuff in
  std::vector<decimal_t> dtts(1, 0.0);
  for (auto &t : times) dtts.push_back(t->getVal() + dtts.back());

  yaw_dts.clear();
  yaw_charts.clear();

  auto it1 = dts.begin();
  auto it2 = ++dtts.begin();

  for (auto p : charts) {
    std::cout << "raw chart " << p << std::endl;
  }
  for (auto p : dts) {
    std::cout << "chart dt " << p << std::endl;
  }
  for (auto p : dtts) {
    std::cout << "regular dt " << p << std::endl;
  }

  while (it1 != dts.end() && it2 != dtts.end()) {
    if (*it1 > *it2) {
      yaw_dts.push_back(*(it2++));
    } else {
      yaw_dts.push_back(*(it1++));
    }
  }
  while (it2 != dtts.end()) {
    yaw_dts.push_back(*(it2++));
  }
  while (it1 != dts.end()) {
    yaw_dts.push_back(*(it1++));
  }
  yaw_seg_ = yaw_dts.size();

  for (int i = 0; i < yaw_seg_; i++) {
    if (i == 0) {
      times_yaw.push_back(solver_yaw.addConstVar(yaw_dts.at(i)));
      yaw_charts.push_back(true);
    } else {
      times_yaw.push_back(
          solver_yaw.addConstVar(yaw_dts.at(i) - yaw_dts.at(i - 1)));
      int n = 0;
      for (int j = 0; j < dts.size(); j++)
        if (dts.at(j) >= yaw_dts.at(i) - 0.001) n++;
      yaw_charts.push_back(n % 2 == 0);
    }
  }
  for (auto p : times_yaw) {
    std::cout << "merged dt " << p->val << std::endl;
  }
}

}  // namespace traj_opt

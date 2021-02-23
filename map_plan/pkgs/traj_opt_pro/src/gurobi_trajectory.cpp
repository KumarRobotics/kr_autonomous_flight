// Copyright 2015 Michael Watterson
#include <gurobi_c++.h>
#include <traj_opt_pro/gurobi_trajectory.h>

#include <Eigen/Eigenvalues>
#include <boost/range/irange.hpp>

using boost::irange;

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace traj_opt {

TrajSection1D::TrajSection1D(GRBModel *model_, uint n_p_, uint k_r_,
                             decimal_t dt_,
                             boost::shared_ptr<BasisBundlePro> basis_)
    : n_p(n_p_), k_r(k_r_), dt(dt_), basis(basis_), model(model_) {
  generated = false;
  for (uint i = 0; i <= n_p; i++) {
    coeffs_var.push_back(
        model->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
  }
  // Integrate new variables
  model->update();
}
void TrajSection1D::getLinCost(GRBLinExpr &expr, uint i) {
  if (i > n_p - k_r)
    return;
  int index = i + k_r;
  decimal_t delta = std::pow(dt, 1 - 2 * static_cast<int>(k_r));
  expr = coeffs_var.at(index) * std::sqrt(delta);
}

void TrajSection1D::getCost(GRBQuadExpr &objective) {
  // adds cost using inner products
  decimal_t delta = std::pow(dt, 1 - 2 * static_cast<int>(k_r));

  if (basis->getBasis(k_r)->orthogonal()) {
    for (auto i : boost::irange(0, static_cast<int>(n_p + 1))) {
      decimal_t ip = basis->getBasis(k_r)->innerproduct(i, i);
      // std::cout << "ip of " << i << " is " << ip << std::endl;
      if (std::abs(ip) >= 1e-10)
        objective += delta * ip * coeffs_var.at(i) * coeffs_var.at(i);
    }
  } else {
    for (auto i : boost::irange(0, static_cast<int>(n_p + 1))) {
      for (auto j : boost::irange(0, static_cast<int>(n_p + 1))) {
        decimal_t ip = basis->getBasis(k_r)->innerproduct(i, j);
        // std::cout << "ip of " << i << " , " << j << " is " << ip  <<
        // std::endl;
        if (std::abs(ip) >= 1e-10)
          objective += delta * ip * coeffs_var.at(i) * coeffs_var.at(j);
      }
    }
  }
  //  std::cout << "cost " << objective << std::endl;
  // for (uint i = 0; i <= n_p - k_r; i++) {
  //  int index = i + k_r;
  //  objective += delta * coeffs_var.at(index) * coeffs_var.at(index);
  //}
}
decimal_t TrajSection1D::getCostDerr() {
  decimal_t objective = 0;
  // adds cost based on assumption of using a legendre basis
  for (uint i = 0; i <= n_p - k_r; i++) {
    int index = i + k_r;
    decimal_t ratio = (1 - 2 * static_cast<int>(k_r)) *
                      std::pow(dt, -2 * static_cast<int>(k_r));
    decimal_t delta = 1.0 / (1.0 + 2.0 * (decimal_t)i);
    //    std::cout << "delta " << delta << std::endl;
    objective += delta * ratio * coeffs.at(index) * coeffs.at(index);
  }
  return objective;
}
void TrajSection4D::getCost(GRBQuadExpr &objective) {
  // add cost from each dimension
  for (auto &sec : secs)
    sec->getCost(objective);
}
decimal_t TrajSection4D::getCostDerr() {
  decimal_t cost = 0;
  for (auto &sec : secs)
    cost += sec->getCostDerr();
  return cost;
}

void TrajSection1D::getContr(decimal_t x, uint derr, GRBLinExpr &expr) {
  for (uint i = 0; i <= n_p; i++)
    expr += basis->getVal(x, dt, i, derr) * coeffs_var.at(i);
}
void TrajSection4D::getContr(decimal_t x, uint derr, uint dim,
                             GRBLinExpr &expr) {
  secs.at(dim)->getContr(x, derr, expr);
}
// point_id is 0 to n_p
void TrajSection4D::getControl(uint point_id, uint dim, GRBLinExpr &expr,
                               bool derr) {
  auto &sec = secs.at(dim);
  const MatD *tf2;
  if (derr)
    tf2 = &basisD->getBasisBasisTransform();
  else
    tf2 = &basisT->getBasisBasisTransform();
  const MatD &tf(*tf2);

  //  std::cout << "D: " << tf << std::endl;
  //  std::cout << "T: " << tf << std::endl;

  for (uint i = 0; i <= sec->n_p; i++) {
    // robust non-zero threshholding
    if (std::abs(tf(point_id, i)) >
        5.0 * std::numeric_limits<decimal_t>::epsilon())
      expr += tf(point_id, i) * sec->coeffs_var.at(i);
  }
  //  std::cout << "expr " << expr << std::endl;
}

decimal_t TrajSection1D::evaluate(decimal_t t, uint derr) const {
  if (!generated)
    throw std::runtime_error("Trying to evaluate unoptimized trajectory");

  decimal_t rsn = 0;
  for (uint i = 0; i <= n_p; i++) {
    rsn += coeffs.at(i) * basis->getVal(t, dt, i, derr);
  }
  return rsn;
}

void TrajSection4D::addPathCost(const MatD &A, const VecD &b, double epsilon) {
  GRBQuadExpr cost;
  GRBLinExpr lin;
  double ddx = 0.1;

  //    // add new vars
  //    for (double dx = 0 ; dx <=1.0; dx+=ddx) {
  //        for(int c = 0 ; c < points.cols(); c++) {
  //            const_var.push_back( model->addVar(-GRB_INFINITY, GRB_INFINITY,
  //            0.0, GRB_CONTINUOUS));
  //        }
  //    }
  //    model->update();
  cost = model_->getObjective();
  double epsilon_i = epsilon / A.cols();

  int count = 0;
  for (double dx = 0; dx <= 1.0; dx += ddx) {
    for (int c = 0; c < A.rows(); c++) {
      GRBLinExpr row;
      for (int i = 0; i < 3; i++) {
        GRBLinExpr lin;
        getContr(dx, 0, i, lin);
        row += A(c, i) * lin;
      }
      //            model->addConstr(const_var.at(i) == row - b(c));
      cost += (row - b(c)) * (row - b(c)) * epsilon_i;
      count++;
    }
  }

  model_->setObjective(cost, GRB_MINIMIZE);
  model_->update();
}

void TrajSection4D::addLineCost(const Vec3 &p0, const Vec3 &p1,
                                double upsilon) {
  Vec3 u = p1 - p0;
  u.normalize();

  GRBQuadExpr cost = model_->getObjective();

  double ds = 0.1;

  for (double s = 0; s <= 1.0; s += ds) {
    std::vector<GRBLinExpr> e(3);
    GRBLinExpr emag;
    for (uint i = 0; i < 3; i++) {
      GRBLinExpr lin;
      getContr(s, 0, i, lin);
      e.at(i) = lin - p0(i);
      emag += e.at(i) * u(i);
    }
    for (uint i = 0; i < 3; i++) {
      GRBLinExpr lin;
      lin = e.at(i) - emag * u(i);
      cost += lin * lin * upsilon;
    }
  }
  model_->setObjective(cost, GRB_MINIMIZE);
  model_->update();
}

void TrajSection4D::evaluate(decimal_t t, uint derr, VecD &out) const {
  out == VecD::Zero(dim, 1);
  for (int i = 0; i < dim; i++)
    out(i, 0) = secs.at(i)->evaluate(t, derr);
}
TrajSection4D::TrajSection4D(GRBModel *model, uint n_p_, uint k_r, decimal_t dt,
                             boost::shared_ptr<BasisBundlePro> basis,
                             boost::shared_ptr<BasisTransformer> basisT_,
                             boost::shared_ptr<BasisTransformer> basisD_)
    : basisT(basisT_), basisD(basisD_), n_p(n_p_) {
  for (int i = 0; i < dim; i++)
    secs.push_back(boost::shared_ptr<TrajSection1D>(
        new TrajSection1D(model, n_p, k_r, dt, basis)));
  model_ = model;
  basis_ = basis;
  dt_ = dt;
}

void TrajSection1D::recoverVars(decimal_t ratio) {
  coeffs.clear();
  for (auto &var : coeffs_var)
    coeffs.push_back(var.get(GRB_DoubleAttr_X));
  generated = true;
}
void TrajSection4D::recoverVars(decimal_t ratio) {
  for (auto &sec : secs)
    sec->recoverVars(ratio);
}
void TrajSection1D::recoverVarsF(decimal_t ratio) {
  coeffs.clear();
  for (auto &var : coeffs_var)
    coeffs.push_back(var.get(GRB_DoubleAttr_UB));
  generated = true;
}
void TrajSection4D::recoverVarsF(decimal_t ratio) {
  for (auto &sec : secs)
    sec->recoverVarsF(ratio);
}
bool LegendreTrajectory::recoverVarsF(decimal_t ratio) {
  try {
    int i = 0;
    for (auto &sec : individual_sections) {
      sec->recoverVarsF(ratio);
      dts.at(i) /= ratio;
      sec->setDt(dts.at(i));
      i++;
    }

    return true;
  } catch (GRBException e) {
    std::cout << "Error: " << e.getMessage().c_str() << std::endl;
    return false;
  }
}

bool LegendreTrajectory::recoverVars(decimal_t ratio) {
  if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL ||
      model->get(GRB_IntAttr_Status) == GRB_SUBOPTIMAL) {
    int i = 0;
    for (auto &sec : individual_sections) {
      sec->recoverVars(ratio);
      dts.at(i) /= ratio;
      sec->setDt(dts.at(i));
      i++;
    }
    return true;
  } else {
    return false;
  }
}
bool LegendreTrajectory::evaluate(decimal_t t, uint derr,
                                  VecD &out) const { // returns false when out
  // of time range, but still
  // sets out to endpoint
  if (t < 0) {
    individual_sections.front()->evaluate(0.0, derr, out);
    return false;
  } else {
    // find appropriate section
    auto dt_it = dts.begin();
    for (auto &it : individual_sections) {
      if (t < *dt_it) {
        it->evaluate(t / (*dt_it), derr, out);
        return true;
      }
      t -= *dt_it;

      ++dt_it;
    }
    individual_sections.back()->evaluate(1.0, derr, out);
    return false;
  }
}

LegendreTrajectory::LegendreTrajectory(GRBModel *model_,
                                       const std::vector<decimal_t> dts_,
                                       int degree, PolyType basis_type,
                                       int order)
    : model(model_), dts(dts_) {
  uint n_p = degree;
  uint k_r = order;
  n_p_ = n_p;
  k_r_ = k_r;
  boost::shared_ptr<BasisBundlePro> basis;
  basis.reset(new BasisBundlePro(basis_type, n_p, k_r));

  if (basis_type == LEGENDRE) {
    basisT = boost::make_shared<BasisTransformer>(
        boost::make_shared<LegendreBasis>(n_p, k_r),
        boost::make_shared<BezierBasis>(n_p));
  } else if (basis_type == BEZIER) {
    basisT = boost::make_shared<BasisTransformer>(
        boost::make_shared<BezierBasis>(n_p),
        boost::make_shared<BezierBasis>(n_p));
  } else if (basis_type == STANDARD) {
    basisT = boost::make_shared<BasisTransformer>(
        boost::make_shared<StandardBasis>(n_p),
        boost::make_shared<BezierBasis>(n_p));
  } else if (basis_type == ENDPOINT) {
    basisT = boost::make_shared<BasisTransformer>(
        boost::make_shared<EndPointBasis>(n_p),
        boost::make_shared<BezierBasis>(n_p));
  } else if (basis_type == CHEBYSHEV) {
    basisT = boost::make_shared<BasisTransformer>(
        boost::make_shared<ChebyshevBasis>(n_p),
        boost::make_shared<BezierBasis>(n_p));
  }

  // std::cout << "Matrix " << basisT->getBasisBasisTransform() << std::endl;
  // this basis is a jerk
  //  basisD = boost::make_shared<BasisTransformer>(
  //      boost::dynamic_pointer_cast<StandardBasis>(basis->getBasis(3)),
  //      boost::make_shared<BezierBasis>(n_p - 3), 3);
  basisD = basisT;
  //  ROS_INFO_STREAM("Basis bundle " << *basis);
  uint num_secs = dts_.size();

  if (num_secs <= 0)
    throw std::runtime_error("Not enough waypoints to define trajectory");
  if (num_secs != dts.size())
    throw std::runtime_error(
        "Number of waypoints and list of dt dimension mismatch");

  // create space for sections
  for (uint i = 0; i < num_secs; i++) {
    individual_sections.push_back(boost::shared_ptr<TrajSection4D>(
        new TrajSection4D(model, n_p, k_r, dts.at(i), basis, basisT, basisD)));
  }
  linkSections();
}
void LegendreTrajectory::setLinearCost() {
  Eigen::Matrix<decimal_t, 32, 5> magic;
  magic << -0.4661, 0.0155, 0.6153, -0.6311, -0.0755, 0.3078, 0.0002, 0.5160,
      0.7876, 0.1366, 0.4278, 0.2790, 0.1794, -0.8214, -0.1798, -0.6869, 0.3349,
      0.4380, 0.4364, -0.1837, -0.1792, 0.5520, -0.1327, 0.4666, 0.6541,
      -0.5285, 0.3192, -0.6932, 0.2798, -0.2447, 0.6869, -0.3349, -0.4380,
      -0.4364, 0.1836, 0.0416, 0.2327, 0.0446, -0.6009, 0.7623, -0.4263,
      -0.3475, 0.4760, 0.1624, 0.6668, -0.1316, -0.7076, 0.0584, -0.6108,
      0.3250, 0.5285, -0.3192, 0.6933, -0.2798, 0.2447, -0.2190, 0.9150, 0.0066,
      -0.3270, -0.0881, 0.4812, -0.2081, -0.0075, 0.2432, 0.8160, -0.4277,
      -0.2790, -0.1794, 0.8214, 0.1798, 0.4661, -0.0155, -0.6153, 0.6311,
      0.0755, 0.1792, -0.5521, 0.1326, -0.4666, -0.6541, -0.3078, -0.0002,
      -0.5160, -0.7876, -0.1365, -0.0126, 0.5204, 0.7596, 0.0073, 0.3899,
      0.0125, -0.5203, -0.7596, -0.0073, -0.3899, 0.8532, -0.2410, 0.1121,
      0.2280, -0.3865, -0.2679, -0.3632, -0.6553, 0.0058, 0.6057, 0.7756,
      0.5803, 0.0846, 0.0243, 0.2321, -0.0416, -0.2326, -0.0446, 0.6009,
      -0.7623, 0.2679, 0.3632, 0.6553, -0.0059, -0.6057, -0.7756, -0.5804,
      -0.0846, -0.0243, -0.2320, -0.4813, 0.2080, 0.0075, -0.2432, -0.8160,
      -0.1999, -0.4993, 0.7582, 0.2027, -0.3079, 0.2190, -0.9151, -0.0067,
      0.3269, 0.0881, 0.4263, 0.3475, -0.4759, -0.1625, -0.6668, -0.8532,
      0.2411, -0.1121, -0.2280, 0.3865, 0.1999, 0.4993, -0.7582, -0.2028,
      0.3079, 0.1317, 0.7076, -0.0584, 0.6107, -0.3249;
  Eigen::Matrix<decimal_t, 12, 5> magic2;
  magic2 << -0.2787, -0.8301, -0.2839, 0.3890, 0.0378, -0.6238, 0.2216, 0.5938,
      0.4552, 0.0444, 0.2852, -0.5361, 0.5579, -0.5492, 0.1357, -0.0434, 0.4346,
      -0.6550, 0.3722, 0.4916, -0.1930, 0.6884, 0.0735, -0.6953, -0.0000,
      0.6428, 0.6569, 0.3829, 0.0680, -0.0640, 0.5898, -0.2816, 0.1620, 0.7382,
      -0.0406, -0.0722, -0.2295, 0.3994, -0.1651, -0.8691, -0.7624, -0.2306,
      -0.3386, -0.4970, 0.0629, -0.1686, 0.3543, -0.4978, 0.3429, -0.6933,
      0.6133, -0.1976, -0.6264, -0.4321, -0.0758, 0.0112, -0.0507, 0.2327,
      -0.0273, 0.9708;

  GRBLinExpr cost;
  //  std::cout << "magic matrix " << magic << std::endl;
  std::vector<GRBVar> slack_vars;
  for (auto &sec : individual_sections)
    for (uint i = 0; i < sec->secs.size(); i++)
      slack_vars.push_back(model->addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS));
  model->update();

  int i = 0;
  for (auto &sec : individual_sections) {
    for (auto &subsec : sec->secs) {
      for (int k = 0; k < 12; k++) {
        GRBLinExpr rowcontr;
        for (int j = 0; j <= n_p_ - k_r_; j++) {
          GRBLinExpr varconstr;
          subsec->getLinCost(varconstr, j);
          rowcontr += varconstr * magic2(k, j);
        } // end j
        model->addConstr(rowcontr <= slack_vars.at(i));
        //        std::cout << "constr " << rowcontr << std::endl;
      } // end k
      cost += slack_vars.at(i);
      i++;
    } // end auto sub sec
  }   // end auto sec

  //  std::cout << "cost " << cost << std::endl;
  model->setObjective(cost, GRB_MINIMIZE);
}
void LegendreTrajectory::setQuadraticCost() {
  GRBQuadExpr cost;
  for (auto &sec : individual_sections)
    sec->getCost(cost);
  model->setObjective(cost, GRB_MINIMIZE);
}

void LegendreTrajectory::setSlackCost() {
  GRBLinExpr cost;
  for (auto &sec : individual_sections) {
    auto &sec1d = sec->secs.back(); // only set cost on last param
    for (uint i = 0; i < sec1d->coeffs_var.size(); i++) {
      cost +=
          sec1d->basis->getVal(1.0, sec1d->dt, i, -1) * sec1d->coeffs_var.at(i);
    }
  }
  model->setObjective(cost, GRB_MINIMIZE);
}

void LegendreTrajectory::linkSections() {
  // set right goals
  for (uint i = 0; i < individual_sections.size() - 1; i++) {
    for (int j = 0; j < dim; j++) {
      // derrivatives
      for (int k = 0; k < k_r_; k++) {
        //        decimal_t constr = waypoints[i + 1](j, k);
        GRBLinExpr left;
        individual_sections.at(i)->getContr(1.0, k, j, left);
        GRBLinExpr right;
        individual_sections.at(i + 1)->getContr(0.0, k, j, right);
        model->addConstr(left == right);
        // adds hard coded constraints only if they aren't nan
        //        if (!std::isnan(constr)) model->addConstr(left == constr);
      }
    }
  }
  model->update();
}
void LegendreTrajectory::addWayPointConstrains(
    const std::vector<Waypoint> &waypnts, bool use_v0) {
  waypnts_ = waypnts;
  int num_secs = individual_sections.size();
  for (auto &waypnt : waypnts) {
    // check bounds
    if (waypnt.knot_id > num_secs)
      throw std::out_of_range("knot id is too large");
    else if (waypnt.knot_id < -(num_secs + 1))
      throw std::out_of_range("knot id is too small");
    // change id to section number
    int sec_id = 0;
    if (waypnt.knot_id >= 0)
      sec_id = waypnt.knot_id;
    else
      sec_id = num_secs + waypnt.knot_id + 1;

    // get grb constraints
    decimal_t valuation = 0; // where to evaluate section
    if (sec_id == num_secs) {
      valuation = 1.0;
      sec_id--;
    }
    //    ROS_INFO_STREAM("sec_id " << sec_id << " knot id " << waypnt.knot_id
    //                              << " num secs " << num_secs);
    Vec3 normalized_v0 = waypnt.vel.block<3, 1>(0, 0);
    normalized_v0.normalize();
    GRBLinExpr vel_expr;
    if (use_v0) {
      for (auto col : boost::irange(0, 3)) {
        GRBLinExpr expr;
        individual_sections.at(0)->getContr(valuation, 0, col, expr);
        vel_expr += expr * normalized_v0(col);
      }
    }

    for (int col = 0; col < dim; col++) {
      if (waypnt.use_pos && !std::isnan(waypnt.pos(col))) {
        GRBLinExpr expr;
        individual_sections.at(sec_id)->getContr(valuation, 0, col, expr);
        model->addConstr(expr == waypnt.pos(col));
      }
      if (waypnt.use_vel && !std::isnan(waypnt.vel(col))) {
        // if(!use_v0 || sec_id!=0){
        if (true) {
          GRBLinExpr expr;
          individual_sections.at(sec_id)->getContr(valuation, 1, col, expr);
          model->addConstr(expr == waypnt.vel(col));
        } else {
          GRBLinExpr expr;
          individual_sections.at(sec_id)->getContr(valuation, 1, col, expr);
          model->addConstr(expr == vel_expr * normalized_v0(col));
        }

        //        ROS_INFO_STREAM("Vel constr " << expr << " = " <<
        //        waypnt.vel(col));
      }
      if (waypnt.use_acc && !std::isnan(waypnt.acc(col))) {
        GRBLinExpr expr;
        individual_sections.at(sec_id)->getContr(valuation, 2, col, expr);
        model->addConstr(expr == waypnt.acc(col));
      }
      if (waypnt.use_jrk && !std::isnan(waypnt.jrk(col))) {
        GRBLinExpr expr;
        individual_sections.at(sec_id)->getContr(valuation, 3, col, expr);
        model->addConstr(expr == waypnt.jrk(col));
      }
    }
  }
}

void LegendreTrajectory::addMaximumBound(decimal_t bound, uint derr) {
  // use square to approimate circle, TODO have option for arbitray shape
  static decimal_t r2o2 = 0.7071067811865476;
  decimal_t side = bound * r2o2;

  for (auto &sec : individual_sections) {
    for (uint dim = 0; dim < 3; dim++) {
      decimal_t dt = 1.0 / decimal_t(num_sample_);
      for (decimal_t t = 0.0; t < 1.0; t += dt) {
        // hard code inforcing contraints at 10 points along sections
        GRBLinExpr expr;
        if (con_mode_ != CHEBY) {
          sec->getContr(t, derr, dim, expr);
        } else {
          decimal_t s = 0.5 - 0.5 * std::cos(3.1415926354 * t);
          sec->getContr(s, derr, dim, expr);
        }
        model->addConstr(expr <= side);
        model->addConstr(expr >= -side);
      }
    }
  }
}
decimal_t LegendreTrajectory::getTotalTime() const {
  decimal_t t = 0;
  for (auto &dt : dts)
    t += dt;
  //  ROS_WARN("Getting total time");
  return t;
}
void LegendreTrajectory::addVolumeContraints(const Mat4Vec &constr) {
  if (constr.size() != individual_sections.size())
    throw std::runtime_error(
        "Number of constraints does not match number of segments");
  for (uint i = 0; i < constr.size(); i++) {
    auto &con = constr.at(i);
    auto &sec = individual_sections.at(i);
    // hard code inforcing contraints at 10 points along sections
    for (decimal_t t = 0.0; t < 1.0; t += 0.1) {
      for (uint r = 0; r < 4; r++) {
        GRBLinExpr expr;
        for (uint c = 0; c < 3; c++) {
          GRBLinExpr subexpr;
          sec->getContr(t, 0, c, subexpr);
          subexpr *= con(r, c);
          expr += subexpr;
        }
        model->addConstr(expr >= -con(r, 3));
      } // end r
    }   // end t
  }     // end i
}
void TrajSection4D::setDt(decimal_t dt) {
  for (auto &sec : secs)
    sec->setDt(dt);
  dt_ = dt;
}

decimal_t LegendreTrajectory::getCost() {
  return model->get(GRB_DoubleAttr_ObjVal);
}
void LegendreTrajectory::addAxbConstraints(const std::vector<MatD> &A,
                                           const std::vector<VecD> &b,
                                           const std::vector<decimal_t> &ds) {
  // check bounds
  if (A.size() != individual_sections.size()) {
    printf("A: %zu, segments: %zu\n", A.size(), individual_sections.size());
    throw std::runtime_error(
        "Number of constraints does not match number of segments");
  }

  A_ = A;
  b_ = b;
  for (uint i = 0; i < A.size(); i++) {
    const MatD &conA = A.at(i);
    const VecD &conb = b.at(i);
    auto &sec = individual_sections.at(i);

    if (con_mode_ == SAMPLE) {
      decimal_t dt = 1.0 / decimal_t(num_sample_);
      for (decimal_t t = 0.0; t < 1.0; t += dt) {
        // hard code inforcing contraints at 10 points along sections
        for (uint r = 0; r < conb.rows(); r++) {
          GRBLinExpr expr;
          for (uint c = 0; c < 3; c++) {
            GRBLinExpr subexpr;
            sec->getContr(t, 0, c, subexpr);
            subexpr *= conA(r, c);
            expr += subexpr;
          }
          //                std::cout << "expr " << expr << std::endl;
          model->addConstr(expr <= conb(r));
        } // end r
      }   // end t
      // std::cout << "using sampling method " << std::endl;

    } else if (con_mode_ == CONTROL) {
      // do this by control points

      for (int j = 0; j <= n_p_; j++) {
        for (uint r = 0; r < conb.rows(); r++) {
          GRBLinExpr expr;
          for (uint c = 0; c < 3; c++) {
            GRBLinExpr subexpr;
            sec->getControl(j, c, subexpr, false);
            subexpr *= conA(r, c);
            expr += subexpr;
          }
          //          std::cout << "expr " << expr << std::endl;
          // if (i == 0 && r <= 3) {
          // } else {
          model->addConstr(expr <= conb(r));
          // }
          // std::cout << "ignoring first control points: expr = "<< expr <<
          // std::endl;
        }
      }
    } else if (con_mode_ == CHEBY) {
      decimal_t dt = 1.0 / decimal_t(num_sample_);
      for (decimal_t t = 0.0; t < 1.0; t += dt) {
        decimal_t s = 0.5 - 0.5 * std::cos(3.1415926354 * t);
        for (uint r = 0; r < conb.rows(); r++) {
          GRBLinExpr expr;
          for (uint c = 0; c < 3; c++) {
            GRBLinExpr subexpr;
            sec->getContr(s, 0, c, subexpr);
            subexpr *= conA(r, c);
            expr += subexpr;
          }
          //                std::cout << "expr " << expr << std::endl;
          model->addConstr(expr <= conb(r));
        } // end r
      }   // end t
    } else if (con_mode_ == ELLIPSE) {
      for (int j = 0; j <= n_p_; j++) {
        GRBQuadExpr expr;
        for (uint r = 0; r < 3; r++) {
          for (uint c = r; c < 3; c++) {
            GRBLinExpr subexpr_c, subexpr_r;
            sec->getControl(j, c, subexpr_c, false);
            sec->getControl(j, r, subexpr_r, false);
            if (c == r)
              expr +=
                  (subexpr_r - conb(r)) * conA(r, c) * (subexpr_c - conb(c));
            else
              expr += 2.0 * (subexpr_r - conb(r)) * conA(r, c) *
                      (subexpr_c - conb(c));
          }
        }
        model->addQConstr(expr <= 1.0);
      }
    } // end con mode
  }   // end i
}

// empty detructors
LegendreTrajectory::~LegendreTrajectory() {}
TrajSection1D::~TrajSection1D() {}
TrajSection4D::~TrajSection4D() {}

bool LegendreTrajectory::adjustTimes(decimal_t epsilon) {
  // returns if cost is increasing
  static bool cost_diverging = false;
  static decimal_t old_cost = 1000000000000000;

  bool success = recoverVars();
  if (!success || cost_diverging) {
    // check if proceedure iterated
    if (old_dts.size() == 0)
      return false;

    // revert model
    int numConst = model->get(GRB_IntAttr_NumConstrs);
    GRBConstr *consts = model->getConstrs();

    for (int i = 0; i < numConst; i++) {
      model->remove(consts[i]);
    }
    delete consts;
    model->update();

    for (uint i = 0; i < old_dts.size(); i++) {
      individual_sections.at(i)->setDt(old_dts.at(i));
    }
    linkSections();
    addWayPointConstrains(waypnts_, false);
    addAxbConstraints(A_, b_, old_dts);
    //    setQuadraticCost();
    setLinearCost();
    // replate dts
    dts = old_dts;
    // rerun
    model->optimize();
    recoverVars();

    // reset static variables
    old_cost = 1000000000000000;
    cost_diverging = false;
    return false;
  }
  decimal_t new_cost = getCost();
  std::cout << "Cost is " << new_cost << std::endl;

  if (new_cost > old_cost) {
    cost_diverging = true;
    return true;
  } else {
    old_cost = new_cost;
  }

  int sz = dts.size();
  VecD dcost(sz, 1);
  VecD planN(sz, 1);
  planN.setOnes();

  // get derrivative of cost with respect to segment lengths
  for (int i = 0; i < sz; i++) {
    dcost(i) = -individual_sections.at(i)->getCostDerr();
  }

  //  ROS_INFO_STREAM("Cost Vector pre norm " << dcost.transpose());
  //  std::cout << "Cost Vector pre norm " << dcost.transpose() << std::endl;

  planN.normalize();
  dcost.normalize();
  //  ROS_INFO_STREAM("Cost Vector " << dcost.transpose());
  //  std::cout << "Cost Vector post norm " << dcost.transpose() << std::endl;

  decimal_t dd = planN.dot(dcost);
  dcost -= dd * planN;

  //  ROS_INFO_STREAM("Cost Vector " << dcost.transpose());
  //  std::cout << "Cost Vector post adjustment " << dcost.transpose() <<
  //  std::endl;

  //  decimal_t sum = dcost.sum();
  //  ROS_WARN_STREAM("sum of dcost " << sum);
  old_dts = dts;
  for (int i = 0; i < sz; i++) {
    dts.at(i) += dcost(i) * epsilon;
    individual_sections.at(i)->setDt(dts.at(i));
  }

  // remove old constraints
  int numConst = model->get(GRB_IntAttr_NumConstrs);
  GRBConstr *consts = model->getConstrs();

  for (int i = 0; i < numConst; i++) {
    model->remove(consts[i]);
  }
  delete consts;
  model->update();

  linkSections();
  addWayPointConstrains(waypnts_, false);
  addAxbConstraints(A_, b_, dts);
  //  setQuadraticCost();
  setLinearCost();

  return true;
}
void LegendreTrajectory::resetConstr(bool lp) {
  int numConst = model->get(GRB_IntAttr_NumConstrs);
  GRBConstr *consts = model->getConstrs();

  for (int i = 0; i < numConst; i++) {
    model->remove(consts[i]);
  }
  delete consts;
  model->update();

  linkSections();
  addWayPointConstrains(waypnts_, false);
  addAxbConstraints(A_, b_, dts);
  //  setQuadraticCost();
  setCost(lp);
  model->update();
}

VecD LegendreTrajectory::check_max_violation(decimal_t max_vel,
                                             decimal_t max_acc,
                                             decimal_t max_jrk) {
  decimal_t dt = 0.01;
  decimal_t ma = 0.0, mv = 0.0, mj = 0.0, mp = 0.0;
  for (decimal_t t = 0.0; t <= getTotalTime(); t += dt) {
    VecD temp;
    evaluate(t, 0, temp);
    //    std::cout << "Pos " << temp.transpose() << std::endl;
    evaluate(t, 1, temp);
    mv = std::max(mv, temp.norm());
    evaluate(t, 2, temp);
    ma = std::max(ma, temp.norm());
    evaluate(t, 3, temp);
    mj = std::max(mj, temp.norm());
  }
  int i = 0;
  for (auto &sec : individual_sections) {
    VecD pos;
    for (decimal_t t = 0.0; t <= 1.0; t += 0.01) {
      sec->evaluate(t, 0, pos);
      VecD err = A_.at(i) * pos.block<3, 1>(0, 0) - b_.at(i);
      mp = std::max(mp, err.maxCoeff());
    }
    i++;
  }

  decimal_t rv = std::max(mv - max_vel, 0.0);
  decimal_t ra = std::max(ma - max_acc, 0.0);
  decimal_t rj = std::max(mj - max_jrk, 0.0);
  VecD res;
  res << mp, rv, ra, rj;
  return res;
}
bool LegendreTrajectory::check_max(decimal_t max_vel, decimal_t max_acc,
                                   decimal_t max_jrk) {
  decimal_t dt = 0.01;
  decimal_t ma = 0.0, mv = 0.0, mj = 0.0;
  for (decimal_t t = 0.0; t <= getTotalTime(); t += dt) {
    VecD temp;
    evaluate(t, 0, temp);
    //    std::cout << "Pos " << temp.transpose() << std::endl;
    evaluate(t, 1, temp);
    mv = std::max(mv, temp.norm());
    evaluate(t, 2, temp);
    ma = std::max(ma, temp.norm());
    evaluate(t, 3, temp);
    mj = std::max(mj, temp.norm());
  }
  if (mv > max_vel || ma > max_acc || mj > max_jrk) {
    std::cout << "violated! with mv: " << mv << " ma: " << ma << " mj: " << mj
              << std::endl;
    std::cout << "limits are mv: " << max_vel << " ma: " << max_acc
              << " mj: " << max_jrk << std::endl;
    return false;
  }
  return true;
}
void LegendreTrajectory::adjustTimeToMaxs(decimal_t max_vel, decimal_t max_acc,
                                          decimal_t max_jrk, bool use_v0,
                                          decimal_t v0) {
  decimal_t dt = 0.01;
  decimal_t ma = 0.0, mv = 0.0, mj = 0.0;
  for (decimal_t t = 0.0; t <= getTotalTime(); t += dt) {
    VecD temp;
    evaluate(t, 0, temp);
    //    std::cout << "Pos " << temp.transpose() << std::endl;
    evaluate(t, 1, temp);
    mv = std::max(mv, temp.norm());
    evaluate(t, 2, temp);
    ma = std::max(ma, temp.norm());
    evaluate(t, 3, temp);
    mj = std::max(mj, temp.norm());
  }
  decimal_t rv = max_vel / mv;
  decimal_t ra = std::pow(max_acc / ma, 0.5);
  decimal_t rj = std::pow(max_jrk / mj, 0.3333);

  // find the constraint least violated
  decimal_t ratio = std::min(std::min(rv, ra), rj);
  // decimal_t ratio = std::min(rv, ra);

  // adjust times
  decimal_t t_tot = 0;
  std::vector<decimal_t> cum_t(1, 0.0);
  for (uint i = 0; i < dts.size(); i++) {
    dts.at(i) /= ratio;
    individual_sections.at(i)->setDt(dts.at(i));
    t_tot += dts.at(i);
    cum_t.push_back(t_tot);
  }

  if (use_v0) {
    decimal_t a, b;
    VecD temp;
    evaluate(0, 1, temp);
    b = v0 / temp.norm();
    if (b > 2)
      std::cout << "composite adjustment failed" << std::endl;
    a = 1.0 - b;

    std::cout << "Comp adjustment a: " << a << " b: " << b
              << " ratio: " << ratio << std::endl;
    std::vector<decimal_t> polyc(4, 0.0);
    // (a(t + cum_time) - cum_time)/dti
  }
  std::cout << "Max velocity " << ratio * mv << std::endl;
  std::cout << "Max acceleration " << ratio * ratio * ma << std::endl;
  std::cout << "Max jerk " << ratio * ratio * ratio * mj << std::endl;
}
Poly TrajSection1D::getBoostPoly() {
  Poly base;
  //  ROS_INFO_STREAM("n_p " << n_p << " coeff " << coeffs.size());
  for (int i = 0; i < static_cast<int>(coeffs.size()); i++) {
    //    ROS_INFO_STREAM("Coeff " << coeffs.at(i));
    base += coeffs.at(i) * boost::dynamic_pointer_cast<StandardBasis>(
                               basis->derrivatives.front())
                               ->getPoly(i);
  }
  //  ROS_INFO_STREAM("base " << base);

  return base;
}
TrajData LegendreTrajectory::serialize() {
  // dump all coefficients to msg
  if (!use_time_composite) {
    TrajData traj(individual_sections.front()->secs.size(), dts.size(), n_p_);

    for (int dm = 0; dm < traj.dimensions; dm++) {
      for (uint si = 0; si < dts.size(); si++) {
        Poly poly = individual_sections.at(si)->secs.at(dm)->getBoostPoly();
        for (uint ci = 0; ci <= uint(n_p_); ci++) {
          traj.data.at(dm).segs.at(si).coeffs.at(ci) =
              poly.size() > ci ? poly[ci] : 0;
        }
        traj.data.at(dm).segs.at(si).dt = dts.at(si);
      }
      traj.data.at(dm).t_total = getTotalTime();
    }
    return traj;
  } else {
    TrajData traj(individual_sections.front()->secs.size(), dts.size(),
                  n_p_ * 3);
    for (int dm = 0; dm < traj.dimensions; dm++) {
      for (uint si = 0; si < dts.size(); si++) {
        Poly poly = individual_sections.at(si)->secs.at(dm)->getBoostPoly();
        decimal_t one = 1.0;
        Poly id(&one, 0);
        Poly poly_conv;
        for (int ci = 0; ci <= n_p_; ci++) {
          poly_conv += poly[ci] * id;
          id *= time_composite.at(si);
        }
        for (int ci = 0; ci <= n_p_ * 3; ci++) {
          traj.data.at(dm).segs.at(si).coeffs.at(ci) = poly_conv[ci];
        }

        traj.data.at(dm).segs.at(si).dt = dts.at(si);
      }
      traj.data.at(dm).t_total = getTotalTime();
      return traj;
    }
  }
  return TrajData(0, 0, 0);
}
void LegendreTrajectory::setCost(bool lp) {
  if (lp)
    setLinearCost();
  else
    setQuadraticCost();
}

} // namespace traj_opt

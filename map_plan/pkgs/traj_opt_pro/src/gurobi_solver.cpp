// Copyright 2015 Michael Watterson
#include <gurobi_c++.h>
#include <traj_opt_pro/gurobi_solver.h>
#include <traj_opt_pro/timers.h>

#include <boost/range/irange.hpp>

using boost::irange;
#include <algorithm>
#include <vector>

namespace traj_opt {
void GurobiSolver::sikangSeed(boost::shared_ptr<Vec3Vec> points,
                              std::vector<decimal_t> *dsp, decimal_t v0,
                              decimal_t ratio) {
  decimal_t v_max = v_max_;
  decimal_t a_max = a_max_;
  decimal_t ini_v = v0;
  decimal_t end_v = 0;
  const Vec3Vec &waypoints(*points);

  decimal_t acc;
  if (ini_v > v_max)
    acc = -a_max;
  else
    acc = a_max;

  int ini_sign = 1;
  if (ini_v < 0) ini_sign = -1;

  if (end_v > v_max) {
    printf("end_v = %f too large, set it to %f", end_v, v_max);
    end_v = v_max;
  }
#if TIME_ALLOCATE_VERBOSE
  printf("t_alloc: ini_v: %f, end_v: %f, v_max: %f, a: %f\n", ini_v, end_v,
         v_max, a_max);
#endif

  std::vector<decimal_t> waypoint_times;
  dsp->clear();
  waypoint_times.push_back(0);

  std::vector<decimal_t> dists;
  decimal_t total_dist = 0;
  for (unsigned int i = 1; i < waypoints.size(); i++) {
    decimal_t dist = (waypoints[i] - waypoints[i - 1]).norm();
    total_dist += dist;
    dists.push_back(total_dist);
  }

  decimal_t acc_d_thr = (v_max * v_max - ini_sign * ini_v * ini_v) / (2 * acc);

  decimal_t dec_d_thr = (v_max * v_max - end_v * end_v) / (2 * a_max);

  if (total_dist > (acc_d_thr + dec_d_thr)) {
    decimal_t max_speed = v_max;
    for (const auto &d : dists) {
      decimal_t t;
      if (d < acc_d_thr) {
        t = (sqrt(ini_sign * ini_v * ini_v + 2 * acc * d) - ini_v) / acc;
      } else if (d >= acc_d_thr && d < total_dist - dec_d_thr) {
        t = (max_speed - ini_v) / acc + (d - acc_d_thr) / max_speed;
      } else if (d >= total_dist - dec_d_thr && d <= total_dist) {
        decimal_t ep =
            max_speed * max_speed - 2 * a_max * (d - (total_dist - dec_d_thr));
        if (ep < 0) ep = 0;
        t = (max_speed - ini_v) / acc +
            (total_dist - acc_d_thr - dec_d_thr) / max_speed +
            (max_speed - sqrt(ep)) / a_max;
      } else {
        printf("wrong time allocate!!\n\n");
        break;
      }
      waypoint_times.push_back(t);
    }
  } else {
    decimal_t max_speed = sqrt(
        0.5 * (ini_sign * ini_v * ini_v + end_v * end_v) + a_max * total_dist);

    acc_d_thr = (max_speed * max_speed - ini_sign * ini_v * ini_v) / (2 * acc);
    dec_d_thr = (max_speed * max_speed - end_v * end_v) / (2 * a_max);

#if TIME_ALLOCATE_VERBOSE
    printf(ANSI_COLOR_BLUE "total dist smaller than acc dec dist!!\n");

    printf("max speed: %f, acc_d_thr: %f, dec_d_thr: %f\n" ANSI_COLOR_RESET,
           max_speed, acc_d_thr, dec_d_thr);
#endif

    for (const auto &d : dists) {
      decimal_t t;
      if (d < acc_d_thr)
        t = (sqrt(ini_sign * ini_v * ini_v + 2 * acc * d) - ini_v) / acc;
      else if (d >= acc_d_thr && d <= total_dist)
        t = (max_speed - ini_v) / acc +
            (max_speed -
             sqrt(std::fmax(max_speed * max_speed - 2 * a_max * (d - acc_d_thr),
                            0.0))) /
                a_max;
      waypoint_times.push_back(t);
    }
  }
  // return waypoint_times;
  for (unsigned int i = 1; i < waypoint_times.size(); i++) {
    dsp->push_back((waypoint_times[i] - waypoint_times[i - 1]) * ratio);
  }

  //  std::cout << "waypoints size " << waypoint_times.size() << std::endl;
}

void GurobiSolver::seedDs(boost::shared_ptr<Vec3Vec> points,
                          std::vector<decimal_t> *dsp) {
  // use hurristic
  if (points == NULL) return;
  std::vector<decimal_t> &ds(*dsp);

  decimal_t dx = 0;
  std::vector<decimal_t> cum_points, cum_ds;
  // std::cout << "xi ";
  for (auto i : irange(1, static_cast<int>(points->size()))) {
    dx += (points->at(i) - points->at(i - 1)).norm();
    cum_points.push_back(dx);
    // std::cout << dx << std::endl;
  }

  if (dx < v_max_ * v_max_ / a_max_) {
    // trap doesn't fully go up have to segs
    decimal_t peak_v = std::sqrt(dx * a_max_);
    decimal_t t_tot = 2.0 * dx / peak_v;
    for (auto &xi : cum_points) {
      if (xi < dx / 2.0)
        cum_ds.push_back(std::sqrt(2.0 * xi / a_max_));
      else
        cum_ds.push_back(t_tot - std::sqrt(2.0 * (dx - xi) / a_max_));
    }
  } else {
    decimal_t t_tot = dx / v_max_ + v_max_ / a_max_;
    decimal_t x_ris = 0.5 * v_max_ * v_max_ / a_max_;
    for (auto &xi : cum_points) {
      if (xi < x_ris)
        cum_ds.push_back(std::sqrt(2.0 * xi / a_max_));
      else if (xi < dx - x_ris)
        cum_ds.push_back(xi / v_max_ + 0.5 * v_max_ / a_max_);
      else
        cum_ds.push_back(t_tot - std::sqrt(2.0 * (dx - xi) / a_max_));
    }
  }
  ds.clear();
  ds.push_back(cum_ds.front());
  for (auto i : irange(1, static_cast<int>(cum_ds.size()))) {
    ds.push_back(cum_ds.at(i) - cum_ds.at(i - 1));
  }

  // std::cout << "ds ";
  for (auto &s : ds) s *= 3.0;
  //   std::cout << s << " , ";
  // std::cout << std::endl;
}

bool GurobiSolver::initializeGurobi() {
  // check if environment pointer is null
  if (grb_env_ == boost::shared_ptr<GRBEnv>()) {
    bool success = false;
    try {
      grb_env_.reset(new GRBEnv);
      success = true;
    } catch (GRBException e) {
      std::cout << e.getMessage().c_str() << std::endl;
    } catch (...) {
      std::cout << "Unkown error during contruction of trajectory tracker"
                << std::endl;
    }
    return success;
  } else {
    return true;
  }
}

GurobiSolver::GurobiSolver() {
  trajectory_solved_ = false;
  v_max_ = 2.0;
  a_max_ = 5.0;
  j_max_ = 50.0;
  time_its_ = 0;
  time_eps_ = -0.10;
  degree_ = 7;
  order_ = 3;
  polytype_ = LEGENDRE;
  con_mode_ = CONTROL;
  num_sample_ = 10;
}
boost::shared_ptr<Trajectory> GurobiSolver::getTrajectory() {
  return ltraj_;
  //  if (trajectory_solved_) {
  //  } else
  //    return traj_;
}
void GurobiSolver::addPathCost(const std::vector<MatD> &A,
                               const std::vector<VecD> &b, double epsilon) {
  if (A.size() == 0) return;
  if ((ltraj_->individual_sections.size()) != A.size()) {
    std::cout << "Un-equal path and traj size" << std::endl;
    return;
  }
  for (uint i = 0; i < A.size(); i++) {
    ltraj_->individual_sections.at(i)->addPathCost(A.at(i), b.at(i), epsilon);
  }
}
void GurobiSolver::addLineCost(boost::shared_ptr<Vec3Vec> points,
                               double upsilon) {
  if (points == NULL) return;
  if (upsilon == 0) return;  // oops you forgot to set upsilon

  for (uint i = 1; i < points->size(); i++) {
    ltraj_->individual_sections.at(i - 1)->addLineCost(points->at(i - 1),
                                                       points->at(i), upsilon);
  }
}
void GurobiSolver::setPolyParams(int degree, PolyType type, int order) {
  degree_ = degree;
  polytype_ = type;
  order_ = order;
}

bool GurobiSolver::solveTrajectory(
    const std::vector<Waypoint> &waypnts, const std::vector<MatD> &A,
    const std::vector<VecD> &b, const std::vector<decimal_t> &ds2,
    decimal_t epsilon, boost::shared_ptr<Vec3Vec> points, decimal_t upsilon) {
  std::vector<decimal_t> ds(ds2);
  trajectory_solved_ = false;
  cost_ = 100000000000000000.0;
  // std::cout << "Using gurobi solver" << std::endl;
  // basic checks
  if (!initializeGurobi()) {
    std::cout << "Gurobi not initialized. Did you setup license?" << std::endl;
    return false;
  }
  waypoints_ = waypnts;
  if (A.size() != b.size()) {
    std::cout << "Constraint vectors should be the same length" << std::endl;
    return false;
  } else if (A.size() != ds.size()) {
    std::cout << "ds should be same length as constraints" << std::endl;
    ds = std::vector<decimal_t>(A.size(), 1.0);
  }
  try {
    grb_env_->set(GRB_IntParam_OutputFlag, 0);       // turn off output
    grb_env_->set(GRB_DoubleParam_TimeLimit, 0.15);  // execution time limit
    GRBModel model(*grb_env_);

    bool use_vel_contr = false;
    decimal_t v0;

    v0 = waypnts.front().vel.norm();
    use_vel_contr = true;
    // sikangSeed(points, &ds, v0);

    decimal_t ratio = 1.0;
    //    std::cout << "sikang seeded" << std::endl;
    for (auto &dti : ds) {
      dti *= ratio;
      //      std::cout << dti << std::endl;
    }

    waypoints_.front().vel /= ratio;
    v0 /= ratio;
    {
      // ScopedTimer tm("Model creation");
      ltraj_ = boost::make_shared<LegendreTrajectory>(&model, ds, degree_,
                                                      polytype_, order_);
      ltraj_->setConParam(con_mode_, num_sample_);
    }
    {
      // ScopedTimer tm("Ax <= b");
      ltraj_->addAxbConstraints(A, b, ds);
    }
    {
      // ScopedTimer tm("Waypoints");

      ltraj_->addWayPointConstrains(waypoints_, use_vel_contr);
    }
    {
      // ScopedTimer tm("Set Cost");
      ltraj_->setCost(use_lp_);
    }
    //    addPathCost(A, b, epsilon);
    //    addLineCost(points, upsilon);

    {
      // ScopedTimer tm("Backend");
      model.optimize();
    }

    trajectory_solved_ = ltraj_->recoverVars();
    // exit early
    if (!trajectory_solved_) {
      std::cout << "Initial model is infeasible" << std::endl;
      model.computeIIS();
      model.write("initial_model_dump.ilp");
      return false;
    }
    /*
      for (int i = 0; i < time_its_; i++) {
      trajectory_solved_ = ltraj_->adjustTimes(time_eps_);
      if (!trajectory_solved_)
      break;
      model.optimize();
      }
    */

    // std::cout << "Cost is " << ltraj_->getCost() << std::endl;

    //    return trajectory_solved_;// for debug

    if (v_max_ != 0 || a_max_ != 0 || j_max_ != 0) {
      // ScopedTimer tm("Adjust time");
      ltraj_->adjustTimeToMaxs(v_max_ / ratio, a_max_ / ratio / ratio,
                               j_max_ / ratio / ratio / ratio, false, v0);
      ltraj_->resetConstr(false);
    } else {
      std::cout << "adding non fliping constraints" << std::endl;
      for (auto &seg : ltraj_->individual_sections) {
        for (double dt = 0.1; dt <= 1.0; dt += 0.1) {
          GRBLinExpr expr;
          seg->secs.at(2)->getContr(dt, 2, expr);
          model.addConstr(expr >= -9.9);
        }
      }
    }
    // std::cout << "Total time is " << ltraj_->getTotalTime() << std::endl;
    model.optimize();
    trajectory_solved_ = ltraj_->recoverVars(ratio);

    if (!trajectory_solved_) {
      std::cout << "Re optimization infeasible" << std::endl;
      return false;
    }

    cost_ = ltraj_->getCost();
    return true;
  } catch (GRBException e) {
    std::cout << e.getMessage().c_str() << std::endl;
    return false;
  }
}

bool GurobiSolver::checkMax(decimal_t ratio) {
  return ltraj_->check_max(v_max_ * ratio, a_max_ * ratio, j_max_ * ratio);
}

}  // namespace traj_opt

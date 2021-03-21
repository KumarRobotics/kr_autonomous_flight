/**
 * @file traj_solver.h
 * @brief Trajectory generator from given waypoints
 */
#ifndef MPL_TRAJ_SOLVER_H
#define MPL_TRAJ_SOLVER_H
#include <mpl_basis/trajectory.h>
#include <mpl_traj_solver/poly_solver.h>

/// Trajectory generator
template <int Dim>
class TrajSolver {
 public:
  /**
   * @brief Constructor
   * @param control define the control flag for start and end
   * @param yaw_control define the control flag for yaw start and end
   */
  TrajSolver(Control::Control control,
             Control::Control yaw_control = Control::VEL, bool debug = false)
      : control_(control), yaw_control_(yaw_control) {
    if (control == Control::VEL || control == Control::VELxYAW)
      poly_solver_.reset(new PolySolver<Dim>(0, 1, debug));
    else if (control == Control::ACC || control == Control::ACCxYAW)
      poly_solver_.reset(new PolySolver<Dim>(1, 2, debug));
    else if (control == Control::JRK || control == Control::JRKxYAW)
      poly_solver_.reset(new PolySolver<Dim>(2, 3, debug));
    // Due to dimension issue, only workd up to thrid order
    // if(control == Control::SNP || control == Control::SNPxYAW)
    // poly_solver_.reset(new PolySolver<Dim>(3, 4));
    if (yaw_control == Control::VEL)
      yaw_solver_.reset(new PolySolver<1>(0, 1));
    else if (yaw_control == Control::ACC)
      yaw_solver_.reset(new PolySolver<1>(1, 2));
    else if (yaw_control == Control::JRK)
      yaw_solver_.reset(new PolySolver<1>(2, 3));
  }

  /// Set Waypoint array directly, overwrite global vars
  void setWaypoints(const vec_E<Waypoint<Dim>>& ws) {
    path_.resize(ws.size());
    for (size_t i = 0; i < ws.size(); i++) path_[i] = ws[i].pos;
    waypoints_ = ws;
  }

  /// Set velocity used for internal time allocation
  void setV(decimal_t v) { v_ = v; }

  /// Set time allocation (optional), overwrite global dts, if not set, we will
  /// set a time allocation using L-inf
  void setDts(const std::vector<decimal_t>& dts) { dts_ = dts; }

  /// Set Waypoint array from path, overwrite global vars, in this mode, the
  /// intermediate Waypoint are with `Control::VEL`
  void setPath(const vec_Vecf<Dim>& path) {
    path_ = path;

    waypoints_.resize(path_.size());
    for (size_t i = 0; i < waypoints_.size(); i++) {
      waypoints_[i].pos = path[i];
      waypoints_[i].vel = Vecf<Dim>::Zero();
      waypoints_[i].acc = Vecf<Dim>::Zero();
      waypoints_[i].jrk = Vecf<Dim>::Zero();
      waypoints_[i].yaw = 0;
      waypoints_[i].control = Control::VEL;
    }

    waypoints_.front().control = control_;
    waypoints_.back().control = control_;
  }

  /// Solve for trajectory
  Trajectory<Dim> solve(bool verbose = false) {
    if (waypoints_.size() != dts_.size() + 1) dts_ = allocate_time(path_, v_);

    if (verbose) {
      for (const auto& it : dts_) std::cout << "dt: " << it << std::endl;
      for (const auto& it : waypoints_) it.print();
    }

    if (poly_solver_ && yaw_solver_) {
      // solve for pos
      poly_solver_->solve(waypoints_, dts_);
      auto traj =
          Trajectory<Dim>(poly_solver_->getTrajectory()->toPrimitives());
      // solve for yaw
      vec_E<Waypoint<1>> yaws;
      for (const auto& it : waypoints_) {
        Waypoint<1> yaw(Control::VEL);
        yaw.pos(0) = it.yaw;
        yaw.vel(0) = 0;
        yaw.acc(0) = 0;
        yaw.jrk(0) = 0;
        yaws.push_back(yaw);
      }
      yaws.front().control = yaw_control_;
      yaws.back().control = yaw_control_;
      yaw_solver_->solve(yaws, dts_);
      auto yaw_prs = yaw_solver_->getTrajectory()->toPrimitives();
      for (size_t i = 0; i < traj.segs.size(); i++)
        traj.segs[i].pr_yaw_ = yaw_prs[i].prs_[0];
      return traj;
    } else {
      if (verbose)
        printf(ANSI_COLOR_RED
               "TrajSolver is not initialized properlly!\n" ANSI_COLOR_RESET);
      return Trajectory<Dim>();
    }
  }

  /// Get the path used for time allocation
  vec_Vecf<Dim> getPath() const { return path_; }

  /// Get the Waypoint array used to solve trajectory
  vec_E<Waypoint<Dim>> getWaypoints() const { return waypoints_; }

  /// Get the time allocation
  std::vector<decimal_t> getDts() const { return dts_; }

 private:
  /// Internal time allocation from path and vel using L-inf
  std::vector<decimal_t> allocate_time(const vec_Vecf<Dim>& pts, decimal_t v) {
    if (pts.size() < 2 || v <= 0) return std::vector<decimal_t>();
    std::vector<decimal_t> dts(pts.size() - 1);
    for (unsigned int i = 1; i < pts.size(); i++) {
      decimal_t d = (pts[i] - pts[i - 1]).template lpNorm<Eigen::Infinity>();
      dts[i - 1] = d / v;
    }
    return dts;
  }

  /// Intermediate pos
  vec_Vecf<Dim> path_;

  /// Intermediate waypoints
  vec_E<Waypoint<Dim>> waypoints_;

  /// Time allocation
  std::vector<decimal_t> dts_;

  /// Velocity used for internal time allocation
  decimal_t v_{1};

  /// Control constraints for start and goal
  Control::Control control_;

  /// Control constraints for start and goal yaw
  Control::Control yaw_control_;

  /// Poly solver
  std::unique_ptr<PolySolver<Dim>> poly_solver_;

  /// Poly solver for yaw only
  std::unique_ptr<PolySolver<1>> yaw_solver_;
};

/// TrajSolver for 2D
typedef TrajSolver<2> TrajSolver2D;

/// TrajSolver for 3D
typedef TrajSolver<3> TrajSolver3D;
#endif

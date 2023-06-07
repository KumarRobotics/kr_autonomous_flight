
#include <action_planner/local_plan_server.h>

void LocalPlanServer::OptPlanner::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] Optimization planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();

  /* initialize main modules */
  planner_manager_.reset(new opt_planner::PlannerManager);
  planner_manager_->initPlanModules(local_plan_server_->traj_planner_nh_,
                                    mp_map_util_);

  // TODO(xu:) not differentiating between 2D and 3D, causing extra resource
  // usage for 2D case, this needed to be changed in both planner util as well
  // as map util, which requires the slicing map function
}

void LocalPlanServer::MPLPlanner::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] MPL planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  // set up visualization publisher for mpl planner
  expanded_cloud_pub =
      local_plan_server_->pnh_.advertise<sensor_msgs::PointCloud>(
          "expanded_cloud", 1, true);

  // set up mpl planner
  local_plan_server_->traj_planner_nh_.param("debug", debug_, false);
  local_plan_server_->traj_planner_nh_.param("verbose", verbose_, false);

  double dt;
  double W, v_fov;
  int ndt, max_num;
  local_plan_server_->traj_planner_nh_.param("tol_pos", tol_pos_, 0.5);
  local_plan_server_->traj_planner_nh_.param(
      "global_goal_tol_vel", goal_tol_vel_, 0.5);
  local_plan_server_->traj_planner_nh_.param(
      "global_goal_tol_acc", goal_tol_acc_, 0.5);
  /// Execution time for each primitive
  local_plan_server_->traj_planner_nh_.param("dt", dt, 1.0);
  local_plan_server_->traj_planner_nh_.param("ndt", ndt, -1);
  local_plan_server_->traj_planner_nh_.param("max_num", max_num, -1);
  local_plan_server_->traj_planner_nh_.param("heuristic_weight", W, 10.0);
  local_plan_server_->traj_planner_nh_.param("vertical_semi_fov", v_fov, 0.392);

  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();

  local_plan_server_->traj_planner_nh_.param(
      "use_3d_local", use_3d_local_, false);

  double vz_max, az_max, jz_max, uz_max;
  local_plan_server_->traj_planner_nh_.param("max_v_z", vz_max, 2.0);
  local_plan_server_->traj_planner_nh_.param("max_a_z", az_max, 1.0);
  local_plan_server_->traj_planner_nh_.param("max_j_z", jz_max, 1.0);
  local_plan_server_->traj_planner_nh_.param("max_u_z", uz_max, 1.0);

  double v_max, a_max, j_max, u_max;
  local_plan_server_->traj_planner_nh_.param("max_v", v_max, 2.0);
  local_plan_server_->traj_planner_nh_.param("max_a", a_max, 1.0);
  local_plan_server_->traj_planner_nh_.param("max_j", j_max, 1.0);
  local_plan_server_->traj_planner_nh_.param("max_u", u_max, 1.0);

  // Important: set motion primitive control inputs
  vec_E<VecDf> U;
  if (!use_3d_local_) {
    const decimal_t du = u_max;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        U.push_back(Vec3f(dx, dy, 0));
  } else {
    const decimal_t du = u_max;
    const decimal_t du_z = uz_max;
    for (decimal_t dx = -u_max; dx <= u_max; dx += du)
      for (decimal_t dy = -u_max; dy <= u_max; dy += du)
        for (decimal_t dz = -uz_max; dz <= uz_max; dz += du_z)
          U.push_back(Vec3f(dx, dy, dz));
  }

  mp_planner_util_.reset(new MPL::VoxelMapPlanner(verbose_));  // verbose
  mp_planner_util_->setMapUtil(
      mp_map_util_);                  // Set collision checking function
  mp_planner_util_->setEpsilon(1.0);  // Set greedy param (default equal to 1)

  mp_planner_util_->setVxy(v_max);  // Set max velocity along x and y
  mp_planner_util_->setVz(vz_max);  // Set max velocity along z

  mp_planner_util_->setAmax(a_max);  // Set max acceleration
  mp_planner_util_->setJmax(j_max);  // Set max jerk
  mp_planner_util_->setVfov(v_fov);  // Set vertical semi-fov
  // mp_planner_util_->setUmax(u_max); // Set max control input
  mp_planner_util_->setDt(dt);          // Set dt for each primitive
  mp_planner_util_->setTmax(ndt * dt);  // Set max time horizon of planning
  mp_planner_util_->setMaxNum(
      max_num);  // Set maximum allowed expansion, -1 means no limitation
  mp_planner_util_->setU(U);  // 2D discretization if false, 3D if true
  mp_planner_util_->setW(W);  // 2D discretization if false, 3D if true
  mp_planner_util_->setLPAstar(false);  // Use Astar
}

void LocalPlanServer::OptPlanner::plan(const MPL::Waypoint3D& start,
                                       const MPL::Waypoint3D& goal,
                                       const kr_planning_msgs::VoxelMap& map) {
  ROS_WARN("[LocalPlanServer] trigger opt_planner!!!!!");
  setMap(mp_map_util_, map);
  Eigen::MatrixXd startState(3, 3), endState(3, 3);
  startState << start.pos(0), start.vel(0), start.acc(0), start.pos(1),
      start.vel(1), start.acc(1), start.pos(2), start.vel(2), start.acc(2);
  endState << goal.pos(0), goal.vel(0), goal.acc(0), goal.pos(1), goal.vel(1),
      goal.acc(1), goal.pos(2), goal.vel(2), goal.acc(2);

  bool valid = planner_manager_->localPlanner(startState, endState);
  if (valid) {
    opt_traj_ = planner_manager_->local_data_.traj_;
    local_plan_server_->traj_total_time_ = opt_traj_.getTotalDuration();
    ROS_WARN("[LocalPlanServer] planning success ! !!!!!");
  }

  kr_planning_msgs::SplineTrajectory spline_msg;
  spline_msg.header.frame_id = local_plan_server_->frame_id_;
  spline_msg.dimensions = 3;

  int piece_num = opt_traj_.getPieceNum();
  Eigen::VectorXd durs = opt_traj_.getDurations();

  for (uint d = 0; d < 3; d++) {
    kr_planning_msgs::Spline spline;
    for (uint s = 0; s < piece_num; s++) {
      kr_planning_msgs::Polynomial poly;

      min_jerk::CoefficientMat coeff = opt_traj_[s].getCoeffMat(true);

      for (uint c = 0; c < 6; c++) {
        poly.coeffs.push_back(coeff(d, 5 - c));
      }
      poly.dt = durs[s];
      poly.degree = 5;
      spline.segs.push_back(poly);
    }
    spline.segments = piece_num;
    spline.t_total = local_plan_server_->traj_total_time_;
    spline_msg.data.push_back(spline);
  }
  local_plan_server_->process_result(spline_msg, valid);
}
void LocalPlanServer::MPLPlanner::plan(const MPL::Waypoint3D& start,
                                       const MPL::Waypoint3D& goal,
                                       const kr_planning_msgs::VoxelMap& map) {
  ROS_WARN("[LocalPlanServer] trigger mp_planner!!!!!");

  setMap(mp_map_util_, map);
  if (local_plan_server_->goal_->check_vel) {
    // Tolerance for goal region, -1 means no limitation
    mp_planner_util_->setTol(tol_pos_, goal_tol_vel_, goal_tol_acc_);
  } else {
    mp_planner_util_->setTol(tol_pos_, -1, -1);
  }

  mp_planner_util_->reset();
  // start and new_goal contain full informaiton about
  // position/velocity/acceleration
  bool valid = false;
  if (!use_3d_local_) {
    MPL::Waypoint3D new_goal = goal;
    new_goal.pos(2) =
        start.pos(2);  // or else it cannot reach the local goal position.
    valid = mp_planner_util_->plan(start, new_goal);
  } else {
    valid = mp_planner_util_->plan(start, goal);
  }

  if (valid) {
    mp_traj_ = mp_planner_util_->getTraj();
    local_plan_server_->traj_total_time_ = mp_traj_.getTotalTime();
    ROS_WARN("[LocalPlanServer] planning success ! !!!!!");
  }

  // for visualization: publish expanded nodes as a point cloud
  sensor_msgs::PointCloud expanded_ps =
      kr::vec_to_cloud(mp_planner_util_->getExpandedNodes());
  expanded_ps.header.frame_id = local_plan_server_->frame_id_;
  expanded_cloud_pub.publish(expanded_ps);
  kr_planning_msgs::Trajectory traj_msg = toTrajectoryROSMsg(mp_traj_);
  traj_msg.header.frame_id = local_plan_server_->frame_id_;
  auto spline_msg = traj_opt::SplineTrajectoryFromTrajectory(traj_msg);
  local_plan_server_->process_result(spline_msg, valid);
}

kr_planning_msgs::PlanTwoPointResult
LocalPlanServer::OptPlanner::process_result(double endt, int num_goals) {
  Eigen::Vector3d pos, vel, acc;
  kr_planning_msgs::PlanTwoPointResult result;

  pos.setZero(), vel.setZero(), acc.setZero();

  for (int i = 0; i < num_goals; i++) {
    geometry_msgs::Pose p_fin;
    geometry_msgs::Twist v_fin, a_fin, j_fin;

    double t_cur = endt * static_cast<double>(i + 1);

    pos = opt_traj_.getPos(t_cur);
    vel = opt_traj_.getVel(t_cur);
    acc = opt_traj_.getAcc(t_cur);

    // check if evaluation is successful, if not, set result.success to be
    // false! (if failure case, a null Waypoint is returned)
    if (pos == Eigen::Vector3d::Zero() && vel == Eigen::Vector3d::Zero()) {
      result.success = 0;
      ROS_WARN_STREAM(
          "waypoint evaluation failed, set result.success to be false");
      ROS_WARN_STREAM(
          "trajectory total time:" << local_plan_server_->traj_total_time_);
      ROS_WARN_STREAM("evaluating at:" << endt * static_cast<double>(i + 1));
    }

    p_fin.position.x = pos(0), p_fin.position.y = pos(1),
    p_fin.position.z = pos(2);
    p_fin.orientation.w = 1, p_fin.orientation.z = 0;
    v_fin.linear.x = vel(0), v_fin.linear.y = vel(1), v_fin.linear.z = vel(2);
    v_fin.angular.z = 0;
    a_fin.linear.x = acc(0), a_fin.linear.y = acc(1), a_fin.linear.z = acc(2);
    a_fin.angular.z = 0;
    result.p_stop.push_back(p_fin);
    result.v_stop.push_back(v_fin);
    result.a_stop.push_back(a_fin);
    result.j_stop.push_back(j_fin);
  }

  pos = opt_traj_.getPos(local_plan_server_->traj_total_time_);
  result.traj_end.position.x = pos(0);
  result.traj_end.position.y = pos(1);
  result.traj_end.position.z = pos(2);
  return result;
}

kr_planning_msgs::PlanTwoPointResult
LocalPlanServer::MPLPlanner::process_result(double endt, int num_goals) {
  kr_planning_msgs::PlanTwoPointResult result;

  for (int i = 0; i < num_goals; i++) {
    geometry_msgs::Pose p_fin;
    geometry_msgs::Twist v_fin, a_fin, j_fin;

    MPL::Waypoint3D pt_f = mp_traj_.evaluate(endt * static_cast<double>(i + 1));
    // check if evaluation is successful, if not, set result.success to be
    // false! (if failure case, a null Waypoint is returned)
    if ((pt_f.pos(0) == 0) && (pt_f.pos(1) == 0) && (pt_f.pos(2) == 0) &&
        (pt_f.vel(0) == 0) && (pt_f.vel(1) == 0) && (pt_f.vel(2) == 0)) {
      result.success = 0;
      ROS_WARN_STREAM(
          "waypoint evaluation failed, set result.success to be false");
      ROS_WARN_STREAM(
          "trajectory total time:" << local_plan_server_->traj_total_time_);
      ROS_WARN_STREAM("evaluating at:" << endt * static_cast<double>(i + 1));
    }

    p_fin.position.x = pt_f.pos(0), p_fin.position.y = pt_f.pos(1),
    p_fin.position.z = pt_f.pos(2);
    p_fin.orientation.w = 1, p_fin.orientation.z = 0;
    v_fin.linear.x = pt_f.vel(0), v_fin.linear.y = pt_f.vel(1),
    v_fin.linear.z = pt_f.vel(2);
    v_fin.angular.z = 0;
    a_fin.linear.x = pt_f.acc(0), a_fin.linear.y = pt_f.acc(1),
    a_fin.linear.z = pt_f.acc(2);
    a_fin.angular.z = 0;
    result.p_stop.push_back(p_fin);
    result.v_stop.push_back(v_fin);
    result.a_stop.push_back(a_fin);
    result.j_stop.push_back(j_fin);
  }

  MPL::Waypoint3D pt = mp_traj_.evaluate(local_plan_server_->traj_total_time_);
  result.traj_end.position.x = pt.pos(0);
  result.traj_end.position.y = pt.pos(1);
  result.traj_end.position.z = pt.pos(2);
  return result;
}
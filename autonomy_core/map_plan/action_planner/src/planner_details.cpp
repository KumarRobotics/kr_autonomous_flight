
#include <action_planner/planner_details.h>

//
// Opt Planner
//
void OptPlanner::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] Optimization planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();

  /* initialize main modules */
  planner_manager_.reset(new opt_planner::PlannerManager);
  planner_manager_->initPlanModules(nh_, mp_map_util_, frame_id_);

  // TODO(xu:) not differentiating between 2D and 3D, causing extra resource
  // usage for 2D case, this needed to be changed in both planner util as well
  // as map util, which requires the slicing map function
}

kr_planning_msgs::SplineTrajectory OptPlanner::plan(
    const MPL::Waypoint3D& start,
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
    traj_total_time_ = opt_traj_.getTotalDuration();
    ROS_WARN("[LocalPlanServer] planning success ! !!!!!");
  } else {
    return kr_planning_msgs::SplineTrajectory();
  }

  kr_planning_msgs::SplineTrajectory spline_msg;
  spline_msg.header.frame_id = frame_id_;
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
    spline.t_total = traj_total_time_;
    spline_msg.data.push_back(spline);
  }
  return spline_msg;
}

MPL::Waypoint3D OptPlanner::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  waypoint.pos = opt_traj_.getPos(t);
  waypoint.vel = opt_traj_.getVel(t);
  waypoint.acc = opt_traj_.getAcc(t);

  return waypoint;
}

//
// MPL Planner
//

void MPLPlanner::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] MPL planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  // set up visualization publisher for mpl planner
  expanded_cloud_pub =
      nh_.advertise<sensor_msgs::PointCloud>("expanded_cloud", 1, true);

  // set up mpl planner
  nh_.param("debug", debug_, false);
  nh_.param("verbose", verbose_, false);

  double dt;
  double W, v_fov;
  int ndt, max_num;
  nh_.param("tol_pos", tol_pos_, 0.5);
  nh_.param("global_goal_tol_vel", goal_tol_vel_, 0.5);
  nh_.param("global_goal_tol_acc", goal_tol_acc_, 0.5);
  /// Execution time for each primitive
  nh_.param("dt", dt, 1.0);
  nh_.param("ndt", ndt, -1);
  nh_.param("max_num", max_num, -1);
  nh_.param("heuristic_weight", W, 10.0);
  nh_.param("vertical_semi_fov", v_fov, 0.392);

  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();

  nh_.param("use_3d_local", use_3d_local_, false);

  double vz_max, az_max, jz_max, uz_max;
  nh_.param("max_v_z", vz_max, 2.0);
  nh_.param("max_a_z", az_max, 1.0);
  nh_.param("max_j_z", jz_max, 1.0);
  nh_.param("max_u_z", uz_max, 1.0);

  double v_max, a_max, j_max, u_max;
  nh_.param("max_v", v_max, 2.0);
  nh_.param("max_a", a_max, 1.0);
  nh_.param("max_j", j_max, 1.0);
  nh_.param("max_u", u_max, 1.0);

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

kr_planning_msgs::SplineTrajectory MPLPlanner::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  ROS_WARN("[LocalPlanServer] trigger mp_planner!!!!!");

  setMap(mp_map_util_, map);
  if (action_server_goal_.check_vel) {
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
    traj_total_time_ = mp_traj_.getTotalTime();
    ROS_WARN("[LocalPlanServer] planning success ! !!!!!");
  } else {
    return kr_planning_msgs::SplineTrajectory();
  }

  // for visualization: publish expanded nodes as a point cloud
  sensor_msgs::PointCloud expanded_ps =
      kr::vec_to_cloud(mp_planner_util_->getExpandedNodes());
  expanded_ps.header.frame_id = frame_id_;
  expanded_cloud_pub.publish(expanded_ps);
  kr_planning_msgs::Trajectory traj_msg = toTrajectoryROSMsg(mp_traj_);
  traj_msg.header.frame_id = frame_id_;
  auto spline_msg = traj_opt::SplineTrajectoryFromTrajectory(traj_msg);
  return spline_msg;
}

MPL::Waypoint3D MPLPlanner::evaluate(double t) { return mp_traj_.evaluate(t); }

//
// Min Dispersion Planner
//

void DispersionPlanner::setup() {
  nh_.param("trajectory_planner/tol_pos", tol_pos_, 0.5);
  ROS_INFO_STREAM("Position tolerance: " << tol_pos_);
  nh_.param("trajectory_planner/global_goal_tol_vel", tol_vel_, 1.5);
  nh_.param<std::string>("heuristic", heuristic_, "min_time");

  std::string graph_file;
  nh_.param("graph_file",
            graph_file,
            std::string("/home/laura/medium_faster20.json"));
  ROS_INFO("Reading graph file %s", graph_file.c_str());
  graph_ = std::make_shared<motion_primitives::MotionPrimitiveGraph>(
      motion_primitives::read_motion_primitive_graph(graph_file));
  visited_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("visited", 1, true);
}

kr_planning_msgs::SplineTrajectory DispersionPlanner::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  // TODO cleanup if(compute_first_mp) blocks

  // TODO harcoded for 2D
  Eigen::VectorXd start_state(graph_->state_dim());
  start_state << start.pos(0), start.pos(1), start.vel(0), start.vel(1),
      start.acc(0), start.acc(1);
  Eigen::VectorXd goal_state(graph_->state_dim());
  goal_state << goal.pos(0), goal.pos(1), goal.vel(0), goal.vel(1), goal.acc(0),
      goal.acc(1);
  const kr_planning_msgs::SplineTrajectory last_traj =
      action_server_goal_.last_traj;
  double eval_time = action_server_goal_.eval_time;
  int planner_start_index = -1;
  bool compute_first_mp = last_traj.data.size() > 0;
  if (!compute_first_mp)
    ROS_WARN("Missing last trajectory (stopping policy may have run)");
  int seg_num = -1;
  double finished_segs_time = 0;
  std::shared_ptr<motion_primitives::MotionPrimitive> mp;

  // If we received a last trajectory, we want to use it to create a new
  // trajectory that starts on with the segment of the old trajectory that
  // we are flying on, at the requested eval time (which is a little bit in
  // the future of where its currently flying). With a slight abuse of
  // notation, I will refer to this as the "current segment"
  if (compute_first_mp) {
    // Figure out which segment of the last trajectory is the current
    // segment
    for (int i = 0; i < last_traj.data[0].segments; i++) {
      auto seg = last_traj.data[0].segs[i];
      finished_segs_time += seg.dt;
      if (finished_segs_time > eval_time) {
        finished_segs_time -= seg.dt;
        seg_num = i;
        break;
      }
    }
    // todo(laura) what to do if eval time is past the end of the traj
    if (seg_num == -1) seg_num = last_traj.data[0].segments - 1;
    ROS_INFO_STREAM("Seg num " << seg_num);

    // The planner will start at the end of the current segment, so we get
    // its end index.
    planner_start_index = last_traj.data[0].segs[seg_num].end_index;

    mp = motion_primitives::recover_mp_from_SplineTrajectory(
        last_traj, graph_, seg_num);
    start_state = mp->end_state_;
    ROS_INFO_STREAM("Planner adjusted start: " << mp->end_state_.transpose());
  }

  motion_primitives::GraphSearch::Option options = {
      .start_state = start_state,
      .goal_state = goal_state,
      .distance_threshold = tol_pos_,
      .parallel_expand = true,
      .heuristic = heuristic_,
      .access_graph = false,
      .start_index = planner_start_index,
      .step_size = .2};
  if (graph_->spatial_dim() == 2) {
    options.fixed_z = start.pos(2);
  }
  //   if (msg->check_vel) options.velocity_threshold = tol_vel;
  if (planner_start_index == -1 ||
      planner_start_index >= graph_->num_tiled_states())
    options.access_graph = true;
  // TODO(laura) why should planner_start_index >= graph_->num_tiled_states()
  // ever happen except when switching graphs, but checking if
  // graph_index_!=last_graph.graph_index didn't work

  motion_primitives::GraphSearch gs(*graph_, map, options);
  const auto start_time = ros::Time::now();

  auto [path, nodes] = gs.Search();
  bool planner_start_too_close_to_goal =
      motion_primitives::StatePosWithin(options.start_state,
                                        options.goal_state,
                                        graph_->spatial_dim(),
                                        options.distance_threshold);
  if (path.empty() && !planner_start_too_close_to_goal) {
    ROS_ERROR("Graph search failed, aborting action server.");
    return kr_planning_msgs::SplineTrajectory();
  }
  if (compute_first_mp) {
    // To our planned trajectory, we add a cropped motion primitive that is
    // in the middle of the last_traj segment that we are evaluating at

    Eigen::VectorXd cropped_start(graph_->state_dim());
    Eigen::MatrixXd cropped_poly_coeffs;

    double shift_time = 0;
    // We need to shift the motion primitive to start at the eval_time. If
    // we are the first segment, it might have been itself a cropped motion
    // primitive, so the shift_time is a little more complicated.
    // finished_segs_time is the total time of all the segments before the
    // current segment.
    if (seg_num == 0)
      shift_time = eval_time;
    else
      shift_time = eval_time - finished_segs_time;

    cropped_poly_coeffs = motion_primitives::GraphSearch::shift_polynomial(
        mp->poly_coeffs_, shift_time);

    // Use the polynomial coefficients to get the new start, which is
    // hopefully the same as the planning query requested start.
    for (int i = 0; i < graph_->spatial_dim(); i++) {
      cropped_start(i) = cropped_poly_coeffs(i, cropped_poly_coeffs.cols() - 1);
      cropped_start(graph_->spatial_dim() + i) =
          cropped_poly_coeffs(i, cropped_poly_coeffs.cols() - 2);
      if (graph_->control_space_dim() > 2) {
        cropped_start(2 * graph_->spatial_dim() + i) =
            cropped_poly_coeffs(i, cropped_poly_coeffs.cols() - 3);
      }
    }
    ROS_INFO_STREAM("Cropped start " << cropped_start.transpose());

    auto first_mp =
        graph_->createMotionPrimitivePtrFromGraph(cropped_start, start.pos);
    // auto first_mp = graph_->createMotionPrimitivePtrFromGraph(
    //     cropped_start, path[0]->start_state_);

    double new_seg_time = mp->traj_time_ - shift_time;
    first_mp->populate(0,
                       new_seg_time,
                       cropped_poly_coeffs,
                       last_traj.data[0].segs[seg_num].start_index,
                       last_traj.data[0].segs[seg_num].end_index);
    // Add the cropped motion primitive to the beginning of the planned
    // trajectory
    path.insert(path.begin(), first_mp);
  }

  ROS_INFO("Graph search succeeded.");

  const auto total_time = (ros::Time::now() - start_time).toSec();
  ROS_INFO("Finished planning. Planning time %f s", total_time);

  auto spline_msg = motion_primitives::path_to_spline_traj_msg(
      path, map.header, options.fixed_z);
  traj_total_time_ = 0;
  for (auto spline : spline_msg.data) {
    traj_total_time_ += spline.t_total;
  }

  ROS_INFO_STREAM("path size: " << path.size());
  dispersion_traj_ = path;
  const auto visited_marray = motion_primitives::StatesToMarkerArray(
      gs.GetVisitedStates(), gs.spatial_dim(), map.header, 0.1, false, options.fixed_z);
  visited_pub_.publish(visited_marray);

  return spline_msg;
};

MPL::Waypoint3D DispersionPlanner::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  waypoint.pos = motion_primitives::getState(dispersion_traj_, t, 0);
  waypoint.vel = motion_primitives::getState(dispersion_traj_, t, 1);
  waypoint.acc = motion_primitives::getState(dispersion_traj_, t, 2);

  return waypoint;
}
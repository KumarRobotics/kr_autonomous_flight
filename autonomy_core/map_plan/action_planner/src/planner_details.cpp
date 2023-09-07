
#include <action_planner/planner_details.h>
#include <kr_planning_rviz_plugins/spline_trajectory_visual.h>

#include <iostream>
//
// Double Descrisption Planner 
//
void OptPlanner::iLQR_Planner::setup() {
  ROS_INFO("[iLQR]::SETTING UP iLQR PLANNER");
  bool subscribe_to_traj = false;
  bool publish_optimized_traj = false;
  bool publish_viz = true;  // N sample, time limit
  sampler_.reset(
      new SplineTrajSampler(subscribe_to_traj,
                            publish_optimized_traj,
                            publish_viz,
                            65));  // good if multiple of 5, then it will
                                   // automatically return +1 elements
                                   // this is the number of controls
}
kr_planning_msgs::TrajectoryDiscretized OptPlanner::iLQR_Planner::plan_discrete(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  ROS_WARN("[iLQR] Discrete Planning");
  Eigen::VectorXd start_state(7);
  start_state << start.pos(0), start.pos(1), start.pos(2), start.vel(0),
      start.vel(1), start.vel(2), start.yaw;
  kr_planning_msgs::TrajectoryDiscretized result_discrete =
      sampler_->sample_and_refine_trajectory(
          start_state, search_path_msg_, this->hPolys, this->allo_ts);
  if (result_discrete.pos.size() == 0) {
    traj_total_time_ = 0;
    opt_traj_.clear();
    path_sampling_dt_ = 0.0;
  } else {
    traj_total_time_ = result_discrete.N_ctrl * result_discrete.dt;
    opt_traj_ = sampler_->opt_traj_;
    path_sampling_dt_ = result_discrete.dt;
  }
  return result_discrete;
}

MPL::Waypoint3D OptPlanner::iLQR_Planner::evaluate(double t) {
  // this function is not currently used!!
  // TODO (Yifei): make it better usign non-linear cut!
  MPL::Waypoint3D waypt_return = MPL::Waypoint3D();
  Eigen::VectorXd x_return(9);
  if (t >= traj_total_time_) {
    x_return = opt_traj_.back();
  } else if (t < 0) {
    x_return = opt_traj_.front();
  } else {
    int index = std::floor(t / path_sampling_dt_);  // 0.95 / 0.1 = 9

    Eigen::VectorXd x1 = opt_traj_[index];
    Eigen::VectorXd x2 = opt_traj_[index + 1];

    double tau = t - index * path_sampling_dt_;

    std::cout << "  index  " << index << std::endl;

    // if (is_linear_cut) {
    x_return = x1 + tau / path_sampling_dt_ * (x2 - x1);
    // }

    // Eigen::MatrixXd psi = (Phi(dt - tau).transpose());
    // Eigen::MatrixXd lambda = Phi(tau) - psi * Phi(dt);

    // x_return = lambda * x1 + psi * x2;
  }
  waypt_return.pos = x_return.segment<3>(0);
  waypt_return.vel = x_return.segment<3>(3);
  waypt_return.acc = x_return.segment<3>(6);

  return waypt_return;
}

void OptPlanner::DoubleDescription::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] Double Description planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  mp_map_util_ = std::make_shared<MPL::VoxelMapUtil>();

  /* initialize main modules */
  planner_manager_.reset(new opt_planner::PlannerManager);
  planner_manager_->initPlanModules(nh_, mp_map_util_, frame_id_);

  // TODO(xu:) not differentiating between 2D and 3D, causing extra resource
  // usage for 2D case, this needed to be changed in both planner util as well
  // as map util, which requires the slicing map function
}

kr_planning_msgs::SplineTrajectory OptPlanner::DoubleDescription::plan(
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

  bool valid =
      planner_manager_->localPlanner(startState, endState, search_path_);
  if (valid) {
    opt_traj_ = planner_manager_->local_data_.traj_;
    traj_total_time_ = opt_traj_.getTotalDuration();
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

MPL::Waypoint3D OptPlanner::DoubleDescription::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  waypoint.pos = opt_traj_.getPos(t);
  waypoint.vel = opt_traj_.getVel(t);
  waypoint.acc = opt_traj_.getAcc(t);
  return waypoint;
}

void OptPlanner::GCOPTER::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] GCOPTER Optimization planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  /* initialize main modules */
  //ros::NodeHandle nh = ros::NodeHandle("~");
  planner_manager_.reset(new gcopter::GcopterPlanner(nh_, frame_id_));
  ROS_WARN("[LocalPlanServer:] GCOPTER setup complete");
}

kr_planning_msgs::SplineTrajectory OptPlanner::GCOPTER::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  ROS_WARN("[LocalPlanServer] trigger opt_planner!!!!!");

  ROS_WARN_STREAM("[GCOPTER]: Vel Norm:" << start.vel.norm());
  ROS_WARN_STREAM("[GCOPTER]: Acc Norm:" << start.acc.norm());
  Eigen::MatrixXd startState(3, 3), endState(3, 3);
  startState << start.pos(0), start.vel(0), start.acc(0), start.pos(1),
      start.vel(1), start.acc(1), start.pos(2), start.vel(2), start.acc(2);
  endState << goal.pos(0), goal.vel(0), goal.acc(0), goal.pos(1), goal.vel(1),
      goal.acc(1), goal.pos(2), goal.vel(2), goal.acc(2);

  planner_manager_->setMap(map);

  bool valid =
      planner_manager_->plan(startState, endState, search_path_, hPolys);
  if (valid) {
    opt_traj_ = planner_manager_->getTraj();
    traj_total_time_ = opt_traj_.getTotalDuration();
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

      auto coeff = opt_traj_[s].normalizePosCoeffMat();

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

MPL::Waypoint3D OptPlanner::GCOPTER::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  waypoint.pos = opt_traj_.getPos(t);
  waypoint.vel = opt_traj_.getVel(t);
  waypoint.acc = opt_traj_.getAcc(t);
  return waypoint;
}

//
// MPL Planner
//

void SearchPlanner::UniformInputSampling::setup() {
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
  mp_planner_util_->setU(U);
  mp_planner_util_->setW(W);
  mp_planner_util_->setLPAstar(false);  // Use Astar
}

kr_planning_msgs::SplineTrajectory SearchPlanner::UniformInputSampling::plan(
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
  // start and new_goal contain full information about
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

MPL::Waypoint3D SearchPlanner::UniformInputSampling::evaluate(double t) {
  return mp_traj_.evaluate(t);
}

//
// Min Dispersion Planner
//

void SearchPlanner::Dispersion::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] Dispersion planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  nh_.param("tol_pos", tol_pos_, 0.5);
  ROS_INFO_STREAM("Position tolerance: " << tol_pos_);
  nh_.param<std::string>("heuristic", heuristic_, "min_time");

  std::string graph_file;
  nh_.param<std::string>("dispersion/graph_file", graph_file, "");
  ROS_INFO("Reading graph file %s", graph_file.c_str());
  graph_ = std::make_shared<motion_primitives::MotionPrimitiveGraph>(
      motion_primitives::read_motion_primitive_graph(graph_file));
  visited_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("visited", 1, true);
}

kr_planning_msgs::SplineTrajectory SearchPlanner::Dispersion::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  // TODO(Laura) cleanup if(compute_first_mp) blocks

  // TODO(Laura) hardcoded for 2D
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

  options_ = {.start_state = start_state,
              .goal_state = goal_state,
              .distance_threshold = tol_pos_,
              .parallel_expand = true,
              .heuristic = heuristic_,
              .access_graph = false,
              .start_index = planner_start_index,
              .step_size = .2};
  if (graph_->spatial_dim() == 2) {
    options_.fixed_z = start.pos(2);
  }
  //   if (msg->check_vel) options_.velocity_threshold = tol_vel;
  if (planner_start_index == -1 ||
      planner_start_index >= graph_->num_tiled_states()) {
    options_.access_graph = true;
  }
  // TODO(laura) why should planner_start_index >= graph_->num_tiled_states()
  // ever happen except when switching graphs, but checking if
  // graph_index_!=last_graph.graph_index didn't work

  motion_primitives::GraphSearch gs(*graph_, map, options_);
  const auto start_time = ros::Time::now();

  auto [path, nodes] = gs.Search();
  const auto visited_marray =
      motion_primitives::StatesToMarkerArray(gs.GetVisitedStates(),
                                             gs.spatial_dim(),
                                             map.header,
                                             0.1,
                                             false,
                                             options_.fixed_z);
  visited_pub_.publish(visited_marray);
  bool planner_start_too_close_to_goal =
      motion_primitives::StatePosWithin(options_.start_state,
                                        options_.goal_state,
                                        graph_->spatial_dim(),
                                        options_.distance_threshold);
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
      path, map.header, options_.fixed_z);
  traj_total_time_ = spline_msg.data.back().t_total;
  dispersion_traj_ = path;

  return spline_msg;
}

MPL::Waypoint3D SearchPlanner::Dispersion::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  waypoint.pos =
      motion_primitives::getState(dispersion_traj_, t, 0, options_.fixed_z);
  waypoint.vel = motion_primitives::getState(dispersion_traj_, t, 1, 0);
  waypoint.acc = motion_primitives::getState(dispersion_traj_, t, 2, 0);

  return waypoint;
}

kr_planning_msgs::SplineTrajectory
path_to_spline_traj(  // this function needs float
    kr_planning_msgs::Path path,
    float velocity) {
  float total_dist = 0.0;
  for (int seg_num = 0; seg_num < path.waypoints.size() - 1; seg_num++) {
    Eigen::Vector3f pt1(path.waypoints.at(seg_num + 1).x,
                        path.waypoints.at(seg_num + 1).y,
                        path.waypoints.at(seg_num + 1).z);
    Eigen::Vector3f pt0(path.waypoints.at(seg_num).x,
                        path.waypoints.at(seg_num).y,
                        path.waypoints.at(seg_num).z);

    Eigen::Vector3f vec_between = pt1 - pt0;
    float distance = vec_between.norm();
    total_dist += distance;
  }
  kr_planning_msgs::SplineTrajectory spline_traj;
  spline_traj.dimensions = 3;
  for (int i = 0; i < 3; i++) {
    kr_planning_msgs::Spline spline;
    spline_traj.data.push_back(spline);
  }
  float accumulative_dist = 0.0;
  float v0_norm = 0.0;
  for (int seg_num = 0; seg_num < path.waypoints.size() - 1; seg_num++) {
    Eigen::Vector3f pt1(path.waypoints.at(seg_num + 1).x,
                        path.waypoints.at(seg_num + 1).y,
                        path.waypoints.at(seg_num + 1).z);
    Eigen::Vector3f pt0(path.waypoints.at(seg_num).x,
                        path.waypoints.at(seg_num).y,
                        path.waypoints.at(seg_num).z);

    Eigen::Vector3f vec_between = pt1 - pt0;
    float distance = vec_between.norm();

    // we make distance porpotional to velocity, this will make speed not
    // linearly but still smooth
    accumulative_dist += distance;

    float vf = std::min(std::min(accumulative_dist, velocity),
                        total_dist - accumulative_dist);  // total velocity
    float t_seg = distance * 2 / (vf + v0_norm);          // triangle integrate

    // std::cout << "total dist :" << accumulative_dist << " vf: " << vf
    //           << " v0: " << v0_norm << " t_seg: " << t_seg << std::endl;

    for (int dim = 0; dim < 3; dim++) {
      float v0_dim = vec_between(dim) / distance * v0_norm;
      float vf_dim = vec_between(dim) / distance * vf;
      kr_planning_msgs::Polynomial seg;
      seg.degree = 5;  // Chosen a little arbitrarily
      seg.dt = t_seg;

      seg.coeffs = {
          pt0(dim), v0_dim * t_seg, 0.5 * (vf_dim - v0_dim) * t_seg, 0, 0, 0};

      spline_traj.data.at(dim).segments++;
      spline_traj.data.at(dim).segs.push_back(seg);
      spline_traj.data.at(dim).t_total += seg.dt;
    }
    v0_norm = vf;
  }
  return spline_traj;
}

void SearchPlanner::Geometric::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] JPS planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  // Hardcoded 3d, will make performance worse in 2d
  jps_3d_map_util_ = std::make_shared<JPS::VoxelMapUtil>();
  jps_3d_util_ = std::make_shared<JPS::JPSPlanner3D>(verbose_);
  jps_3d_util_->setMapUtil(jps_3d_map_util_);
  path_pub_ = nh_.advertise<kr_planning_msgs::Path>("path", 1, true);
}

kr_planning_msgs::SplineTrajectory SearchPlanner::Geometric::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  spline_traj_ = kr_planning_msgs::SplineTrajectory();
  setMap(jps_3d_map_util_, map);
  jps_3d_util_->updateMap();
  if (!jps_3d_util_->plan(start.pos, goal.pos, 1.0, true)) {
    ROS_WARN("Failed to plan a JPS path!");
  } else {
    kr_planning_msgs::Path path = kr::path_to_ros(jps_3d_util_->getPath());
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    path_pub_.publish(path);

    double velocity;
    nh_.param("max_v", velocity, 2.0);
    spline_traj_ = path_to_spline_traj(path, static_cast<float>(velocity));
    spline_traj_.header.frame_id = frame_id_;
    spline_traj_.header.stamp = ros::Time::now();
    traj_total_time_ = spline_traj_.data[0].t_total;
  }
  return spline_traj_;
}
MPL::Waypoint3D SearchPlanner::Geometric::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  // A little ugly to use this method for evaluation
  waypoint.pos = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 0);
  waypoint.vel = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 1);
  waypoint.acc = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 2);
  return waypoint;
}

void SearchPlanner::PathThrough::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] Path through mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  path_pub_ = nh_.advertise<kr_planning_msgs::Path>("path", 1, true);
  high_level_planner_sub_ =
      nh_.subscribe("/quadrotor/global_plan_server/path",
                    1,
                    &SearchPlanner::PathThrough::highLevelPlannerCB,
                    this);
}

void SearchPlanner::PathThrough::highLevelPlannerCB(
    const kr_planning_msgs::Path& path) {
  path_ = path;
  ROS_INFO_STREAM("[Local Plan Search]: Passing through global plan!");
}

kr_planning_msgs::SplineTrajectory SearchPlanner::PathThrough::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  (void)map;
  if (path_.waypoints.empty()) {
    ROS_WARN("No path received yet!");
    return kr_planning_msgs::SplineTrajectory();
  }
  kr_planning_msgs::Path path = path_;
  path.header.frame_id = frame_id_;
  path.header.stamp = ros::Time::now();
  path_pub_.publish(path);

  double velocity;
  nh_.param("max_v", velocity, 2.0);
  spline_traj_ = path_to_spline_traj(path, velocity);
  spline_traj_.header.frame_id = frame_id_;
  spline_traj_.header.stamp = ros::Time::now();
  traj_total_time_ = spline_traj_.data[0].t_total;

  return spline_traj_;
}

MPL::Waypoint3D SearchPlanner::PathThrough::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  // A little ugly to use this method for evaluation
  waypoint.pos = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 0);
  waypoint.vel = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 1);
  waypoint.acc = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 2);
  return waypoint;
}

// rrt
void SearchPlanner::Sampling::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] RRT planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  //ros::NodeHandle nh = ros::NodeHandle("~");
  rrtplanner_.reset(new gcopter::GcopterPlanner(nh_, frame_id_));
  path_pub_ = nh_.advertise<kr_planning_msgs::Path>("path", 1, true);
}

kr_planning_msgs::SplineTrajectory SearchPlanner::Sampling::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  spline_traj_ = kr_planning_msgs::SplineTrajectory();

  rrtplanner_->setMap(map);
  std::vector<Eigen::Vector3d> route;

  if (!rrtplanner_->planRRT(start.pos, goal.pos, route)) {
    ROS_WARN("Failed to plan a RRT path!");
  } else {
    path_.clear();
    for (auto& wp : route) {
      path_.push_back(wp);
    }

    kr_planning_msgs::Path path = kr::path_to_ros(path_);
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    path_pub_.publish(path);

    //@yuwei: constant velocity
    double velocity;
    nh_.param("max_v", velocity, 2.0);
    spline_traj_ = path_to_spline_traj(path, velocity);
    spline_traj_.header.frame_id = frame_id_;
    spline_traj_.header.stamp = ros::Time::now();
    traj_total_time_ = spline_traj_.data[0].t_total;
  }
  return spline_traj_;
}

MPL::Waypoint3D SearchPlanner::Sampling::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  // A little ugly to use this method for evaluation
  waypoint.pos = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 0);
  waypoint.vel = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 1);
  waypoint.acc = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 2);
  return waypoint;
}

// sst
void SearchPlanner::DynSampling::setup() {
  ROS_WARN("+++++++++++++++++++++++++++++++++++");
  ROS_WARN("[LocalPlanServer:] SST planner mode!!!!!");
  ROS_WARN("+++++++++++++++++++++++++++++++++++");

  //ros::NodeHandle nh = ros::NodeHandle("~");
  sstplanner_.reset(new gcopter::GcopterPlanner(nh_, frame_id_));
  path_pub_ = nh_.advertise<kr_planning_msgs::Path>("path", 1, true);
}

kr_planning_msgs::SplineTrajectory SearchPlanner::DynSampling::plan(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map) {
  spline_traj_ = kr_planning_msgs::SplineTrajectory();

  Eigen::MatrixXd startState(3, 3), endState(3, 3);
  startState << start.pos(0), start.vel(0), start.acc(0), start.pos(1),
      start.vel(1), start.acc(1), start.pos(2), start.vel(2), start.acc(2);
  endState << goal.pos(0), goal.vel(0), goal.acc(0), goal.pos(1), goal.vel(1),
      goal.acc(1), goal.pos(2), goal.vel(2), goal.acc(2);

  sstplanner_->setMap(map);
  std::vector<Eigen::VectorXd> route;

  if (!sstplanner_->planSST(startState, endState, route)) {
    ROS_WARN("Failed to plan a SST path!");
  } else {
    path_.clear();
    for (auto& wp : route) {
      path_.push_back(wp.head(3));
    }
    path_.push_back(goal.pos);

    kr_planning_msgs::Path path = kr::path_to_ros(path_);
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    path_pub_.publish(path);

    //@yuwei: constant velocity
    // double velocity;
    // nh_.param("max_v", velocity, 2.0);
    // spline_traj_ = path_to_spline_traj(path, velocity);
    spline_traj_.dimensions = 3;
    int piece_num = route.size();
    for (uint d = 0; d < 3; d++) {
      kr_planning_msgs::Spline spline;
      double total_time = 0.0;
      for (uint s = 0; s < piece_num; s++) {
        kr_planning_msgs::Polynomial poly;
        double dt = route.at(s)(9);
        Eigen::Matrix<double, 3, 3> coeff;
        coeff.col(0) = route.at(s).head(3);                        // position
        coeff.col(1) = route.at(s).segment(3, 3) * dt;             // vel
        coeff.col(2) = 0.5 * route.at(s).segment(6, 3) * dt * dt;  // vel

        for (uint c = 0; c < 3; c++) {
          poly.coeffs.push_back(coeff(d, c));
        }
        poly.dt = dt;
        poly.degree = 2;
        spline.segs.push_back(poly);
        total_time += poly.dt;
      }
      spline.segments = piece_num;
      spline.t_total = total_time;
      spline_traj_.data.push_back(spline);
    }

    spline_traj_.header.frame_id = frame_id_;
    spline_traj_.header.stamp = ros::Time::now();
    traj_total_time_ = spline_traj_.data[0].t_total;
  }
  return spline_traj_;
}

MPL::Waypoint3D SearchPlanner::DynSampling::evaluate(double t) {
  MPL::Waypoint3D waypoint;

  // A little ugly to use this method for evaluation
  waypoint.pos = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 0);
  waypoint.vel = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 1);
  waypoint.acc = kr::SplineTrajectoryVisual::evaluate(spline_traj_, t, 2);
  return waypoint;
}

void CompositePlanner::setup() {
  int search_planner_type_id, opt_planner_type_id;
  nh_.param("search_planner_type", search_planner_type_id, -1);
  nh_.param("opt_planner_type", opt_planner_type_id, -1);
  search_traj_pub_ = nh_.advertise<kr_planning_msgs::SplineTrajectory>(
      "search_trajectory", 1, true);

  poly_gen_map_util_ = std::make_shared<MPL::VoxelMapUtil>();
  poly_generator_.reset(new opt_planner::PlannerManager);
  poly_generator_->initPlanModules(nh_, poly_gen_map_util_, frame_id_);

  switch (search_planner_type_id) {
    case 0:
      search_planner_type_ =
          new SearchPlanner::UniformInputSampling(nh_, frame_id_);
      break;
    case 1:
      search_planner_type_ = new SearchPlanner::Dispersion(nh_, frame_id_);
      break;
    case 2:
      search_planner_type_ = new SearchPlanner::Geometric(nh_, frame_id_);
      break;
    case 3:
      search_planner_type_ = new SearchPlanner::PathThrough(nh_, frame_id_);
      break;
    case 4:
      search_planner_type_ =
          new SearchPlanner::Sampling(nh_, frame_id_);  // RRT
      break;
    case 5:
      search_planner_type_ =
          new SearchPlanner::DynSampling(nh_, frame_id_);  // SST
      break;
    // case 4:
    //   search_planner_type_ = new
    //   SearchPlanner::SingleBVP(nh_, frame_id_); break;
    default:
      ROS_ERROR("No search planner selected; cannot continue");
      return;
  }

  switch (opt_planner_type_id) {
    case 0:
      opt_planner_type_ = new OptPlanner::DoubleDescription(nh_, frame_id_);
      break;
    case 1:
      opt_planner_type_ = new OptPlanner::GCOPTER(nh_, frame_id_);
      break;
    case 2:
      opt_planner_type_ = new OptPlanner::iLQR_Planner(nh_, frame_id_);
      break;
    default:
      ROS_ERROR("No opt planner selected; will use search planner only.");
      break;
  }

  if (search_planner_type_ != nullptr) {
    search_planner_type_->setup();
  }
  if (opt_planner_type_ != nullptr) {
    opt_planner_type_->setup();
  }
}

std::pair<kr_planning_msgs::SplineTrajectory,
          kr_planning_msgs::TrajectoryDiscretized>
CompositePlanner::plan_composite(
    const MPL::Waypoint3D& start,
    const MPL::Waypoint3D& goal,
    const kr_planning_msgs::VoxelMap& map,
    const kr_planning_msgs::VoxelMap& map_no_inflation,
    float* compute_time_front_end,
    float* compute_time_back_end,
    int& success_status) {
  // success status = 0: no success 1: front success 2: poly_gen_success 3: back
  // success
  auto start_timer = std::chrono::high_resolution_clock::now();
  kr_planning_msgs::SplineTrajectory result =
      search_planner_type_->plan(start, goal, map);
  kr_planning_msgs::TrajectoryDiscretized result_discretized;
  auto end_timer = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_timer - start_timer);
  auto empty_result =
      std::make_pair(kr_planning_msgs::SplineTrajectory(), result_discretized);
  *compute_time_front_end = duration.count() / 1000.0;

  // if result is empty, then just return an empty SplineTrajectory
  if (result.data.size() == 0) {
    success_status = 0;
    return empty_result;
  }
  success_status = 1;
  search_traj_pub_.publish(result);

  start_timer = std::chrono::high_resolution_clock::now();

  if (opt_planner_type_ != nullptr) {
    opt_planner_type_->search_path_msg_ = result;
    auto path = search_planner_type_->SamplePath();
    // Double description initialization traj must fully reach the end or it
    // will fail.
    // TODO(Laura) consider whether should actually calculate the BVP instead
    path.push_back(goal.pos);
    opt_planner_type_->setSearchPath(path);

    // before planning, generate polytopes
    std::vector<Eigen::MatrixXd> hPolys;
    Eigen::MatrixXd inner_pts;  // (4, N -1)
    Eigen::VectorXd allo_ts;
    setMap(poly_gen_map_util_, map);
    if (!poly_generator_->getSikangConst(
            opt_planner_type_->search_path_, inner_pts, allo_ts, hPolys)) {
      ROS_ERROR("[Local Planner]:Corridor generation fails!\n");
      return empty_result;
    }
    success_status = 2;
    opt_planner_type_->hPolys = hPolys;
    opt_planner_type_->allo_ts = allo_ts;
    // now do optimization
    result = opt_planner_type_->plan(start, goal, map);
    result_discretized = opt_planner_type_->plan_discrete(start, goal, map);
    if (result.data.size() != 0 || result_discretized.pos.size() != 0)
      success_status = 3;
    // TODO:(Yifei) only use no infla for gcopter planner, not dd planner
  }
  end_timer = std::chrono::high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>(end_timer -
                                                                   start_timer);
  *compute_time_back_end = duration.count() / 1000.0;

  return std::make_pair(result, result_discretized);
}

MPL::Waypoint3D CompositePlanner::evaluate(double t) {
  if (opt_planner_type_ != nullptr) {
    return opt_planner_type_->evaluate(t);
  }
  return search_planner_type_->evaluate(t);
}

std::vector<Eigen::Vector3d> PlannerType::SamplePath() {
  std::vector<Eigen::Vector3d> result(traj_total_time_ / path_sampling_dt_ + 1);
  for (int i = 0; i < result.size(); i++) {
    result[i] = evaluate(path_sampling_dt_ * i).pos;
  }
  return result;
}

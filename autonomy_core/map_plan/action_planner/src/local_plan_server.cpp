#include <action_planner/local_plan_server.h>

LocalPlanServer::LocalPlanServer(const ros::NodeHandle& nh) : pnh_(nh) {
  local_map_sub_ =
      pnh_.subscribe("local_voxel_map", 2, &LocalPlanServer::localMapCB, this);
  local_no_infla_map_sub_ = pnh_.subscribe(
      "local_noinfla_voxel_map", 2, &LocalPlanServer::localMapCBNoInfla, this);

  local_as_ = std::make_unique<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>(
      pnh_, "plan_local_trajectory", false);
  traj_planner_nh_ = ros::NodeHandle(pnh_, "trajectory_planner");
  sg_pub = pnh_.advertise<kr_planning_msgs::Path>("start_goal", 1, true);

  local_map_cleared_pub_ = pnh_.advertise<kr_planning_msgs::VoxelMap>(
      "local_voxel_map_cleared", 1, true);
  traj_pub_ =
      pnh_.advertise<kr_planning_msgs::SplineTrajectory>("trajectory", 1, true);
  local_as_->registerGoalCallback(boost::bind(&LocalPlanServer::goalCB, this));
  while (local_map_ptr_ == nullptr || local_nofla_map_ptr_ == nullptr) {
    ROS_WARN_THROTTLE(
        4,
        "[Local plan server]: Waiting for local map either inflated or not...");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  /**@yuwei : for falcon 250 interface**/
  pnh_.param("poly_srv_name", poly_srv_name_, std::string(" "));
  // trajectory boardcast
  traj_goal_pub_ = pnh_.advertise<kr_tracker_msgs::PolyTrackerActionGoal>(
      "tracker_cmd", 10, true);

  planner_ = new CompositePlanner(traj_planner_nh_, frame_id_);
  ROS_WARN("[Local planner:] planner instance created!");

  planner_->setup();
  ROS_WARN("[Local planner:] setup-ed!");

  local_as_->start();
  ROS_WARN("[Local planner:] Initialized!");
}

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCB(
    const kr_planning_msgs::VoxelMap::ConstPtr& msg) {
  ROS_WARN_ONCE("[Local planner:] Got the local voxel map!");
  local_map_ptr_ = msg;
  frame_id_ = local_map_ptr_->header.frame_id;
}

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCBNoInfla(
    const kr_planning_msgs::VoxelMap::ConstPtr& msg) {
  ROS_WARN_ONCE(
      "[Local planner:] Got the local voxel map without inflation!!!");
  local_nofla_map_ptr_ = msg;
  frame_id_ = local_nofla_map_ptr_->header.frame_id;
}

void LocalPlanServer::goalCB() {
  auto start_timer = std::chrono::high_resolution_clock::now();
  // check_vel is true if local planner reaches global goal

  aborted_ = false;
  auto goal_ptr = local_as_->acceptNewGoal();
  if (goal_ptr == NULL) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] Goal is null!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else if (local_map_ptr_ == nullptr) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] local map is not received!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else if (local_nofla_map_ptr_ == nullptr) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("[LocalPlanServer:] local no inflation map is not received!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  } else {
    process_goal(*goal_ptr);
  }
  auto end_timer = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_timer - start_timer);
  ROS_INFO("Local goalCB took %f ms", duration.count() / 1000.0);
  computation_time_ = duration.count() / 1000.0;
  planner_->setGoal(*goal_ptr);
}

void LocalPlanServer::process_goal(
    const kr_planning_msgs::PlanTwoPointGoal& as_goal) {
  boost::mutex::scoped_lock lockt(traj_mtx);
  if (aborted_) {
    if (local_as_->isActive()) {
      ROS_WARN("process_goal: Abort!");
      local_as_->setAborted();
    }
    return;
  }

  // set start and goal, assume goal is not null
  MPL::Waypoint3D start, goal;
  // instead of using current odometry as start, we use the given start position
  // for consistency between old and new trajectories in replan process
  start.pos = kr::pose_to_eigen(as_goal.p_init);
  start.vel = kr::twist_to_eigen(as_goal.v_init);
  start.acc = kr::twist_to_eigen(as_goal.a_init);
  start.jrk = kr::twist_to_eigen(as_goal.j_init);

  // Important: define use position, velocity, acceleration or jerk as control
  // inputs, note that the lowest order "false" term will be used as control
  // input, see instructions here:
  // https://github.com/KumarRobotics/kr_autonomous_flight/tree/master/autonomy_core/map_plan/mpl#example-usage
  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;
  start.use_jrk = false;
  // yaw planning or not (note that yaw alignment with velocity can be turned on
  // in trajectory_tracker)
  start.use_yaw = false;

  goal.pos = kr::pose_to_eigen(as_goal.p_final);
  goal.vel = kr::twist_to_eigen(as_goal.v_final);
  goal.acc = kr::twist_to_eigen(as_goal.a_final);
  goal.jrk = kr::twist_to_eigen(as_goal.j_final);
  goal.use_yaw = start.use_yaw;
  goal.use_pos = start.use_pos;
  goal.use_vel = start.use_vel;
  goal.use_acc = start.use_acc;
  goal.use_jrk = start.use_jrk;

  kr_planning_msgs::VoxelMap local_map_cleared, local_no_infla_map_cleared;
  local_map_cleared = clear_map_position(*local_map_ptr_, start.pos);
  local_no_infla_map_cleared =
      clear_map_position(*local_nofla_map_ptr_, start.pos);

  if (pub_cleared_map_) {
    local_map_cleared_pub_.publish(local_map_cleared);
    ROS_ERROR("Local map cleared published");
  }

  // for visualization: publish a path connecting local start and local goal
  vec_Vec3f sg;
  sg.push_back(start.pos);
  sg.push_back(goal.pos);
  kr_planning_msgs::Path sg_msg = kr::path_to_ros(sg);
  sg_msg.header.frame_id = frame_id_;
  sg_pub.publish(sg_msg);

  process_result(planner_->plan_composite(start,
                                          goal,
                                          local_map_cleared,
                                          local_no_infla_map_cleared,
                                          &compute_time_front_end_,
                                          &compute_time_back_end_),
                 as_goal.execution_time,
                 as_goal.epoch);
}

kr_planning_msgs::VoxelMap LocalPlanServer::clear_map_position(
    const kr_planning_msgs::VoxelMap& local_map_original, const Vec3f& start) {
  // make a copy, not the most efficient way, but necessary because we need to
  // maintain both original and cleared maps
  kr_planning_msgs::VoxelMap local_map_cleared;
  local_map_cleared = local_map_original;

  kr_planning_msgs::VoxelMap voxel_map;

  // Replaced with corresponding parameter value from VoxelMsg.msg
  int8_t val_free = voxel_map.val_free;
  ROS_WARN_ONCE("Value free is set as %d", val_free);
  double robot_r = 0.5;
  int robot_r_n = std::ceil(robot_r / local_map_cleared.resolution);

  vec_Vec3i clear_ns;
  for (int nx = -robot_r_n; nx <= robot_r_n; nx++) {
    for (int ny = -robot_r_n; ny <= robot_r_n; ny++) {
      for (int nz = -robot_r_n; nz <= robot_r_n; nz++) {
        clear_ns.push_back(Eigen::Vector3i(nx, ny, nz));
      }
    }
  }

  auto origin_x = local_map_cleared.origin.x;
  auto origin_y = local_map_cleared.origin.y;
  auto origin_z = local_map_cleared.origin.z;
  Eigen::Vector3i dim = Eigen::Vector3i::Zero();
  dim(0) = local_map_cleared.dim.x;
  dim(1) = local_map_cleared.dim.y;
  dim(2) = local_map_cleared.dim.z;
  auto res = local_map_cleared.resolution;
  const Eigen::Vector3i pn =
      Eigen::Vector3i(std::round((start(0) - origin_x) / res),
                      std::round((start(1) - origin_y) / res),
                      std::round((start(2) - origin_z) / res));

  for (const auto& n : clear_ns) {
    Eigen::Vector3i pnn = pn + n;
    int idx_tmp = pnn(0) + pnn(1) * dim(0) + pnn(2) * dim(0) * dim(1);
    if (!is_outside_map(pnn, dim) &&
        local_map_cleared.data[idx_tmp] != val_free) {
      local_map_cleared.data[idx_tmp] = val_free;
      // ROS_ERROR("clearing!!! idx %d", idx_tmp);
    }
  }
  return local_map_cleared;
}

bool LocalPlanServer::is_outside_map(const Eigen::Vector3i& pn,
                                     const Eigen::Vector3i& dim) {
  return pn(0) < 0 || pn(0) >= dim(0) || pn(1) < 0 || pn(1) >= dim(1) ||
         pn(2) < 0 || pn(2) >= dim(2);
}
Eigen::MatrixXd compute_bvp_pva(const Eigen::VectorXd& state1,
                                const Eigen::VectorXd& state2,
                                double T) {
  Eigen::Vector3d p0 = state1.head<3>();
  Eigen::Vector3d v0 = state1.segment<3>(3);
  Eigen::Vector3d a0 = state1.tail<3>();
  Eigen::Vector3d pf = state2.head<3>();
  Eigen::Vector3d vf = state2.segment<3>(3);
  Eigen::Vector3d af = state2.tail<3>();

  Eigen::MatrixXd time_mat(6, 6);
  time_mat << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 1, T,
      std::pow(T, 2), std::pow(T, 3), std::pow(T, 4), std::pow(T, 5), 0, 1,
      2 * T, 3 * std::pow(T, 2), 4 * std::pow(T, 3), 5 * std::pow(T, 4), 0, 0,
      2, 6 * T, 12 * std::pow(T, 2), 20 * std::pow(T, 3);

  Eigen::MatrixXd pva_mat(6, 3);
  pva_mat << p0, v0, a0, pf, vf, af;

  Eigen::MatrixXd coef = time_mat.inverse() * pva_mat;

  return coef;
}

Eigen::MatrixXd computeShotTraj(Eigen::VectorXd state1,  // p0 v0 a0, size 9
                                Eigen::VectorXd state2,
                                double time_to_goal) {
  /* ---------- get coefficient ---------- */
  using namespace Eigen;
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d dv = state2.segment(3, 3) - v0;
  const Vector3d a0 = state1.tail(3);
  // order = 5;
  double t_d = time_to_goal;
  MatrixXd coef(3, 6);
  double t_d_2 = t_d * t_d;
  double t_d_3 = t_d_2 * t_d;
  double t_d_4 = t_d_3 * t_d;
  const Vector3d delta_p = dp - v0 * t_d - 0.5 * a0 * t_d_2;
  const Vector3d delta_v = dv - a0 * t_d;
  const Vector3d delta_a = state2.tail(3) - a0;
  Vector3d a = 1.0 / 120.0 *
               (720.0 / (t_d_4 * t_d) * delta_p - 360.0 / t_d_4 * delta_v +
                60.0 / t_d_3 * delta_a);
  Vector3d b = 1.0 / 24.0 *
               (-360.0 / t_d_4 * delta_p + 168.0 / t_d_3 * delta_v -
                24.0 / t_d_2 * delta_a);
  Vector3d c =
      1.0 / 6.0 *
      (60.0 / t_d_3 * delta_p - 24.0 / t_d_2 * delta_v + 3.0 / t_d * delta_a);
  Vector3d d = 0.5 * a0;
  Vector3d e = v0;
  Vector3d f = p0;
  // 1/24 *  * t^4  +  1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^4 + b*t^3 + c*t^2 + v0*t + p0
  coef.col(5) = a;
  coef.col(4) = b;
  coef.col(3) = c;
  coef.col(2) = d;
  coef.col(1) = e;
  coef.col(0) = f;
  return coef.transpose();
}
void normalizePosCoeffMat(double duration, Eigen::MatrixXd& coeffMat) {
  double multiplier = 1.0;
  for (int i = 0; i < coeffMat.rows(); i++) {
    coeffMat.row(i) = coeffMat.row(i) * multiplier;
    multiplier *= duration;
  }
}

kr_planning_msgs::SplineTrajectory SplineTrajfromDiscreteTwoPoints(
    const kr_planning_msgs::TrajectoryDiscretized& traj_dis_msg) {
  int degree_plus1 = 6;
  double dt = traj_dis_msg.t[1] - traj_dis_msg.t[0];
  kr_planning_msgs::SplineTrajectory traj_msg;
  traj_msg.header = traj_dis_msg.header;
  traj_msg.dimensions = 3;
  // prepare polynomial fit time vector
  kr_planning_msgs::Spline spline;
  for (int dim = 0; dim < 3; dim++) {
    traj_msg.data.push_back(spline);
    traj_msg.data[dim].t_total = traj_dis_msg.t[traj_dis_msg.t.size() - 1];
  }
  // calc coeff
  for (int traj_idx = 0; traj_idx < traj_dis_msg.pos.size() - 1; traj_idx++) {
    // Eigen::MatrixXd pos_mat = Eigen::MatrixXd::Zero(degree_plus1, 3);
    // create a vector of pos, vel, acc for traj_idx and next point
    Eigen::VectorXd state1(9);
    Eigen::VectorXd state2(9);
    state1 << traj_dis_msg.pos[traj_idx].x, traj_dis_msg.pos[traj_idx].y,
        traj_dis_msg.pos[traj_idx].z, traj_dis_msg.vel[traj_idx].x,
        traj_dis_msg.vel[traj_idx].y, traj_dis_msg.vel[traj_idx].z,
        traj_dis_msg.acc[traj_idx].x, traj_dis_msg.acc[traj_idx].y,
        traj_dis_msg.acc[traj_idx].z;
    state2 << traj_dis_msg.pos[traj_idx + 1].x,
        traj_dis_msg.pos[traj_idx + 1].y, traj_dis_msg.pos[traj_idx + 1].z,
        traj_dis_msg.vel[traj_idx + 1].x, traj_dis_msg.vel[traj_idx + 1].y,
        traj_dis_msg.vel[traj_idx + 1].z, traj_dis_msg.acc[traj_idx + 1].x,
        traj_dis_msg.acc[traj_idx + 1].y, traj_dis_msg.acc[traj_idx + 1].z;
    std::cout << "state1: \n" << state1.transpose() << std::endl;
    std::cout << "state2: \n" << state2.transpose() << std::endl;
    // solve for coefficients
    Eigen::MatrixXd coeff_mat = compute_bvp_pva(state1,  // p0 v0 a0, size 9
                                                state2,
                                                dt);

    // 1.0);  // #this is because the whole thing is a unit coeff
    normalizePosCoeffMat(dt, coeff_mat);
    //
    // input into the new message
    for (int dim = 0; dim < 3; dim++) {
      traj_msg.data[dim].segments++;
      kr_planning_msgs::Polynomial p;
      p.basis = p.STANDARD;
      Eigen::VectorXd coeff_dim = coeff_mat.col(dim);
      std::vector<float> std_vector(coeff_dim.data(),
                                    coeff_dim.data() + coeff_dim.size());
      p.coeffs = std_vector;
      p.degree = degree_plus1 - 1;
      p.dt = dt;
      // p.start_index = traj_idx;
      traj_msg.data[dim].segs.push_back(p);
    }
  }

  return traj_msg;
}
kr_planning_msgs::SplineTrajectory
SplineTrajfromDiscrete(  // this method will make beginning and end have an
                         // nonzero velocity issue
    const kr_planning_msgs::TrajectoryDiscretized& traj_dis_msg) {
  int degree_plus1 = 6;
  double dt = traj_dis_msg.t[degree_plus1 - 1];
  kr_planning_msgs::SplineTrajectory traj_msg;
  traj_msg.header = traj_dis_msg.header;
  traj_msg.dimensions = 3;
  // prepare polynomial fit time vector
  Eigen::VectorXd time_vec = Eigen::ArrayXd::LinSpaced(degree_plus1, 0, 1);
  // create time matrix by taking this to different powers
  Eigen::MatrixXd time_mat = Eigen::MatrixXd::Zero(degree_plus1, degree_plus1);
  for (int i = 0; i < degree_plus1; i++) {
    time_mat.col(i) = time_vec.array().pow(i);
  }
  kr_planning_msgs::Spline spline;
  for (int dim = 0; dim < 3; dim++) traj_msg.data.push_back(spline);
  // ROS_INFO("Time matrix is %f", time_mat);
  for (int traj_idx = 0;
       traj_idx + (degree_plus1 - 1) < traj_dis_msg.pos.size();  // in range
       traj_idx += (degree_plus1 - 1)) {
    // this is to have 1 repeat point to make sure things connect
    for (int dim = 0; dim < 3; dim++)
      traj_msg.data[dim].t_total = traj_dis_msg.t[traj_idx + degree_plus1];
    // if (traj_idx + (degree_plus1-1) >= traj_dis_msg.pos.size()) {

    //   break;
    // }
    Eigen::MatrixXd pos_mat = Eigen::MatrixXd::Zero(degree_plus1, 3);

    // create a vector of positions
    for (int j = 0; j < degree_plus1; j++) {
      pos_mat(j, 0) = traj_dis_msg.pos[traj_idx + j].x;
      pos_mat(j, 1) = traj_dis_msg.pos[traj_idx + j].y;
      pos_mat(j, 2) = traj_dis_msg.pos[traj_idx + j].z;
    }
    // solve for coefficients
    Eigen::MatrixXd coeff_mat = time_mat.inverse() * pos_mat;
    // input into the new message
    for (int dim = 0; dim < 3; dim++) {
      traj_msg.data[dim].segments++;
      kr_planning_msgs::Polynomial p;
      p.basis = p.STANDARD;
      Eigen::VectorXd coeff_dim = coeff_mat.col(dim);
      std::vector<float> std_vector(coeff_dim.data(),
                                    coeff_dim.data() + coeff_dim.size());
      p.coeffs = std_vector;
      p.degree = degree_plus1 - 1;
      p.dt = dt;
      // p.start_index = traj_idx;
      traj_msg.data[dim].segs.push_back(p);
    }
  }

  return traj_msg;
}

void LocalPlanServer::process_result(
    const std::pair<kr_planning_msgs::SplineTrajectory,
                    kr_planning_msgs::TrajectoryDiscretized>& traj_combined,
    ros::Duration execution_time,
    int epoch) {
  kr_planning_msgs::SplineTrajectory traj_msg = traj_combined.first;
  kr_planning_msgs::TrajectoryDiscretized traj_dis_msg = traj_combined.second;
  bool solved = traj_msg.data.size() > 0;
  if (!solved && traj_dis_msg.pos.size() > 0) {
    traj_msg = SplineTrajfromDiscreteTwoPoints(traj_dis_msg);
    solved = true;
  }
  if (!solved) {
    // local plan fails
    aborted_ = true;
    if (local_as_->isActive()) {
      ROS_WARN("Current local plan trial failed!");
      traj_pub_.publish(kr_planning_msgs::SplineTrajectory());
      local_as_->setAborted();
    }
  } else {
    ROS_WARN("[LocalPlanServer] Planning success!!!!!!");
    traj_pub_.publish(traj_msg);

    /////////////////////////////////////////////////////////////////////////////////////////////
    // get the trajectory
    kr_tracker_msgs::PolyTrackerActionGoal traj_act_msg;

    traj_act_msg.goal.order = traj_msg.data[0].segs[0].degree;
    traj_act_msg.goal.set_yaw = false;

    int piece_num = traj_msg.data[0].segments;

    traj_act_msg.goal.t_start = ros::Time::now();  // spline_traj_.header.stamp
    traj_act_msg.goal.seg_x.resize(piece_num);
    traj_act_msg.goal.seg_y.resize(piece_num);
    traj_act_msg.goal.seg_z.resize(piece_num);

    std::cout << " piece_num  " << piece_num << std::endl;

    for (int i = 0; i < piece_num; ++i) {
      for (uint c = 0; c <= traj_act_msg.goal.order; c++) {
        traj_act_msg.goal.seg_x[i].coeffs.push_back(
            traj_msg.data[0].segs[i].coeffs[c]);
        traj_act_msg.goal.seg_y[i].coeffs.push_back(
            traj_msg.data[1].segs[i].coeffs[c]);
        traj_act_msg.goal.seg_z[i].coeffs.push_back(
            traj_msg.data[2].segs[i].coeffs[c]);
      }

      traj_act_msg.goal.seg_x[i].dt = traj_msg.data[0].segs[i].dt;
      traj_act_msg.goal.seg_x[i].degree = traj_msg.data[0].segs[i].degree;
    }

    // publish the trajectory
    traj_goal_pub_.publish(traj_act_msg);
    std_srvs::Trigger trg;
    ros::service::call(poly_srv_name_, trg);
    /////////////////////////////////////////////////////////////////////////////////////////////

    kr_planning_msgs::PlanTwoPointResult result;
    // evaluate trajectory for 5 steps, each step duration equals
    // execution_time, get corresponding waypoints and record in result
    // (result_->p_stop etc.) (evaluate the whole traj if execution_time is not
    // set (i.e. not in replan mode))
    double endt = execution_time.toSec();
    int num_goals = 5;
    if (endt <= 0) {
      endt = traj_total_time_;
      num_goals = 1;
    }

    for (int i = 0; i < num_goals; i++) {
      geometry_msgs::Pose p_fin;
      geometry_msgs::Twist v_fin, a_fin, j_fin;

      MPL::Waypoint3D pt_f =
          planner_->evaluate(endt * static_cast<double>(i + 1));
      // check if evaluation is successful, if not, set result.success to be
      // false! (if failure case, a null Waypoint is returned)
      if ((pt_f.pos(0) == 0) && (pt_f.pos(1) == 0) && (pt_f.pos(2) == 0) &&
          (pt_f.vel(0) == 0) && (pt_f.vel(1) == 0) && (pt_f.vel(2) == 0)) {
        result.success = 0;
        ROS_WARN_STREAM(
            "waypoint evaluation failed, set result.success to be false");
        ROS_WARN_STREAM("trajectory total time:" << traj_total_time_);
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

    MPL::Waypoint3D pt = planner_->evaluate(traj_total_time_);
    result.traj_end.position.x = pt.pos(0);
    result.traj_end.position.y = pt.pos(1);
    result.traj_end.position.z = pt.pos(2);
    // record trajectory in result
    result.success = solved;  // set success status
    result.policy_status = solved ? 1 : -1;

    result.traj = traj_msg;
    result.traj.header.frame_id = frame_id_;

    // execution_time (set in replanner)
    // equals 1.0/local_replan_rate
    result.execution_time =
        execution_time;  // execution_time (set in replanner)
                         // equals 1.0/local_replan_rate

    result.epoch = epoch;
    result.traj_end.orientation.w = 1.0;
    result.traj_end.orientation.z = 0;
    result.computation_time = computation_time_;
    result.compute_time_front_end = compute_time_front_end_;
    result.compute_time_back_end = compute_time_back_end_;
    if (local_as_->isActive()) {
      local_as_->setSucceeded(result);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_planner");

  ros::NodeHandle nh("~");

  LocalPlanServer tpp(nh);

  ros::Rate r(20);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

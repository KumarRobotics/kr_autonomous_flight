#include <action_planner/local_plan_server.h>

LocalPlanServer::LocalPlanServer(const ros::NodeHandle& nh) : pnh_(nh) {
  local_map_sub_ =
      pnh_.subscribe("local_voxel_map", 2, &LocalPlanServer::localMapCB, this);
  local_as_ = std::make_unique<
      actionlib::SimpleActionServer<kr_planning_msgs::PlanTwoPointAction>>(
      pnh_, "plan_local_trajectory", false);
  traj_planner_nh_ = ros::NodeHandle(pnh_, "trajectory_planner");
  sg_pub = pnh_.advertise<kr_planning_msgs::Path>("start_goal", 1, true);

  local_map_cleared_pub_ = pnh_.advertise<kr_planning_msgs::VoxelMap>(
      "local_voxel_map_cleared", 1, true);
  local_as_->registerGoalCallback(boost::bind(&LocalPlanServer::goalCB, this));
  while (local_map_ptr_ == nullptr) {
    ROS_WARN("[Local plan server]: Waiting for local map...");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }
  int planner_type_id;
  traj_planner_nh_.param("planner_type", planner_type_id, 0);

  switch (planner_type_id) {
    case 0:
      planner_type_ = new MPLPlanner(traj_planner_nh_, frame_id_);
      break;
    case 1:
      planner_type_ = new OptPlanner(traj_planner_nh_, frame_id_);
      break;
    case 2:
      planner_type_ = new DispersionPlanner(traj_planner_nh_, frame_id_);
      break;
    default:
      ROS_ERROR("Invalid planner type id: %d", planner_type_id);
      break;
  }

  planner_type_->setup();
  local_as_->start();
}

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCB(
    const kr_planning_msgs::VoxelMap::ConstPtr& msg) {
  ROS_WARN_ONCE("[Local planner:] Got the local voxel map!");
  local_map_ptr_ = msg;
  frame_id_ = local_map_ptr_->header.frame_id;
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
  } else {
    process_goal(*goal_ptr);
  }
  auto end_timer = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_timer - start_timer);
  ROS_INFO("Local goalCB took %f ms", duration.count() / 1000.0);
  planner_type_->setGoal(*goal_ptr);
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

  kr_planning_msgs::VoxelMap local_map_cleared;
  local_map_cleared = clear_map_position(*local_map_ptr_, start.pos);

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

  process_result(planner_type_->plan(start, goal, local_map_cleared),
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

void LocalPlanServer::process_result(
    const kr_planning_msgs::SplineTrajectory& traj_msg,
    ros::Duration execution_time,
    int epoch) {
  bool solved = traj_msg.data.size() > 0;
  if (!solved) {
    // local plan fails
    aborted_ = true;
    if (local_as_->isActive()) {
      ROS_WARN("Current local plan trial failed!");
      local_as_->setAborted();
    }
  }

  kr_planning_msgs::PlanTwoPointResult result;
  if (solved) {
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
          planner_type_->evaluate(endt * static_cast<double>(i + 1));
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

    MPL::Waypoint3D pt = planner_type_->evaluate(traj_total_time_);
    result.traj_end.position.x = pt.pos(0);
    result.traj_end.position.y = pt.pos(1);
    result.traj_end.position.z = pt.pos(2);
    // record trajectory in result
    result.success = solved;  // set success status
    result.policy_status = solved ? 1 : -1;

    result.traj = traj_msg;
    result.traj.header.frame_id = frame_id_;
    traj_opt::TrajRosBridge::publish_msg(result.traj);

    // execution_time (set in replanner)
    // equals 1.0/local_replan_rate
    result.execution_time =
        execution_time;  // execution_time (set in replanner)
                         // equals 1.0/local_replan_rate

    result.epoch = epoch;
    result.traj_end.orientation.w = 1.0;
    result.traj_end.orientation.z = 0;
  }

  // abort if trajectory generation failed
  if (!solved && local_as_->isActive()) {
    ROS_WARN("Current local plan trial: trajectory generation failed!");
    local_as_->setAborted();
  }

  if (local_as_->isActive()) {
    local_as_->setSucceeded(result);
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
#include <action_planner/local_plan_server.h>

#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <functional>
#include <thread>

LocalPlanServer::LocalPlanServer() : rclcpp::Node("action_planner") {
  local_map_sub_ =
      this->create_subscription<kr_planning_msgs::msg::VoxelMap>(
          "local_voxel_map",
          2,
          std::bind(&LocalPlanServer::localMapCB,
                    this,
                    std::placeholders::_1));
  local_no_infla_map_sub_ =
      this->create_subscription<kr_planning_msgs::msg::VoxelMap>(
          "local_noinfla_voxel_map",
          2,
          std::bind(&LocalPlanServer::localMapCBNoInfla,
                    this,
                    std::placeholders::_1));
  srv_transition_ = this->create_client<kr_tracker_msgs::srv::Transition>(
      "/quadrotor/trackers_manager/transition");
  while (!srv_transition_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted waiting for transition service.");
      break;
    }
    RCLCPP_INFO(this->get_logger(),
                "Waiting for transition service to come up...");
  }

  traj_goal_ac_ = rclcpp_action::create_client<PolyTracker>(
      this, "tracker_client");

  sg_pub = this->create_publisher<kr_planning_msgs::msg::Path>(
      "start_goal", rclcpp::QoS(1).transient_local());

  local_map_cleared_pub_ =
      this->create_publisher<kr_planning_msgs::msg::VoxelMap>(
          "local_voxel_map_cleared", rclcpp::QoS(1).transient_local());
  traj_pub_ = this->create_publisher<kr_planning_msgs::msg::SplineTrajectory>(
      "trajectory", rclcpp::QoS(1).transient_local());

  while ((local_map_ptr_ == nullptr || local_nofla_map_ptr_ == nullptr) &&
         rclcpp::ok()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        4000,
        "[Local plan server]: Waiting for local map either inflated or not...");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(this->get_node_base_interface());
  }

  this->declare_parameter<std::string>("poly_srv_name", std::string(" "));
  this->get_parameter("poly_srv_name", poly_srv_name_);
  this->declare_parameter<bool>("trajectory_planner.use_discrete_traj", false);
  this->get_parameter("trajectory_planner.use_discrete_traj",
                      use_discrete_traj_);
  this->declare_parameter<bool>("trajectory_planner.use_tracker_client", true);
  this->get_parameter("trajectory_planner.use_tracker_client",
                      use_tracker_client_);

  planner_ = new CompositePlanner(this, frame_id_);
  RCLCPP_WARN(this->get_logger(), "[Local planner:] planner instance created!");

  planner_->setup();
  RCLCPP_WARN(this->get_logger(), "[Local planner:] setup-ed!");

  local_as_ = rclcpp_action::create_server<PlanTwoPoint>(
      this,
      "plan_local_trajectory",
      std::bind(&LocalPlanServer::handleGoal,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&LocalPlanServer::handleCancel,
                this,
                std::placeholders::_1),
      std::bind(&LocalPlanServer::handleAccepted,
                this,
                std::placeholders::_1));

  RCLCPP_WARN(this->get_logger(), "[Local planner:] Initialized!");
}

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCB(
    const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg) {
  RCLCPP_WARN_ONCE(this->get_logger(),
                   "[Local planner:] Got the local voxel map!");
  local_map_ptr_ = msg;
  frame_id_ = local_map_ptr_->header.frame_id;
}

// map callback, update local_map_ptr_
void LocalPlanServer::localMapCBNoInfla(
    const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg) {
  RCLCPP_WARN_ONCE(
      this->get_logger(),
      "[Local planner:] Got the local voxel map without inflation!!!");
  local_nofla_map_ptr_ = msg;
  frame_id_ = local_nofla_map_ptr_->header.frame_id;
}

rclcpp_action::GoalResponse LocalPlanServer::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const PlanTwoPoint::Goal> /*goal*/) {
  RCLCPP_INFO(this->get_logger(), "[LocalPlanServer] Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LocalPlanServer::handleCancel(
    const std::shared_ptr<PlanTwoPointGoalHandle> /*goal_handle*/) {
  RCLCPP_INFO(this->get_logger(), "[LocalPlanServer] Cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LocalPlanServer::handleAccepted(
    const std::shared_ptr<PlanTwoPointGoalHandle> goal_handle) {
  // run execute in a detached thread so we don't block the executor
  std::thread{std::bind(&LocalPlanServer::execute, this, std::placeholders::_1),
              goal_handle}
      .detach();
}

void LocalPlanServer::execute(
    const std::shared_ptr<PlanTwoPointGoalHandle> goal_handle) {
  auto start_timer = std::chrono::high_resolution_clock::now();
  current_goal_handle_ = goal_handle;
  aborted_ = false;
  const auto goal = goal_handle->get_goal();
  if (goal == nullptr) {
    RCLCPP_WARN(this->get_logger(),
                "+++++++++++++++++++++++++++++++++++");
    RCLCPP_WARN(this->get_logger(),
                "[LocalPlanServer:] Goal is null!!!!!");
    RCLCPP_WARN(this->get_logger(),
                "+++++++++++++++++++++++++++++++++++");
  } else if (local_map_ptr_ == nullptr) {
    RCLCPP_WARN(this->get_logger(),
                "+++++++++++++++++++++++++++++++++++");
    RCLCPP_WARN(this->get_logger(),
                "[LocalPlanServer:] local map is not received!!!!!");
    RCLCPP_WARN(this->get_logger(),
                "+++++++++++++++++++++++++++++++++++");
  } else if (local_nofla_map_ptr_ == nullptr) {
    RCLCPP_WARN(this->get_logger(),
                "+++++++++++++++++++++++++++++++++++");
    RCLCPP_WARN(this->get_logger(),
                "[LocalPlanServer:] local no inflation map is not received!!!!!");
    RCLCPP_WARN(this->get_logger(),
                "+++++++++++++++++++++++++++++++++++");
  } else {
    process_goal(*goal);
  }
  auto end_timer = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      end_timer - start_timer);
  RCLCPP_INFO(
      this->get_logger(), "Local goalCB took %f ms", duration.count() / 1000.0);
  computation_time_ = duration.count() / 1000.0;
  if (goal != nullptr) {
    planner_->setGoal(*goal);
  }
}

void LocalPlanServer::process_goal(const PlanTwoPoint::Goal& as_goal) {
  std::lock_guard<std::mutex> lockt(traj_mtx);
  if (aborted_) {
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
      RCLCPP_WARN(this->get_logger(), "process_goal: Abort!");
      auto res = std::make_shared<PlanTwoPoint::Result>();
      current_goal_handle_->abort(res);
    }
    return;
  }

  MPL::Waypoint3D start, goal;
  start.pos = kr::pose_to_eigen(as_goal.p_init);
  start.vel = kr::twist_to_eigen(as_goal.v_init);
  start.acc = kr::twist_to_eigen(as_goal.a_init);
  start.jrk = kr::twist_to_eigen(as_goal.j_init);

  start.use_pos = true;
  start.use_vel = true;
  start.use_acc = true;
  start.use_jrk = false;
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

  kr_planning_msgs::msg::VoxelMap local_map_cleared, local_no_infla_map_cleared;
  local_map_cleared = clear_map_position(*local_map_ptr_, start.pos);
  local_no_infla_map_cleared =
      clear_map_position(*local_nofla_map_ptr_, start.pos);

  if (pub_cleared_map_) {
    local_map_cleared_pub_->publish(local_map_cleared);
    RCLCPP_ERROR(this->get_logger(), "Local map cleared published");
  }

  vec_Vec3f sg;
  sg.push_back(start.pos);
  sg.push_back(goal.pos);
  kr_planning_msgs::msg::Path sg_msg = kr::path_to_ros(sg);
  sg_msg.header.frame_id = frame_id_;
  sg_pub->publish(sg_msg);

  process_result(planner_->plan_composite(start,
                                          goal,
                                          local_map_cleared,
                                          local_no_infla_map_cleared,
                                          &compute_time_front_end_,
                                          &compute_time_back_end_,
                                          &compute_time_poly_,
                                          success_status_),
                 as_goal.execution_time,
                 as_goal.epoch);
}

kr_planning_msgs::msg::VoxelMap LocalPlanServer::clear_map_position(
    const kr_planning_msgs::msg::VoxelMap& local_map_original,
    const Vec3f& start) {
  kr_planning_msgs::msg::VoxelMap local_map_cleared;
  local_map_cleared = local_map_original;

  int8_t val_free = kr_planning_msgs::msg::VoxelMap::VAL_FREE;
  RCLCPP_WARN_ONCE(this->get_logger(), "Value free is set as %d", val_free);
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

Eigen::MatrixXd computeShotTraj(Eigen::VectorXd state1,
                                Eigen::VectorXd state2,
                                double time_to_goal) {
  using namespace Eigen;
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d dv = state2.segment(3, 3) - v0;
  const Vector3d a0 = state1.tail(3);
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

kr_planning_msgs::msg::SplineTrajectory SplineTrajfromDiscreteTwoPoints(
    const kr_planning_msgs::msg::TrajectoryDiscretized& traj_dis_msg) {
  (void)traj_dis_msg;
  kr_planning_msgs::msg::SplineTrajectory traj_msg;
  return traj_msg;
}

kr_planning_msgs::msg::SplineTrajectory SplineTrajfromDiscrete(
    const kr_planning_msgs::msg::TrajectoryDiscretized& traj_dis_msg) {
  int degree_plus1 = 6;
  double dt = traj_dis_msg.dt;
  kr_planning_msgs::msg::SplineTrajectory traj_msg;
  traj_msg.header = traj_dis_msg.header;
  traj_msg.dimensions = 3;
  Eigen::VectorXd time_vec = Eigen::ArrayXd::LinSpaced(degree_plus1, 0, 1);
  Eigen::MatrixXd time_mat = Eigen::MatrixXd::Zero(degree_plus1, degree_plus1);
  for (int i = 0; i < degree_plus1; i++) {
    time_mat.col(i) = time_vec.array().pow(i);
  }
  kr_planning_msgs::msg::Spline spline;
  for (int dim = 0; dim < 3; dim++) traj_msg.data.push_back(spline);
  for (int traj_idx = 0;
       traj_idx + (degree_plus1 - 1) <
       static_cast<int>(traj_dis_msg.pos.size());
       traj_idx += (degree_plus1 - 1)) {
    for (int dim = 0; dim < 3; dim++)
      traj_msg.data[dim].t_total = (traj_idx + (degree_plus1 - 1)) * dt;

    Eigen::MatrixXd pos_mat = Eigen::MatrixXd::Zero(degree_plus1, 3);

    for (int j = 0; j < degree_plus1; j++) {
      pos_mat(j, 0) = traj_dis_msg.pos[traj_idx + j].x;
      pos_mat(j, 1) = traj_dis_msg.pos[traj_idx + j].y;
      pos_mat(j, 2) = traj_dis_msg.pos[traj_idx + j].z;
    }
    Eigen::MatrixXd coeff_mat = time_mat.inverse() * pos_mat;
    for (int dim = 0; dim < 3; dim++) {
      traj_msg.data[dim].segments++;
      kr_planning_msgs::msg::Polynomial p;
      p.basis = p.STANDARD;
      Eigen::VectorXd coeff_dim = coeff_mat.col(dim);
      std::vector<float> std_vector(coeff_dim.data(),
                                    coeff_dim.data() + coeff_dim.size());
      p.coeffs = std_vector;
      p.degree = degree_plus1 - 1;
      p.dt = dt * (degree_plus1 - 1);
      traj_msg.data[dim].segs.push_back(p);
    }
  }

  return traj_msg;
}

void LocalPlanServer::process_result(
    const std::tuple<kr_planning_msgs::msg::SplineTrajectory,
                     kr_planning_msgs::msg::SplineTrajectory,
                     kr_planning_msgs::msg::TrajectoryDiscretized>&
        traj_combined,
    const rclcpp::Duration& execution_time,
    int epoch) {
  kr_planning_msgs::msg::SplineTrajectory search_traj_msg =
      std::get<0>(traj_combined);
  kr_planning_msgs::msg::SplineTrajectory traj_msg = std::get<1>(traj_combined);
  kr_planning_msgs::msg::TrajectoryDiscretized traj_dis_msg =
      std::get<2>(traj_combined);
  double tracking_error = 0.0;
  std::vector<geometry_msgs::msg::Point> odom_pts;

  bool solved = traj_msg.data.size() > 0;
  if (!solved && traj_dis_msg.pos.size() > 0) {
    traj_msg = SplineTrajfromDiscrete(traj_dis_msg);
    solved = true;
  }

  if (!solved) {
    aborted_ = true;
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
      RCLCPP_WARN(this->get_logger(), "Current local plan trial failed!");
      traj_pub_->publish(kr_planning_msgs::msg::SplineTrajectory());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "[LocalPlanServer] Planning success!!!!!!");
    traj_pub_->publish(traj_msg);

    PolyTracker::Goal traj_goal;
    traj_goal.set_yaw = false;
    traj_goal.t_start = this->now();

    if (!use_discrete_traj_) {
      traj_goal.order = traj_msg.data[0].segs[0].degree;

      int piece_num = traj_msg.data[0].segments;

      traj_goal.seg_x.resize(piece_num);
      traj_goal.seg_y.resize(piece_num);
      traj_goal.seg_z.resize(piece_num);

      std::cout << "piece_num  " << piece_num << std::endl;

      for (int i = 0; i < piece_num; ++i) {
        for (uint c = 0; c <= traj_goal.order; c++) {
          traj_goal.seg_x[i].coeffs.push_back(traj_msg.data[0].segs[i].coeffs[c]);
          traj_goal.seg_y[i].coeffs.push_back(traj_msg.data[1].segs[i].coeffs[c]);
          traj_goal.seg_z[i].coeffs.push_back(traj_msg.data[2].segs[i].coeffs[c]);
        }

        traj_goal.seg_x[i].dt = traj_msg.data[0].segs[i].dt;
        traj_goal.seg_x[i].degree = traj_msg.data[0].segs[i].degree;
      }
    } else {
      traj_goal.N = traj_dis_msg.N_ctrl + 1;
      traj_goal.pos_pts = traj_dis_msg.pos;
      traj_goal.vel_pts = traj_dis_msg.vel;
      traj_goal.acc_pts = traj_dis_msg.acc;
      traj_goal.dt = traj_dis_msg.dt;
    }

    if (use_tracker_client_ == false) {
      RCLCPP_WARN(this->get_logger(),
                  "Not using tracker client, publish trajectory directly");
    } else {
      std::string tracker_str = "kr_trackers/PolyTracker";
      auto transition_req =
          std::make_shared<kr_tracker_msgs::srv::Transition::Request>();
      transition_req->tracker = tracker_str;

      auto send_goal_future = traj_goal_ac_->async_send_goal(traj_goal);
      if (send_goal_future.wait_for(std::chrono::seconds(1)) !=
          std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to send goal to PolyTracker within 1s");
      }
      auto goal_handle = send_goal_future.get();

      auto srv_future = srv_transition_->async_send_request(transition_req);
      if (srv_future.wait_for(std::chrono::seconds(2)) ==
          std::future_status::ready) {
        auto response = srv_future.get();
        if (response->success)
          RCLCPP_INFO(this->get_logger(), "Transition success!");
      }

      if (goal_handle) {
        auto result_future = traj_goal_ac_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(20)) ==
            std::future_status::ready) {
          auto wrapped_result = result_future.get();
          if (wrapped_result.result) {
            tracking_error = wrapped_result.result->total_tracking_error;
            odom_pts = wrapped_result.result->odom_history;
            RCLCPP_INFO(this->get_logger(),
                        "Poly Tracker finished: Tracking Error = %f",
                        tracking_error);
          }
        }
      }
    }
  }

  auto result = std::make_shared<PlanTwoPoint::Result>();
  double dt = execution_time.seconds();
  int num_goals = 5;
  if (dt <= 0) {
    dt = 0.2;
    num_goals = static_cast<int>(traj_total_time_ / 0.2);
  }
  (void)num_goals;
  std::cout << "num goals set" << std::endl;

  result->tracking_error = tracking_error;
  result->odom_pts = odom_pts;
  result->success = solved;
  result->policy_status = success_status_;
  result->search_traj = search_traj_msg;
  result->traj = traj_msg;
  result->traj.header.frame_id = frame_id_;

  result->execution_time = execution_time;

  result->epoch = epoch;
  result->traj_end.orientation.w = 1.0;
  result->traj_end.orientation.z = 0;
  result->computation_time = compute_time_poly_;
  result->compute_time_front_end = compute_time_front_end_;
  result->compute_time_back_end = compute_time_back_end_;
  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    RCLCPP_INFO(this->get_logger(),
                "Current local plan trial is active & succeeded!");
    current_goal_handle_->succeed(result);
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Current local plan trial is not active !");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalPlanServer>();
  rclcpp::Rate r(20);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}

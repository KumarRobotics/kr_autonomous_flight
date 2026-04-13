#include <cmath>
#include <chrono>
#include <state_machine/local_global_replan_server.hpp>

using namespace std::chrono_literals;

/**
 * @brief Synchronously send a PlanTwoPoint goal and wait for a result.
 * Returns true if the result arrives within the timeout. On success,
 * writes the result pointer to *out_result. This function spins the node
 * while waiting to preserve the blocking semantics of the ROS1 code.
 *
 * NOTE: Because RePlanner is driven from the main loop (which spins the node
 * explicitly), we cannot call spin_until_future_complete here since that
 * would re-enter the executor. Instead we repeatedly call
 * rclcpp::spin_some on a short interval until the result is ready or the
 * timeout expires.
 */
bool RePlanner::SendAndWaitPlanTwoPoint(
    rclcpp_action::Client<PlanTwoPoint>::SharedPtr client,
    const PlanTwoPoint::Goal& goal, double timeout_sec,
    PlanTwoPoint::Result::SharedPtr* out_result) {
  if (out_result) *out_result = nullptr;

  bool is_global = (client == global_plan_client_);
  if (is_global) {
    latest_global_result_ready_ = false;
    latest_global_result_ = nullptr;
  } else {
    latest_local_result_ready_ = false;
    latest_local_result_ = nullptr;
  }

  auto send_goal_options =
      rclcpp_action::Client<PlanTwoPoint>::SendGoalOptions();
  send_goal_options.result_callback =
      [this, is_global](
          const rclcpp_action::ClientGoalHandle<PlanTwoPoint>::WrappedResult&
              result) {
        if (is_global) {
          latest_global_result_ = result.result;
          latest_global_result_ready_ = true;
        } else {
          latest_local_result_ = result.result;
          latest_local_result_ready_ = true;
        }
      };
  client->async_send_goal(goal, send_goal_options);

  rclcpp::Time start = this->now();
  rclcpp::Rate r(100);
  while (rclcpp::ok()) {
    bool ready = is_global ? latest_global_result_ready_
                           : latest_local_result_ready_;
    if (ready) break;
    double elapsed = (this->now() - start).seconds();
    if (elapsed > timeout_sec) return false;
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
  }
  if (is_global) {
    if (out_result) *out_result = latest_global_result_;
  } else {
    if (out_result) *out_result = latest_local_result_;
  }
  return (out_result && *out_result != nullptr);
}

void RePlanner::GlobalPathCb(
    const kr_planning_msgs::msg::Path::ConstSharedPtr path) {
  global_path_.clear();
  global_path_ = kr::ros_to_path(*path);  // extract the global path information
}

void RePlanner::EpochCb(const std_msgs::msg::Int64::ConstSharedPtr msg) {
  if (msg->data < 0) {
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++++++++++++");
    RCLCPP_ERROR(this->get_logger(),
                 "aborting mission because tracker failed!!!");
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++++++++++++");
    AbortReplan();
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  static int epoch_old = -1;  // keep a record of last epoch

  // replan ONLY IF current epoch is different than the previously recorded one
  if (epoch_old != msg->data) {
    int horizon;
    if (epoch_old == -1) {
      horizon = 1;
    } else {
      horizon = msg->data - last_plan_epoch_ + 1;
    }
    epoch_old = msg->data;  // keep a record of last epoch

    // trigger replan, set horizon
    if (!finished_replanning_) {
      if (PlanTrajectory(horizon))
        // execute the replanned trajectory
        RunTrajectoryFn();
    }
  }
  update_status();
}

void RePlanner::CmdCb(
    const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr cmd) {
  std::lock_guard<std::mutex> lock(mtx_);
  cmd_pos_(0) = cmd->position.x;
  cmd_pos_(1) = cmd->position.y;
  cmd_pos_(2) = cmd->position.z;
  cmd_pos_(3) = cmd->yaw;
}

// map callback, update local_map_
void RePlanner::LocalMapCb(
    const kr_planning_msgs::msg::VoxelMap::ConstSharedPtr msg) {
  if (map_counter_ == 0) {
    RCLCPP_WARN(this->get_logger(), "Got the first local voxel map!");
    // reset the local_map_last_timestamp_
    local_map_last_timestamp_ = this->now().seconds();
    total_map_time_ = 0;
  } else {
    double current_time = this->now().seconds();
    double time_duration = current_time - local_map_last_timestamp_;
    total_map_time_ += time_duration;
    double avg_time_per_map = total_map_time_ / map_counter_;
    avg_map_frequency_ = 1.0 / avg_time_per_map;
    // update last timestamp
    local_map_last_timestamp_ = current_time;
    // check frequency jumps
    double current_map_frequency = 1.0 / time_duration;
    // tolerance in update rate changes
    double percent_tol = 0.95;

    // only check if the duration is more than 0.1, if less, it means the map is
    // updated at more than 10Hz, no need to check
    if (time_duration > 0.1) {
      if (current_map_frequency > (1 + percent_tol) * avg_map_frequency_) {
        RCLCPP_WARN_STREAM(
            this->get_logger(),
            "Local map rate unstable and sharply increased! Probably due to "
            "LIDAR packets "
            "loss or computation, check "
            "(1) LIDAR connection, and (2) computation headroom!");
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Average update rate was: "
                               << avg_map_frequency_
                               << " Hz, most recent update rate is: "
                               << current_map_frequency << " Hz");
      } else if (current_map_frequency <
                 (1 - percent_tol) * avg_map_frequency_) {
        RCLCPP_ERROR(this->get_logger(),
                     "Local map rate sharply dropped! Stopping policy triggered!");
        RCLCPP_ERROR(this->get_logger(),
                     "Local map rate sharply dropped! Stopping policy triggered!");
        RCLCPP_ERROR(this->get_logger(),
                     "Probably due to LIDAR packets loss or computation, check "
                     "(1) LIDAR connection, and (2) computation headroom!");
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "Average update rate was: "
                                << avg_map_frequency_
                                << " Hz, most recent update rate is: "
                                << current_map_frequency << " Hz");
      } else {
        RCLCPP_INFO_STREAM_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Local map updated rate is stable, most recent update frequency "
            "is: "
                << current_map_frequency << " Hz");
      }
    }
  }
  map_counter_++;
  local_map_ptr_ = msg;
}

rclcpp_action::GoalResponse RePlanner::handleReplanGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const Replan::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RePlanner::handleReplanCancel(
    const std::shared_ptr<GoalHandleReplan> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RePlanner::handleReplanAccepted(
    const std::shared_ptr<GoalHandleReplan> goal_handle) {
  std::lock_guard<std::mutex> lock(mtx_);
  replan_goal_handle_ = goal_handle;
  auto goal = goal_handle->get_goal();

  // set local planner local_replan_rate to be the same as specified in goal
  local_replan_rate_ = goal->local_replan_rate;

  // set global planner replan rate (global_replan_rate_factor >= 1)
  global_replan_rate_factor_ = goal->global_replan_rate_factor;

  if (goal->continue_mission) {
    // this means continuing with previous mission
    RCLCPP_INFO(this->get_logger(),
                "Continuing to finish waypoints in existing mission...");
    if (pose_goals_.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Existing waypoint list is empty, aborting replan!");
      RCLCPP_WARN(
          this->get_logger(),
          "This can be caused by either clicking RESET MISSION, or clicking "
          "SKIP NEXT WAYPOINT and skipping all of your waypoints!");
      AbortFullMission();
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Last replan process has finished %d waypoints!",
                  cur_cb_waypoint_idx_);
    }
  } else {
    // reset cur_cb_waypoint_idx
    cur_cb_waypoint_idx_ = 0;
    RCLCPP_INFO(this->get_logger(),
                "Non-empty goal is sent, excecuting a new mission...");
    if ((goal->p_finals).empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "waypoints list is empty, now using single goal intead!");
      pose_goals_.push_back(goal->p_final);
    } else {
      pose_goals_ = goal->p_finals;
      RCLCPP_INFO(this->get_logger(),
                  "receive more than one waypoints! Number of waypoints: %zu",
                  pose_goals_.size());
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "new waypoints received, number of waypoints is:"
                           << pose_goals_.size());
  }

  pose_goal_ = pose_goals_[cur_cb_waypoint_idx_];
  TransformGlobalGoal();

  avoid_obstacle_ = goal->avoid_obstacles;

  // check cmd
  if (cmd_pos_.norm() == 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "RePlanner has not received position cmd, failing");
    AbortReplan();
    return;
  }

  if (!active_)        // if not active_, do setup again
    do_setup_ = true;  // only run setup_replanner function after ReplanGoalCb
}

void RePlanner::setup_replanner() {
  // check if local map is updated, if not, abort the mission
  double local_map_min_rate = 0.5;
  double local_map_time_elapsed =
      this->now().seconds() - local_map_last_timestamp_;
  if ((map_counter_ >= 1) &&
      (local_map_time_elapsed) > 1.0 / local_map_min_rate) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Local map has not been updated for"
                            << local_map_time_elapsed
                            << " seconds! Stopping policy triggered!");
    RCLCPP_ERROR(this->get_logger(),
                 "Probably due to LIDAR packets loss or computation, check "
                 "(1) LIDAR connection, and (2) computation headroom!");
  }

  if (!do_setup_ || active_) {
    return;
  }
  if (local_map_ptr_ == nullptr) {
    RCLCPP_WARN(this->get_logger(),
                "local_map_ptr_ is nullptr, local map not received yet!!!!!");
    return;
  }
  do_setup_ = false;  // only run setup_replanner once
  std::lock_guard<std::mutex> lock(mtx_);

  RCLCPP_INFO_STREAM(this->get_logger(), "Setup replan");
  if (!avoid_obstacle_) {
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++++++++++++");
    RCLCPP_WARN(this->get_logger(), "Obstacle avoidance is disabled!!!!!");
    RCLCPP_WARN(this->get_logger(), "+++++++++++++++++++++++++++++++++++");
  }

  if (waypoint_threshold_ < final_waypoint_threshold_) {
    RCLCPP_WARN(
        this->get_logger(),
        "waypoint_reach_threshold is set as smaller than "
        "final_waypoint_reach_threshold, this is not recommanded, now "
        "changing the final_waypoint_threshold to be the same as "
        "waypoint_threshold!");
    final_waypoint_threshold_ = waypoint_threshold_;
  }

  // Initial plan step 1: global plan
  float x_diff = (float)std::abs(pose_goal_wrt_odom_.position.x - cmd_pos_(0));
  float y_diff = (float)std::abs(pose_goal_wrt_odom_.position.y - cmd_pos_(1));
  Vec2f xy_diff = Vec2f{pose_goal_wrt_odom_.position.x - cmd_pos_(0),
                        pose_goal_wrt_odom_.position.y - cmd_pos_(1)};
  float dist_cmd_to_goal = xy_diff.norm();
  if (dist_cmd_to_goal <= waypoint_threshold_) {
    if ((cur_cb_waypoint_idx_ >=
         static_cast<int>(pose_goals_.size()) - 1) &&
        (dist_cmd_to_goal <= final_waypoint_threshold_)) {
      // exit replanning process if the final waypoint is reached
      RCLCPP_WARN(
          this->get_logger(),
          "Initial (and the only) waypoint x and y positions are already close "
          "to the robot, terminating the replanning process!");
      auto success = std::make_shared<Replan::Result>();
      success->status = Replan::Result::SUCCESS;
      active_ = false;
      if (replan_goal_handle_ && replan_goal_handle_->is_active()) {
        replan_goal_handle_->succeed(success);
        replan_goal_handle_.reset();
      }
      last_traj_ = boost::shared_ptr<traj_opt::Trajectory>();
      return;
    } else if (cur_cb_waypoint_idx_ <
               static_cast<int>(pose_goals_.size()) - 1) {
      // take the next waypoint if the intermidiate waypoint is reached
      RCLCPP_WARN(this->get_logger(),
                  "Initial waypoint is already close to the robot position, "
                  "continuing with the next waypoint!");
      cur_cb_waypoint_idx_ = cur_cb_waypoint_idx_ + 1;
      pose_goal_ = pose_goals_[cur_cb_waypoint_idx_];
      TransformGlobalGoal();
    }
  }

  // set goal
  PlanTwoPoint::Goal global_tpgoal;
  global_tpgoal.p_final = pose_goal_wrt_odom_;
  global_tpgoal.avoid_obstacles = avoid_obstacle_;

  // send goal to global plan action server
  // global initial plan timeout duration
  double initial_global_timeout_dur = 6.0 * local_timeout_duration_;
  PlanTwoPoint::Result::SharedPtr global_result;
  bool global_finished_before_timeout = SendAndWaitPlanTwoPoint(
      global_plan_client_, global_tpgoal, initial_global_timeout_dur,
      &global_result);
  // check result of global plan
  if (!global_finished_before_timeout) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "initial global planning timed out, its timeout duration is set as: "
            << initial_global_timeout_dur);
    AbortReplan();
    return;
  }
  if (global_result && global_result->success) {
    global_path_ = kr::ros_to_path(global_result->path);
    RCLCPP_WARN(this->get_logger(), "initial global plan succeeded!");
  } else {
    RCLCPP_WARN(this->get_logger(), "initial global plan failed!");
    AbortReplan();
    return;
  }

  // Initial plan step 2: Crop global path to get local goal
  vec_Vec3f global_path_wrt_map = TransformGlobalPath(global_path_);
  vec_Vec3f path_cropped_wrt_odom = PathCropIntersect(global_path_);

  if (path_cropped_wrt_odom.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Path crop failed!");
    AbortReplan();
    return;
  }

  bool close_to_final_goal;
  if (close_to_final_dist_ > 0) {
    close_to_final_goal =
        CloseToFinal(global_path_, path_cropped_wrt_odom, close_to_final_dist_);
  } else {
    close_to_final_dist_ = false;
  }
  // Initial plan step 3: local plan
  PlanTwoPoint::Goal local_tpgoal;
  state_machine::VecToPose(cmd_pos_, &local_tpgoal.p_init);
  prev_start_pos_(0) = cmd_pos_(0);
  prev_start_pos_(1) = cmd_pos_(1);
  prev_start_pos_(2) = cmd_pos_(2);

  local_tpgoal.epoch = 1;
  local_tpgoal.avoid_obstacles = avoid_obstacle_;
  local_tpgoal.execution_time =
      rclcpp::Duration::from_seconds(1.0 / local_replan_rate_).to_msg();
  local_tpgoal.check_vel = close_to_final_goal;
  Vec3f local_goal = path_cropped_wrt_odom.back();
  local_tpgoal.p_final.position.x = local_goal(0);
  local_tpgoal.p_final.position.y = local_goal(1);
  local_tpgoal.p_final.position.z = local_goal(2);

  // send goal to local trajectory plan action server
  double initial_local_timeout_dur = local_timeout_duration_ * 2.0;
  PlanTwoPoint::Result::SharedPtr local_result;
  bool local_finished_before_timeout = SendAndWaitPlanTwoPoint(
      local_plan_client_, local_tpgoal, initial_local_timeout_dur,
      &local_result);

  // reset the counter
  failed_local_trials_ = 0;

  if (!local_finished_before_timeout) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Initial local planning timed out, its timeout duration is set as: "
            << initial_local_timeout_dur);
    AbortReplan();
    return;
  }
  if (!local_result || !local_result->success) {
    RCLCPP_ERROR(this->get_logger(),
                 "Initial local planning failed to find a local trajectory!");
    AbortReplan();
    return;
  }

  // get the result
  last_traj_ = boost::make_shared<traj_opt::MsgTrajectory>(
      traj_opt::TrajDataFromSplineTrajectory(local_result->traj));
  last_plan_epoch_ = local_result->epoch;

  finished_replanning_ = false;

  RCLCPP_INFO(this->get_logger(), "Started replanning!");
  RunTrajectoryFn();
}

void RePlanner::RunTrajectoryFn() {
  if (!replan_goal_handle_ || !replan_goal_handle_->is_active()) {
    return;
  }

  // set up run goal
  RunTrajectory::Goal rungoal;
  rungoal.traj =
      traj_opt::SplineTrajectoryFromTrajData(last_traj_->serialize());
  rungoal.epoch = last_plan_epoch_;
  rungoal.local_replan_rate = local_replan_rate_;

  // call the trajectory tracker
  latest_run_result_ready_ = false;
  auto send_goal_options =
      rclcpp_action::Client<RunTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<
              RunTrajectory>::WrappedResult& result) {
        latest_run_result_code_ = result.code;
        latest_run_result_ready_ = true;
      };
  run_client_->async_send_goal(rungoal, send_goal_options);

  double tracker_timeout_dur = 0.5;
  rclcpp::Time start = this->now();
  rclcpp::Rate r(100);
  bool tracker_finished_before_timeout = false;
  while (rclcpp::ok()) {
    if (latest_run_result_ready_) {
      tracker_finished_before_timeout = true;
      break;
    }
    double elapsed = (this->now() - start).seconds();
    if (elapsed > tracker_timeout_dur) break;
    rclcpp::spin_some(this->get_node_base_interface());
    r.sleep();
  }
  if (!tracker_finished_before_timeout) {
    RCLCPP_ERROR(this->get_logger(), "Tracker aborted or timeout!");
    AbortReplan();
  }
}

bool RePlanner::PlanTrajectory(int horizon) {
  double local_map_min_rate = 0.5;
  double local_map_time_elapsed =
      this->now().seconds() - local_map_last_timestamp_;
  if ((map_counter_ >= 1) &&
      (local_map_time_elapsed) > 1.0 / local_map_min_rate) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Local map has not been updated for"
                            << local_map_time_elapsed
                            << " seconds! Stopping policy triggered!");
    RCLCPP_ERROR(this->get_logger(),
                 "Probably due to LIDAR packets loss or computation, check "
                 "(1) LIDAR connection, and (2) computation headroom!");
  }

  if (horizon > max_horizon_) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Planning horizon is larger than max_horizon_, aborting the mission!");
    auto abort = std::make_shared<Replan::Result>();
    abort->status = Replan::Result::DYNAMICALLY_INFEASIBLE;
    active_ = false;
    if (replan_goal_handle_ && replan_goal_handle_->is_active()) {
      replan_goal_handle_->abort(abort);
      replan_goal_handle_.reset();
    }
    return false;
  }

  // set global goal
  PlanTwoPoint::Goal global_tpgoal;
  global_tpgoal.p_final = pose_goal_wrt_odom_;
  global_tpgoal.avoid_obstacles = avoid_obstacle_;

  // set local goal
  PlanTwoPoint::Goal local_tpgoal;
  double eval_time = double(horizon) / local_replan_rate_;
  state_machine::EvaluateToMsgs(last_traj_,
                                eval_time,
                                &local_tpgoal.p_init,
                                &local_tpgoal.v_init,
                                &local_tpgoal.a_init,
                                &local_tpgoal.j_init);
  Vec3f start_pos;
  start_pos = kr::pose_to_eigen(local_tpgoal.p_init);

  // Replan step 1: Global plan (fire-and-forget; we do not synchronously wait)
  if (global_replan_rate_factor_ > 1) {
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        "[LG_replan_server]: Local replan rate is " << global_replan_rate_factor_
                                                     << "times global replan rate");
    if ((global_plan_counter_ % global_replan_rate_factor_) == 0) {
      // fire-and-forget global replan request; global path sub will receive
      // the resulting path via topic.
      auto opts = rclcpp_action::Client<PlanTwoPoint>::SendGoalOptions();
      global_plan_client_->async_send_goal(global_tpgoal, opts);
      vec_Vec3f global_path_wrt_map = TransformGlobalPath(global_path_);
    }
    global_plan_counter_++;
  } else {
    auto opts = rclcpp_action::Client<PlanTwoPoint>::SendGoalOptions();
    global_plan_client_->async_send_goal(global_tpgoal, opts);
    vec_Vec3f global_path_wrt_map = TransformGlobalPath(global_path_);
  }

  prev_start_pos_ = start_pos;

  // Replan step 2: Crop global path to get local goal
  vec_Vec3f path_cropped_wrt_odom = PathCropIntersect(global_path_);
  if (path_cropped_wrt_odom.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Path crop failed!");
    AbortReplan();
    return false;
  }

  bool close_to_final_goal;
  if (close_to_final_dist_ > 0) {
    close_to_final_goal =
        CloseToFinal(global_path_, path_cropped_wrt_odom, close_to_final_dist_);
  } else {
    close_to_final_dist_ = false;
  }

  // Replan step 3: local plan
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "[LG_replan_server]:local replan rate is "
                         << local_replan_rate_ << " Hz, horizon is " << horizon);
  local_tpgoal.avoid_obstacles = avoid_obstacle_;
  local_tpgoal.execution_time =
      rclcpp::Duration::from_seconds(1.0 / local_replan_rate_).to_msg();
  local_tpgoal.epoch = last_plan_epoch_ + horizon;
  local_tpgoal.check_vel = close_to_final_goal;
  Vec3f local_goal = path_cropped_wrt_odom.back();
  local_tpgoal.p_final.position.x = local_goal(0);
  local_tpgoal.p_final.position.y = local_goal(1);
  local_tpgoal.p_final.position.z = local_goal(2);

  timer.start();
  PlanTwoPoint::Result::SharedPtr local_result;
  bool local_finished_before_timeout = SendAndWaitPlanTwoPoint(
      local_plan_client_, local_tpgoal, local_timeout_duration_, &local_result);

  sensor_msgs::msg::Temperature tmsg2;
  tmsg2.header.stamp = this->now();
  tmsg2.header.frame_id = map_frame_;
  tmsg2.temperature = static_cast<double>(timer.elapsed().wall) / 1e6;

  // check result of local plan
  bool local_succeeded = true;
  if (!local_finished_before_timeout) {
    failed_local_trials_ += 1;
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Local planner timed out, trying to replan...  (local planner already "
        "failed "
            << failed_local_trials_
            << " times, total allowed trails: " << max_local_trials_ << ")");
    local_succeeded = false;
  } else {
    if (local_result && local_result->success) {
      last_traj_ = boost::make_shared<traj_opt::MsgTrajectory>(
          traj_opt::TrajDataFromSplineTrajectory(local_result->traj));
      last_plan_epoch_ = local_result->epoch;
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Got local plan with epoch " << last_plan_epoch_);
      failed_local_trials_ = 0;
      return true;
    } else {
      failed_local_trials_ += 1;
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Local planner failed, trying to replan...  (local planner already "
          "failed "
              << failed_local_trials_
              << " times, total allowed trails: " << max_local_trials_ << ")");
      local_succeeded = false;
    }
  }

  if (failed_local_trials_ >= max_local_trials_ - 1) {
    AbortReplan();
    return false;
  }

  return local_succeeded;
}

void RePlanner::update_status() {
  // check for termination
  if (last_traj_ != nullptr) {
    traj_opt::VecD pos_goal = traj_opt::VecD::Zero(4, 1);
    traj_opt::VecD pos_final, pos_finaln;
    pos_goal(0) = pose_goal_wrt_odom_.position.x;
    pos_goal(1) = pose_goal_wrt_odom_.position.y;

    last_traj_->evaluate(1.0 / local_replan_rate_, 0, pos_finaln);

    pos_final = state_machine::Make4d(pos_finaln);
    pos_final(2) = 0;
    pos_final(3) = 0;

    // check if goal and traj evaluated position is less than threshold
    double dist_cmd_to_goal = (pos_goal - pos_final).norm();
    if (dist_cmd_to_goal <= waypoint_threshold_) {
      if ((cur_cb_waypoint_idx_ >=
           static_cast<int>(pose_goals_.size()) - 1) &&
          (dist_cmd_to_goal <= final_waypoint_threshold_)) {
        finished_replanning_ = true;
        RCLCPP_INFO_STREAM_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Final waypoint reached according to x and y positions! The "
            "distance threshold is set as: "
                << final_waypoint_threshold_ << " Total " << pose_goals_.size()
                << " waypoints received");
      } else if (cur_cb_waypoint_idx_ <
                 static_cast<int>(pose_goals_.size()) - 1) {
        cur_cb_waypoint_idx_ = cur_cb_waypoint_idx_ + 1;
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "Intermidiate waypoint reached according to x and y positions, "
            "continue to the next waypoint, "
            "whose index is: "
                << cur_cb_waypoint_idx_
                << "The distance threshold is set as:" << waypoint_threshold_);
        pose_goal_ = pose_goals_[cur_cb_waypoint_idx_];
        TransformGlobalGoal();
      }
    }
  }

  // check for termination, evaluating trajectory and see if it's close to
  // pos_final
  if (last_traj_ != nullptr && finished_replanning_) {
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Waiting for traj termination");
    traj_opt::VecD pos_no_yaw = cmd_pos_;
    traj_opt::VecD pos_final, pos_finaln;
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    pos_final = state_machine::Make4d(pos_finaln);

    pos_no_yaw(2) = 0;
    pos_no_yaw(3) = 0;
    pos_final(2) = 0;
    pos_final(3) = 0;

    if ((pos_no_yaw - pos_final).norm() <= 1e-3) {
      auto success = std::make_shared<Replan::Result>();
      success->status = Replan::Result::SUCCESS;
      active_ = false;
      if (replan_goal_handle_ && replan_goal_handle_->is_active()) {
        replan_goal_handle_->succeed(success);
        replan_goal_handle_.reset();
      }
      for (int i = 0; i < 5; i++) {
        RCLCPP_INFO(this->get_logger(),
                    "Congratulations! Mission accomplished!!!");
      }
      last_traj_ = boost::shared_ptr<traj_opt::Trajectory>();
      return;
    }
  }

  if (replan_goal_handle_ && replan_goal_handle_->is_active()) {
    active_ = true;
  }
}

vec_Vec3f RePlanner::PathCropIntersect(const vec_Vec3f& path) {
  if (path.size() < 2) {
    RCLCPP_ERROR(this->get_logger(), "global path has <= 1 waypoints. Check!");
    return vec_Vec3f{};
  }

  float lower_x = local_map_ptr_->origin.x;
  float lower_y = local_map_ptr_->origin.y;
  float lower_z = local_map_ptr_->origin.z;

  int num_shrink_voxels = 1;
  float upper_x = lower_x + (local_map_ptr_->dim.x - num_shrink_voxels) *
                                local_map_ptr_->resolution;
  float upper_y = lower_y + (local_map_ptr_->dim.y - num_shrink_voxels) *
                                local_map_ptr_->resolution;
  float upper_z = lower_z + (local_map_ptr_->dim.z - num_shrink_voxels) *
                                local_map_ptr_->resolution;
  Vec3f map_lower = Vec3f{lower_x, lower_y, lower_z};
  Vec3f map_upper = Vec3f{upper_x, upper_y, upper_z};

  bool start_in_local_map =
      state_machine::CheckPointInBox(map_lower, map_upper, path[0]);
  if (!start_in_local_map) {
    RCLCPP_INFO(
        this->get_logger(),
        "Global path start is outside local voxel map. This can be (1) global "
        "replan rate is set to be low (if so, just ignore this message), (2) "
        "computation burden is high, (3) global map is too cluttered so that "
        "no feasible global path can be found after multiple trails!");
    RCLCPP_INFO(
        this->get_logger(),
        "The replanner still works, multiple intersections (between the global "
        "path and the local voxel map) may be found, and it will ignore the "
        "1st intersection and choose the 2nd intersection (if exists).");
  }

  vec_Vec3f cropped_path{};
  cropped_path.push_back(path[0]);

  Vec3f intersect_pt;
  bool intersected;
  bool is_first_intersection = true;
  for (unsigned int i = 1; i < path.size(); i++) {
    intersected = state_machine::IntersectLineBox(
        map_lower, map_upper, path[i - 1], path[i], &intersect_pt);
    if (intersected) {
      if (start_in_local_map) {
        cropped_path.push_back(intersect_pt);
        break;
      } else {
        if (is_first_intersection) {
          RCLCPP_INFO(this->get_logger(), "Ignoring the first intersection...");
          is_first_intersection = false;

          if (i == path.size() - 1) {
            cropped_path.push_back(path[i]);
            RCLCPP_INFO(
                this->get_logger(),
                "Global goal is inside local voxel map, directly using it as local "
                "goal!");
          }

        } else {
          cropped_path.push_back(intersect_pt);
          RCLCPP_INFO(
              this->get_logger(),
              "The second intersection is found, now using it as local goal!");
          break;
        }
      }
    } else {
      cropped_path.push_back(path[i]);
      if (i == path.size() - 1) {
        RCLCPP_INFO(
            this->get_logger(),
            "Global goal is inside local voxel map, directly using it as local "
            "goal!");
      }
    }
  }

  // publish for visualization
  kr_planning_msgs::msg::Path local_path_msg_ = kr::path_to_ros(cropped_path);
  local_path_msg_.header.frame_id = map_frame_;
  cropped_path_pub_->publish(local_path_msg_);

  if (cropped_path.size() < 2) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Cropped path only has 1 point, this happens only if the start is not "
        "in the voxel map AND no intersection is found between the "
        "global path and local voxel map!!");
    return vec_Vec3f{};
  }

  return cropped_path;
}

vec_Vec3f RePlanner::TransformGlobalPath(const vec_Vec3f& path_original) {
  geometry_msgs::msg::TransformStamped transformStamped;

  try {
    transformStamped = tfBuffer->lookupTransform(
        map_frame_, odom_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get tf from %s to %s",
                 odom_frame_.c_str(), map_frame_.c_str());
    return path_original;
  }

  geometry_msgs::msg::Pose map_to_odom;
  map_to_odom.position.x = transformStamped.transform.translation.x;
  map_to_odom.position.y = transformStamped.transform.translation.y;
  map_to_odom.position.z = transformStamped.transform.translation.z;
  map_to_odom.orientation.w = transformStamped.transform.rotation.w;
  map_to_odom.orientation.x = transformStamped.transform.rotation.x;
  map_to_odom.orientation.y = transformStamped.transform.rotation.y;
  map_to_odom.orientation.z = transformStamped.transform.rotation.z;

  auto map_to_odom_tf = kr::toTF(map_to_odom);
  Vec3f waypoint_wrt_map;

  vec_Vec3f path_wrt_map;
  for (unsigned int i = 0; i < path_original.size(); i++) {
    waypoint_wrt_map = map_to_odom_tf * path_original[i];
    path_wrt_map.push_back(waypoint_wrt_map);
  }

  // publish transformed global path for visualization
  kr_planning_msgs::msg::Path path_wrt_map_msg = kr::path_to_ros(path_wrt_map);
  path_wrt_map_msg.header.frame_id = map_frame_;
  global_path_wrt_map_pub_->publish(path_wrt_map_msg);
  return path_wrt_map;
}

void RePlanner::TransformGlobalGoal() {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer->lookupTransform(
        odom_frame_, map_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.01));
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get tf from %s to %s",
                 map_frame_.c_str(), odom_frame_.c_str());
    AbortReplan();
  }
  geometry_msgs::msg::PoseStamped pose_in;
  geometry_msgs::msg::PoseStamped pose_out;
  pose_in.pose = pose_goal_;
  tf2::doTransform(pose_in, pose_out, transformStamped);
  pose_goal_wrt_odom_ = pose_out.pose;

  if (std::abs(pose_goal_wrt_odom_.position.z - pose_goal_.position.z) >= 1) {
    RCLCPP_WARN(
        this->get_logger(),
        "When transforming global goal, the goal can be tranformed outside "
        "your voxel map boundaries, if the drift is significant and the "
        "waypoints are far away (especially for z axis boundaries).");
  }
}

vec_Vec3f RePlanner::PathCropDist(const vec_Vec3f& path,
                                  double crop_dist_xyz,
                                  double crop_dist_z) {
  if (path.size() < 2 || crop_dist_xyz < 0 || crop_dist_z < 0) {
    return path;
  }

  double d = crop_dist_xyz;
  double dz = crop_dist_z;

  vec_Vec3f cropped_path;
  Vec3f crop_end = path.back();

  double dist = 0;
  double dist_z = 0;

  for (unsigned int i = 1; i < path.size(); i++) {
    if (dist_z + abs(path[i][2] - path[i - 1][2]) > dz) {
      auto seg_normalized = (path[i] - path[i - 1]).normalized();
      double remaining_crop_dist = (dz - dist_z) / abs(seg_normalized[2]);

      double min_crop_dist = std::min(d - dist, remaining_crop_dist);
      crop_end =
          path[i - 1] + min_crop_dist * (path[i] - path[i - 1]).normalized();
      cropped_path.push_back(path[i - 1]);
      break;
    } else if (dist + (path[i] - path[i - 1]).norm() > d) {
      crop_end =
          path[i - 1] + (d - dist) * (path[i] - path[i - 1]).normalized();
      cropped_path.push_back(path[i - 1]);
      break;
    } else {
      dist += (path[i] - path[i - 1]).norm();
      dist_z += abs(path[i][2] - path[i - 1][2]);
      cropped_path.push_back(path[i - 1]);
    }
  }

  if ((cropped_path.back() - crop_end).norm() > 1e-1)
    cropped_path.push_back(crop_end);

  return cropped_path;
}

bool RePlanner::CloseToFinal(const vec_Vec3f& original_path,
                             const vec_Vec3f& cropped_path,
                             double dist_threshold) {
  if (original_path.size() < 2 || cropped_path.size() < 2) {
    return true;
  }

  Vec3f original_goal_pos = original_path.back();
  Vec3f cropped_goal_pos = cropped_path.back();

  if ((cropped_goal_pos - original_goal_pos).norm() < dist_threshold) {
    return true;
  } else {
    return false;
  }
}

void RePlanner::StateTriggerCb(
    const std_msgs::msg::String::ConstSharedPtr msg) {
  std::string requested_state = msg->data;

  bool abort_full_mission = false;
  if (requested_state == "reset_mission") {
    RCLCPP_WARN(this->get_logger(),
                "Reset mission button is clicked! Now resetting the mission!");
    abort_full_mission = true;
    cur_cb_waypoint_idx_ = 0;
    pose_goals_.clear();
  } else if (requested_state == "skip_next_waypoint") {
    RCLCPP_INFO(this->get_logger(), "Skip next waypoint button is clicked!");

    if (cur_cb_waypoint_idx_ >=
        static_cast<int>(pose_goals_.size()) - 1) {
      RCLCPP_ERROR(this->get_logger(),
                   "++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      RCLCPP_ERROR(this->get_logger(),
                   "++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      RCLCPP_ERROR(
          this->get_logger(),
          "The next waypoint is the final waypoint in the mission, aborting "
          "this full mission! \n To re-start, you have to either click CLEAR "
          "ALL and then publish a new mission, or re-click EXECUTE WAYPOINT "
          "MISSION (which will start over executing the existing mission)!");
      RCLCPP_ERROR(this->get_logger(),
                   "++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      RCLCPP_ERROR(this->get_logger(),
                   "++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      abort_full_mission = true;
      cur_cb_waypoint_idx_ = 0;
      pose_goals_.clear();
    } else {
      cur_cb_waypoint_idx_++;
      AbortReplan();
    }
  } else {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Wrong replanner state trigger information received, which is: "
            << requested_state
            << " , only allowed replanner state trigger msgs are "
               "skip_next_waypoint and reset_mission. Check!!!");
  }

  if (abort_full_mission) {
    AbortFullMission();
  }
}

void RePlanner::AbortFullMission(void) {
  auto abort = std::make_shared<Replan::Result>();
  abort->status = Replan::Result::ABORT_FULL_MISSION;
  active_ = false;
  if (replan_goal_handle_ && replan_goal_handle_->is_active()) {
    replan_goal_handle_->abort(abort);
    replan_goal_handle_.reset();
  }
  RCLCPP_WARN(
      this->get_logger(),
      "Now aborting mission and exiting replanner! \n If you want to restart "
      "with a new mission, click CLEAR ALL first in the RVIZ GUI to remove "
      "existing waypoints, before publishing new waypoints!");
}

void RePlanner::AbortReplan(void) {
  active_ = false;
  if (replan_goal_handle_ && replan_goal_handle_->is_active()) {
    auto res = std::make_shared<Replan::Result>(critical_);
    replan_goal_handle_->abort(res);
    replan_goal_handle_.reset();
  }
  RCLCPP_WARN(this->get_logger(), "Replanning terminated!");
}

RePlanner::RePlanner() : Node("replanner") {}

void RePlanner::init() {
  tfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  time_pub1 = this->create_publisher<sensor_msgs::msg::Temperature>(
      "/timing/replanner/global_replan", 1);
  time_pub2 = this->create_publisher<sensor_msgs::msg::Temperature>(
      "/timing/replanner/local_replan", 1);

  rclcpp::QoS latching_qos(1);
  latching_qos.transient_local();
  cropped_path_pub_ = this->create_publisher<kr_planning_msgs::msg::Path>(
      "cropped_local_path", latching_qos);
  global_path_wrt_map_pub_ = this->create_publisher<kr_planning_msgs::msg::Path>(
      "global_path_wrt_map", latching_qos);

  this->declare_parameter("local_global_server.max_horizon", 5);
  this->declare_parameter("local_global_server.crop_radius", 10.0);
  this->declare_parameter("local_global_server.crop_radius_z", 2.0);
  this->declare_parameter("local_global_server.close_to_final_dist", 10.0);
  this->declare_parameter("local_global_server.final_goal_reach_xy_threshold",
                          5.0);
  this->declare_parameter("local_global_server.waypoint_reach_xy_threshold",
                          10.0);
  this->declare_parameter("local_global_server.local_plan_timeout_duration",
                          1.0);
  this->declare_parameter("local_global_server.max_local_plan_trials", 3);
  this->declare_parameter("local_global_server.odom_frame",
                          std::string("odom"));
  this->declare_parameter("local_global_server.map_frame", std::string("map"));

  this->get_parameter("local_global_server.max_horizon", max_horizon_);
  this->get_parameter("local_global_server.crop_radius", crop_radius_);
  this->get_parameter("local_global_server.crop_radius_z", crop_radius_z_);
  this->get_parameter("local_global_server.close_to_final_dist",
                      close_to_final_dist_);
  double tmp_final, tmp_waypoint;
  this->get_parameter("local_global_server.final_goal_reach_xy_threshold",
                      tmp_final);
  this->get_parameter("local_global_server.waypoint_reach_xy_threshold",
                      tmp_waypoint);
  final_waypoint_threshold_ = static_cast<float>(tmp_final);
  waypoint_threshold_ = static_cast<float>(tmp_waypoint);
  this->get_parameter("local_global_server.local_plan_timeout_duration",
                      local_timeout_duration_);
  this->get_parameter("local_global_server.max_local_plan_trials",
                      max_local_trials_);
  this->get_parameter("local_global_server.odom_frame", odom_frame_);
  this->get_parameter("local_global_server.map_frame", map_frame_);

  // replan action server
  replan_server_ = rclcpp_action::create_server<Replan>(
      this,
      "replan",
      std::bind(&RePlanner::handleReplanGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&RePlanner::handleReplanCancel, this, std::placeholders::_1),
      std::bind(&RePlanner::handleReplanAccepted, this, std::placeholders::_1));

  global_plan_client_ =
      rclcpp_action::create_client<PlanTwoPoint>(this, "plan_global_path");
  local_plan_client_ =
      rclcpp_action::create_client<PlanTwoPoint>(this, "plan_local_trajectory");
  run_client_ =
      rclcpp_action::create_client<RunTrajectory>(this, "execute_trajectory");

  cmd_sub_ = this->create_subscription<kr_mav_msgs::msg::PositionCommand>(
      "position_cmd", 1,
      std::bind(&RePlanner::CmdCb, this, std::placeholders::_1));
  local_map_sub_ = this->create_subscription<kr_planning_msgs::msg::VoxelMap>(
      "local_voxel_map", 2,
      std::bind(&RePlanner::LocalMapCb, this, std::placeholders::_1));
  local_map_ptr_ = nullptr;

  epoch_sub_ = this->create_subscription<std_msgs::msg::Int64>(
      "epoch", 1,
      std::bind(&RePlanner::EpochCb, this, std::placeholders::_1));

  global_path_sub_ = this->create_subscription<kr_planning_msgs::msg::Path>(
      "global_path", 1,
      std::bind(&RePlanner::GlobalPathCb, this, std::placeholders::_1));

  state_trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
      "replan_state_trigger", 1,
      std::bind(&RePlanner::StateTriggerCb, this, std::placeholders::_1));

  // create critical bug report
  critical_.status = Replan::Result::CRITICAL_ERROR;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto replanner = std::make_shared<RePlanner>();
  replanner->init();
  rclcpp::Rate r(20);  // Should > max(local_replan_rate_, local_map_rate)
  while (rclcpp::ok()) {
    r.sleep();
    replanner->setup_replanner();
    replanner->update_status();
    rclcpp::spin_some(replanner);
  }
  rclcpp::shutdown();
  return 0;
}

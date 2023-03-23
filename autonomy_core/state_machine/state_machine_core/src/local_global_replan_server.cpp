#include <cmath>
#include <state_machine/local_global_replan_server.hpp>
/**
 * @brief Goal done callback function
 */
void RePlanner::GlobalPathCb(const kr_planning_msgs::Path& path) {
  global_path_.clear();
  global_path_ = kr::ros_to_path(
      path);  // extract the global path information
}

void RePlanner::EpochCb(const std_msgs::Int64& msg) {
  if (msg.data < 0) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_ERROR("aborting mission because tracker failed!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    AbortReplan();
    return;
  }

  boost::mutex::scoped_lock lock(mtx_);
  static int epoch_old = -1;  // keep a record of last epoch

  // replan ONLY IF current epoch is different than the previously recorded one
  if (epoch_old != msg.data) {
    int horizon;
    if (epoch_old == -1) {
      horizon = 1;
    } else {
      // msg.data = current_epoch_ + int(std::floor(duration/execution_time)),
      // where duration = (t_now - started_).toSec() horizon = 1 +
      // (current_plan_epoch - last_plan_epoch). Duration of one epoch is
      // execution_time, which is 1.0/local_replan_rate
      horizon = msg.data - last_plan_epoch_ + 1;
    }
    epoch_old = msg.data;  // keep a record of last epoch

    // trigger replan, set horizon
    if (!finished_replanning_) {
      if (PlanTrajectory(horizon))
        // execute the replanned trajectory
        RunTrajectory();
    }
  }
  update_status();
}

void RePlanner::CmdCb(const kr_mav_msgs::PositionCommand& cmd) {
  boost::mutex::scoped_lock lock(mtx_);
  cmd_pos_(0) = cmd.position.x;
  cmd_pos_(1) = cmd.position.y;
  cmd_pos_(2) = cmd.position.z;
  cmd_pos_(3) = cmd.yaw;
}

// map callback, update local_map_
void RePlanner::LocalMapCb(const kr_planning_msgs::VoxelMap::ConstPtr& msg) {
  if (map_counter_ == 0) {
    ROS_WARN("Got the first local voxel map!");
    // reset the local_map_last_timestamp_
    local_map_last_timestamp_ = ros::Time::now().toSec();
    total_map_time_ = 0;
  } else {
    double current_time = ros::Time::now().toSec();
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
        ROS_WARN_STREAM(
            "Local map rate unstable and sharply increased! Probably due to "
            "LIDAR packets "
            "loss or computation, check "
            "(1) LIDAR connection, and (2) computation headroom!");
        ROS_WARN_STREAM("Average update rate was: "
                        << avg_map_frequency_
                        << " Hz, most recent update rate is: "
                        << current_map_frequency << " Hz");
        // abort replan and trigger stopping policy
        // AbortReplan();
      } else if (current_map_frequency <
                 (1 - percent_tol) * avg_map_frequency_) {
        ROS_ERROR("Local map rate sharply dropped! Stopping policy triggered!");
        ROS_ERROR("Local map rate sharply dropped! Stopping policy triggered!");
        ROS_ERROR(
            "Probably due to LIDAR packets loss or computation, check "
            "(1) LIDAR connection, and (2) computation headroom!");
        ROS_ERROR_STREAM("Average update rate was: "
                         << avg_map_frequency_
                         << " Hz, most recent update rate is: "
                         << current_map_frequency << " Hz");
        // abort replan and trigger stopping policy
        AbortReplan();
      } else {
        ROS_INFO_STREAM_THROTTLE(
            1,
            "Local map updated rate is stable, most recent update frequency "
            "is: "
                << current_map_frequency << " Hz");
      }
    }
  }
  map_counter_++;
  local_map_ptr_ = msg;
}

void RePlanner::ReplanGoalCb() {
  boost::mutex::scoped_lock lock(mtx_);
  // accept new goal (ref:
  // http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a4964ef9e28f5620e87909c41f0458ecb)
  auto goal = replan_server_->acceptNewGoal();

  // set local planner local_replan_rate to be the same as specified in goal
  // (assigned in state machine)
  local_replan_rate_ = goal->local_replan_rate;

  // set global planner replan rate (global_replan_rate_factor >= 1)
  global_replan_rate_factor_ = goal->global_replan_rate_factor;

  if (goal->continue_mission) {
    // this means continuing with previous mission
    ROS_INFO("Continuing to finish waypoints in existing mission...");
    if (pose_goals_.empty()) {
      ROS_WARN("Existing waypoint list is empty, aborting replan!");
      ROS_WARN(
          "This can be caused by either clicking RESET MISSION, or clicking "
          "SKIP NEXT WAYPOINT and skipping all of your waypoints!");
      AbortFullMission();
      // AbortReplan();
    } else {
      // update the waypoint idx to continue executing the remaining waypoints
      // from the previous replan callback
      ROS_INFO("Last replan process has finished %d waypoints!",
               cur_cb_waypoint_idx_);
    }
  } else {
    // reset cur_cb_waypoint_idx
    cur_cb_waypoint_idx_ = 0;
    // this means starting a new mission
    ROS_INFO("Non-empty goal is sent, excecuting a new mission...");
    if ((goal->p_finals).empty()) {
      ROS_INFO("waypoints list is empty, now using single goal intead!");
      pose_goals_.push_back(goal->p_final);
    } else {
      pose_goals_ = goal->p_finals;
      ROS_INFO(
          "receive more than one waypoints! Number of waypoints: "
          "%zu",
          pose_goals_.size());
    }
    ROS_INFO_STREAM("new waypoints received, number of waypoints is:"
                    << pose_goals_.size());
  }

  pose_goal_ = pose_goals_[cur_cb_waypoint_idx_];
  TransformGlobalGoal();

  avoid_obstacle_ = goal->avoid_obstacles;  // obstacle avoidance mode
                                            // (assigned in state machine)

  // check cmd
  if (cmd_pos_.norm() == 0) {
    ROS_ERROR("RePlanner has not received position cmd, failing");
    AbortReplan();
    return;
  }

  if (!active_)        // if not active_, do setup again
    do_setup_ = true;  // only run setup_replanner function after ReplanGoalCb
}

void RePlanner::setup_replanner() {
  // check if local map is updated, if not, abort the mission
  // TODO(xu:) load this from param
  double local_map_min_rate = 0.5;
  double local_map_time_elapsed =
      ros::Time::now().toSec() - local_map_last_timestamp_;
  if ((map_counter_ >= 1) &&
      (local_map_time_elapsed) > 1.0 / local_map_min_rate) {
    ROS_ERROR_STREAM("Local map has not been updated for"
                     << local_map_time_elapsed
                     << " seconds! Stopping policy triggered!");
    ROS_ERROR(
        "Probably due to LIDAR packets loss or computation, check "
        "(1) LIDAR connection, and (2) computation headroom!");
    // abort replan and trigger stopping policy
    AbortReplan();
  }

  if (!do_setup_ || active_) {
    return;
  }
  if (local_map_ptr_ == nullptr) {
    ROS_WARN(
        "local_map_ptr_ is nullptr, local map not received "
        "yet!!!!!");
    return;
  }
  do_setup_ = false;  // only run setup_replanner once
  boost::mutex::scoped_lock lock(mtx_);

  ROS_INFO_STREAM("Setup replan");
  if (!avoid_obstacle_) {
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
    ROS_WARN("Obstacle avoidance is disabled!!!!!");
    ROS_WARN("+++++++++++++++++++++++++++++++++++");
  }

  if (waypoint_threshold_ < final_waypoint_threshold_) {
    ROS_WARN(
        "waypoint_reach_threshold is set as smaller than "
        "final_waypoint_reach_threshold, this is not recommanded, now "
        "changing the final_waypoint_threshold to be the same as "
        "waypoint_threshold!");
    final_waypoint_threshold_ = waypoint_threshold_;
  }

  // Initial plan step 1: global plan
  // ########################################################################################################
  // get the distance from cmd pos to goal (waypoint) pos
  float x_diff = (float)std::abs(pose_goal_wrt_odom_.position.x - cmd_pos_(0));
  float y_diff = (float)std::abs(pose_goal_wrt_odom_.position.y - cmd_pos_(1));
  Vec2f xy_diff = Vec2f{pose_goal_wrt_odom_.position.x - cmd_pos_(0),
                        pose_goal_wrt_odom_.position.y - cmd_pos_(1)};
  float dist_cmd_to_goal = xy_diff.norm();
  if (dist_cmd_to_goal <= waypoint_threshold_) {
    if ((cur_cb_waypoint_idx_ >= (pose_goals_.size() - 1)) &&
        (dist_cmd_to_goal <= final_waypoint_threshold_)) {
      // exit replanning process if the final waypoint is reached
      ROS_WARN(
          "Initial (and the only) waypoint x and y positions are already close "
          "to the robot, terminating the replanning process!");
      state_machine::ReplanResult success;
      success.status = state_machine::ReplanResult::SUCCESS;
      active_ = false;
      if (replan_server_->isActive()) {
        replan_server_->setSucceeded(success);
      }
      last_traj_ = boost::shared_ptr<traj_opt::Trajectory>();
      return;
    } else if (cur_cb_waypoint_idx_ < (pose_goals_.size() - 1)) {
      // take the next waypoint if the intermidiate waypoint is reached
      ROS_WARN(
          "Initial waypoint is already close to the robot "
          "position, continuing "
          "with the next waypoint!");
      cur_cb_waypoint_idx_ = cur_cb_waypoint_idx_ + 1;
      pose_goal_ = pose_goals_[cur_cb_waypoint_idx_];
      TransformGlobalGoal();
    }
  };

  // set goal
  kr_planning_msgs::PlanTwoPointGoal global_tpgoal;
  global_tpgoal.p_final = pose_goal_wrt_odom_;
  global_tpgoal.avoid_obstacles = avoid_obstacle_;

  // send goal to global plan action server
  global_plan_client_->sendGoal(
      global_tpgoal);  // only send goal, because global plan server is
                       // subscribing to odom and use that as start
  // global initial plan timeout duration
  double initial_global_timeout_dur = 6.0 * local_timeout_duration_;
  bool global_finished_before_timeout = global_plan_client_->waitForResult(
      ros::Duration(initial_global_timeout_dur));
  // check result of global plan
  if (!global_finished_before_timeout) {
    ROS_ERROR_STREAM(
        "initial global planning timed out, its timeout duration is set as: "
        << initial_global_timeout_dur);
    AbortReplan();
    return;
  }
  auto global_result = global_plan_client_->getResult();
  if (global_result->success) {
    global_path_ = kr::ros_to_path(
        global_result->path);  // extract the global path information
    ROS_WARN("initial global plan succeeded!");
  } else {
    ROS_WARN("initial global plan failed!");
    AbortReplan();
    return;
  }

  //  Initial plan step 2: Crop global path to get local goal
  //  #################################################################################
  // transform global path only for visualization
  vec_Vec3f global_path_wrt_map = TransformGlobalPath(global_path_);
  // local goal will be generated using intersection
  vec_Vec3f path_cropped_wrt_odom = PathCropIntersect(global_path_);

  if (path_cropped_wrt_odom.size() == 0) {
    ROS_ERROR("Path crop failed!");
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
  // ##########################################################################################################
  // set goal
  kr_planning_msgs::PlanTwoPointGoal local_tpgoal;
  state_machine::VecToPose(
      cmd_pos_, &local_tpgoal.p_init);  // use current position command as the
                                        // start position for local planner
  // initialize prev_start_pos_ for replan purpose
  prev_start_pos_(0) = cmd_pos_(0);
  prev_start_pos_(1) = cmd_pos_(1);
  prev_start_pos_(2) = cmd_pos_(2);

  // set vars
  local_tpgoal.epoch = 1;
  local_tpgoal.avoid_obstacles = avoid_obstacle_;
  local_tpgoal.execution_time = ros::Duration(1.0 / local_replan_rate_);
  // if close_to_final_goal, we need to check velocity tolerance as well
  local_tpgoal.check_vel = close_to_final_goal;
  // set p_final to be path_cropped_wrt_odom.back(), which is exactly at
  // accumulated distance d from the robot (unless path is shorter than d,
  // crop_end will be default as the end of path)
  Vec3f local_goal = path_cropped_wrt_odom.back();
  local_tpgoal.p_final.position.x = local_goal(0);
  local_tpgoal.p_final.position.y = local_goal(1);
  local_tpgoal.p_final.position.z = local_goal(2);

  // send goal to local trajectory plan action server
  local_plan_client_->sendGoal(local_tpgoal);

  // wait for result (initial timeout duration can be large because robot is not
  // moving)
  double initial_local_timeout_dur = local_timeout_duration_ * 2.0;
  bool local_finished_before_timeout = local_plan_client_->waitForResult(
      ros::Duration(initial_local_timeout_dur));

  // reset the counter
  // TODO(xu:) double check this is working properlly
  failed_local_trials_ = 0;

  if (!local_finished_before_timeout) {
    // check result of local plan
    ROS_ERROR_STREAM(
        "Initial local planning timed out, its timeout duration is set as: "
        << initial_local_timeout_dur);
    AbortReplan();
    return;
  }
  auto local_result = local_plan_client_->getResult();
  if (!local_result->success) {
    ROS_ERROR("Initial local planning failed to find a local trajectory!");
    AbortReplan();
    return;
  }

  // get the result
  last_traj_ = boost::make_shared<traj_opt::MsgTrajectory>(
      traj_opt::TrajDataFromSplineTrajectory(local_result->traj));
  last_plan_epoch_ = local_result->epoch;

  // setup state vars
  finished_replanning_ = false;

  // Initial plan step 4: execute the planned trajectory, during which
  // replanning process will also be triggered
  ROS_INFO("Started replanning!");
  RunTrajectory();
}

void RePlanner::RunTrajectory() {
  if (!replan_server_->isActive()) {
    return;
  }

  // set up run goal
  action_trackers::RunTrajectoryGoal rungoal;
  rungoal.traj =
      traj_opt::SplineTrajectoryFromTrajData(last_traj_->serialize());
  rungoal.epoch = last_plan_epoch_;
  rungoal.local_replan_rate = local_replan_rate_;

  // call the trajectory tracker
  run_client_->sendGoal(rungoal);
  // ROS_INFO("Sent Trajectory!");
  double tracker_timeout_dur = 0.5;
  bool tracker_finished_before_timeout =
      run_client_->waitForResult(ros::Duration(tracker_timeout_dur));
  if (!tracker_finished_before_timeout) {
    ROS_ERROR("Tracker aborted or timeout!");
    AbortReplan();
  }
}

bool RePlanner::PlanTrajectory(int horizon) {
  // check if local map is updated, if not, abort the mission
  // TODO(xu:) load this from param
  double local_map_min_rate = 0.5;
  double local_map_time_elapsed =
      ros::Time::now().toSec() - local_map_last_timestamp_;
  if ((map_counter_ >= 1) &&
      (local_map_time_elapsed) > 1.0 / local_map_min_rate) {
    ROS_ERROR_STREAM("Local map has not been updated for"
                     << local_map_time_elapsed
                     << " seconds! Stopping policy triggered!");
    ROS_ERROR(
        "Probably due to LIDAR packets loss or computation, check "
        "(1) LIDAR connection, and (2) computation headroom!");
    // abort replan and trigger stopping policy
    AbortReplan();
  }

  // horizon = 1 + (current_plan_epoch - last_plan_epoch),
  // where duration of one epoch is execution_time, i.e., 1.0/local_replan_rate
  if (horizon > max_horizon_) {
    ROS_ERROR(
        "Planning horizon is larger than max_horizon_, aborting the mission!");
    state_machine::ReplanResult abort;
    abort.status = state_machine::ReplanResult::DYNAMICALLY_INFEASIBLE;
    active_ = false;
    if (replan_server_->isActive()) {
      replan_server_->setAborted(abort);
    }
    return false;
  }

  // set global goal
  kr_planning_msgs::PlanTwoPointGoal global_tpgoal;

  // Important: for two reference frame system, we need to apply transform from
  // slam to odom so that the global goal in odom frame is adjusted to account
  // for the drift
  global_tpgoal.p_final = pose_goal_wrt_odom_;
  global_tpgoal.avoid_obstacles = avoid_obstacle_;

  // set local goal
  kr_planning_msgs::PlanTwoPointGoal local_tpgoal;
  double eval_time =
      double(horizon) /
      local_replan_rate_;  // calculate evaluation time, i.e., beginning of
                           // current plan epoch (in seconds)
  // make the end of last trajectory consistent with the start of current
  // trajectory
  state_machine::EvaluateToMsgs(last_traj_,
                                eval_time,
                                &local_tpgoal.p_init,
                                &local_tpgoal.v_init,
                                &local_tpgoal.a_init,
                                &local_tpgoal.j_init);
  Vec3f start_pos;
  start_pos = kr::pose_to_eigen(local_tpgoal.p_init);

  // Replan step 1: Global plan
  // ########################################################################################################
  // send goal to global plan server to replan (new goal will preempt old goals)
  if (global_replan_rate_factor_ > 1) {
    // calling global replan slower than calling local replan
    ROS_INFO_STREAM("local replan rate is " << global_replan_rate_factor_
                                            << "x global replan rate");
    if ((global_plan_counter_ % global_replan_rate_factor_) == 0) {
      global_plan_client_->sendGoal(global_tpgoal);
      // this is just for visualization purposes
      vec_Vec3f global_path_wrt_map = TransformGlobalPath(global_path_);
    }
    global_plan_counter_++;
  } else {
    // calling global replan at the same rate as calling local replan
    global_plan_client_->sendGoal(global_tpgoal);
    // this is just for visualization purposes
    vec_Vec3f global_path_wrt_map = TransformGlobalPath(global_path_);
  }

  prev_start_pos_ = start_pos;  // keep updating prev_start_pos_

  //  Replan step 2: Crop global path to get local goal
  //  #################################################################################

  // ROS_WARN_STREAM("++++ total_crop_dist = " << crop_dist);
  vec_Vec3f path_cropped_wrt_odom = PathCropIntersect(global_path_);
  if (path_cropped_wrt_odom.size() == 0) {
    ROS_ERROR("Path crop failed!");
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
  // ##########################################################################################################
  // set vars
  local_tpgoal.avoid_obstacles = avoid_obstacle_;
  local_tpgoal.execution_time = ros::Duration(1.0 / local_replan_rate_);
  local_tpgoal.epoch = last_plan_epoch_ + horizon;
  // if close_to_final_goal, we need to check velocity tolerance as well
  local_tpgoal.check_vel = close_to_final_goal;
  // change p_final to be path_cropped_wrt_odom.back(), which is exactly at
  // accumulated distance d from the robot (unless path is shorter than d,
  // crop_end will be default as the end of path)
  Vec3f local_goal = path_cropped_wrt_odom.back();
  local_tpgoal.p_final.position.x = local_goal(0);
  local_tpgoal.p_final.position.y = local_goal(1);
  local_tpgoal.p_final.position.z = local_goal(2);

  timer.start();
  // send goal to local trajectory plan action server
  local_plan_client_->sendGoal(local_tpgoal);

  // wait for result
  bool local_finished_before_timeout =
      local_plan_client_->waitForResult(ros::Duration(local_timeout_duration_));

  // timer stuff
  sensor_msgs::Temperature tmsg2;
  tmsg2.header.stamp = ros::Time::now();
  tmsg2.header.frame_id = map_frame_;
  // millisecond
  tmsg2.temperature = static_cast<double>(timer.elapsed().wall) / 1e6;
  // total_replan_time_ += tmsg2.temperature;
  // timer_counter_ += 1;
  // average_time_ = total_replan_time_ / timer_counter_;
  // ROS_WARN("[current local_planner_time]: %f", tmsg2.temperature);
  // ROS_WARN("[average local_planner_time]: %f", average_time_);
  // time_pub2.publish(tmsg2);

  // check result of local plan
  bool local_succeeded = true;
  if (!local_finished_before_timeout) {
    failed_local_trials_ += 1;
    ROS_WARN_STREAM(
        "Local planner timed out, trying to replan...  (local planner already "
        "failed "
        << failed_local_trials_
        << " times, total allowed trails: " << max_local_trials_ << ")");
    local_succeeded = false;
  } else {
    auto local_result = local_plan_client_->getResult();
    if (local_result->success) {
      last_traj_ = boost::make_shared<traj_opt::MsgTrajectory>(
          traj_opt::TrajDataFromSplineTrajectory(local_result->traj));
      last_plan_epoch_ = local_result->epoch;
      // ROS_INFO_STREAM("Got local plan with epoch " << last_plan_epoch_);
      failed_local_trials_ = 0;  // reset this
      return true;
    } else {
      failed_local_trials_ += 1;
      ROS_WARN_STREAM(
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

  // return true if local planner does not time out AND local_result is success
  return local_succeeded;
}

void RePlanner::update_status() {
  // check for termination
  if (last_traj_ != NULL) {
    traj_opt::VecD pos_goal = traj_opt::VecD::Zero(4, 1);
    traj_opt::VecD pos_final, pos_finaln;
    pos_goal(0) = pose_goal_wrt_odom_.position.x;
    pos_goal(1) = pose_goal_wrt_odom_.position.y;

    // evaluate traj at 1.0/local_replan_rate_, output recorded to pos_finaln,
    // refer to msg_traj.cpp.
    last_traj_->evaluate(1.0 / local_replan_rate_, 0, pos_finaln);

    pos_final = state_machine::Make4d(pos_finaln);
    pos_final(2) = 0;
    pos_final(3) = 0;

    // check if goal and traj evaluated position is less than threshold
    // get the distance from current pos to goal (waypoint) pos
    double dist_cmd_to_goal = (pos_goal - pos_final).norm();
    if (dist_cmd_to_goal <= waypoint_threshold_) {
      if ((cur_cb_waypoint_idx_ >= (pose_goals_.size() - 1)) &&
          (dist_cmd_to_goal <= final_waypoint_threshold_)) {
        // finished_replanning_ is set as true if this is the final waypoint
        finished_replanning_ = true;
        ROS_INFO_STREAM_THROTTLE(
            1.0,
            "Final waypoint reached according to x and y positions! The "
            "distance threshold is set as: "
                << final_waypoint_threshold_ << " Total " << pose_goals_.size()
                << " waypoints received");
      } else if (cur_cb_waypoint_idx_ < (pose_goals_.size() - 1)) {
        // take the next waypoint if the intermidiate waypoint is reached
        cur_cb_waypoint_idx_ = cur_cb_waypoint_idx_ + 1;
        ROS_INFO_STREAM(
            "Intermidiate waypoint reached according to x and y positions, "
            "continue to the next waypoint, "
            "whose index is: "
            << cur_cb_waypoint_idx_
            << "The distance threshold is set as:" << waypoint_threshold_);
        pose_goal_ = pose_goals_[cur_cb_waypoint_idx_];
        TransformGlobalGoal();

        /////////////////////////////////////////////////////////////////////
        // TODO(xu:) remove this after we are done with demo, have the quad
        // stop after reaching the waypoint for safety pilot to better react
        // ROS_WARN("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        // ROS_WARN("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        // ROS_WARN("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
        // ROS_WARN(
        //     "(REMOVE AFTER DEMO:) Waypoint reached! Now aborting replan and "
        //     "calling stopping policy before restart with the next
        //     waypoint!");
        // ROS_WARN(
        //     "(REMOVE AFTER DEMO:) Waypoint reached! Now aborting replan and "
        //     "calling stopping policy before restart with the next
        //     waypoint!");
        // AbortReplan();
        /////////////////////////////////////////////////////////////////////
      }
    };
  }

  // check for termination, evaluating trajectory and see if it's close to
  // pos_final
  if (last_traj_ != NULL && finished_replanning_) {
    ROS_INFO_STREAM_THROTTLE(1, "Waiting for traj termination");
    traj_opt::VecD pos_no_yaw = cmd_pos_;
    traj_opt::VecD pos_final, pos_finaln;
    // evaluate traj at last_traj_->getTotalTime(), output recorded to
    // pos_finaln,
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    last_traj_->evaluate(last_traj_->getTotalTime(), 0, pos_finaln);
    pos_final = state_machine::Make4d(pos_finaln);

    pos_no_yaw(2) = 0;
    pos_no_yaw(3) = 0;
    pos_final(2) = 0;
    pos_final(3) = 0;
    // ROS_INFO_STREAM_THROTTLE(
    //     1, "Termination error: " << (pos_no_yaw - pos_final).norm());

    // replan_server_ set as success if the commanded position and traj end
    // position is less than 1e-3
    if ((pos_no_yaw - pos_final).norm() <= 1e-3) {
      state_machine::ReplanResult success;
      success.status = state_machine::ReplanResult::SUCCESS;
      active_ = false;
      if (replan_server_->isActive()) {
        replan_server_->setSucceeded(success);
      }
      for (int i = 0; i < 5; i++) {
        ROS_INFO("Congratulations! Mission accomplished!!!");
      }
      last_traj_ = boost::shared_ptr<traj_opt::Trajectory>();
      return;
    }
  }

  // state_machine::ReplanResult in_progress;
  // in_progress.status = state_machine::ReplanResult::IN_PROGRESS;

  if (replan_server_->isActive()) {
    active_ = true;
    // replan_server_->setSucceeded(in_progress);
  }
}

vec_Vec3f RePlanner::PathCropIntersect(const vec_Vec3f& path) {
  if (path.size() < 2) {
    ROS_ERROR("global path has <= 1 waypoints. Check!");
    // return empty
    return vec_Vec3f{};
  }

  float lower_x = local_map_ptr_->origin.x;
  float lower_y = local_map_ptr_->origin.y;
  float lower_z = local_map_ptr_->origin.z;

  // shrink the local map boundary a bit to guarantee the goal is inside
  // boundary (insead of on the boundary)
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
    ROS_INFO(
        "Global path start is outside local voxel map. This can be (1) global "
        "replan rate is set to be low (if so, just ignore this message), (2) "
        "computation burden is high, (3) global map is too cluttered so that "
        "no feasible global path can be found after multiple trails!");
    ROS_INFO(
        "The replanner still works, multiple intersections (between the global "
        "path and the local voxel map) may be found, and it will ignore the "
        "1st intersection and choose the 2nd intersection (if exists).");
    // return empty
    // return vec_Vec3f{};
  }

  // the end of cropped path will be at the intersection between global path and
  // local map (if all segments of global path is inside the local map, it will
  // be default as the end of path)
  vec_Vec3f cropped_path{};
  // first, include the start of the global path
  cropped_path.push_back(path[0]);

  // add path segments until the intersection is found
  Vec3f intersect_pt;
  bool intersected;
  bool is_first_intersection = true;
  for (unsigned int i = 1; i < path.size(); i++) {
    intersected = state_machine::IntersectLineBox(
        map_lower, map_upper, path[i - 1], path[i], &intersect_pt);
    if (intersected) {
      if (start_in_local_map) {
        // intersects and start_in_local_map, add the intersection as the last
        // point on cropped path
        cropped_path.push_back(intersect_pt);
        break;
      } else {
        // intersects but start is not in local voxel map, ignore the 1st
        // intersection and use the 2nd intersection
        if (is_first_intersection) {
          ROS_INFO("Ignoring the first intersection...");
          is_first_intersection = false;
        } else {
          cropped_path.push_back(intersect_pt);
          ROS_INFO(
              "The second intersection is found, now using it as local goal!");
          break;
        }
      }
    } else {
      // does not intersect, add the end current segment to cropped path
      cropped_path.push_back(path[i]);
      if (i == path.size() - 1) {
        ROS_INFO(
            "Global goal is inside local voxel map, directly using it as local "
            "goal!");
      }
    }
  }

  // publish for visualization
  kr_planning_msgs::Path local_path_msg_ =
      kr::path_to_ros(cropped_path);
  local_path_msg_.header.frame_id = map_frame_;
  cropped_path_pub_.publish(local_path_msg_);

  // sanity check, cropped_path has to have at least 2 points, otherwise it
  // means that only the start of global path is included, which is not valid
  if (cropped_path.size() < 2) {
    ROS_ERROR(
        "Cropped path only has 1 point, this happens only if the start is not "
        "in the voxel map AND no intersection is found between the "
        "global path and local voxel map!!");
    // return empty
    return vec_Vec3f{};
  }

  // cropped path is valid and the last element will be used as local goal
  return cropped_path;
}

vec_Vec3f RePlanner::TransformGlobalPath(const vec_Vec3f& path_original) {
  // get the latest tf from map to odom reference frames
  geometry_msgs::TransformStamped transformStamped;

  try {
    transformStamped = tfBuffer.lookupTransform(
        map_frame_, odom_frame_, ros::Time(0), ros::Duration(0.01));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("Failed to get tf from %s to %s",
              odom_frame_.c_str(),
              map_frame_.c_str());
    // AbortReplan(); // no need to abort plan since this is just for
    // visualization return original path
    return path_original;
  }

  geometry_msgs::Pose map_to_odom;
  map_to_odom.position.x = transformStamped.transform.translation.x;
  map_to_odom.position.y = transformStamped.transform.translation.y;
  map_to_odom.position.z = transformStamped.transform.translation.z;
  map_to_odom.orientation.w = transformStamped.transform.rotation.w;
  map_to_odom.orientation.x = transformStamped.transform.rotation.x;
  map_to_odom.orientation.y = transformStamped.transform.rotation.y;
  map_to_odom.orientation.z = transformStamped.transform.rotation.z;

  // TF transform from the map frame to odom frame
  auto map_to_odom_tf = kr::toTF(map_to_odom);
  Vec3f waypoint_wrt_map;

  vec_Vec3f path_wrt_map;
  for (unsigned int i = 0; i < path_original.size(); i++) {
    // apply TF on current waypoint
    waypoint_wrt_map = map_to_odom_tf * path_original[i];
    path_wrt_map.push_back(waypoint_wrt_map);
  }

  // publish transformed global path for visualization
  kr_planning_msgs::Path path_wrt_map_msg =
      kr::path_to_ros(path_wrt_map);
  path_wrt_map_msg.header.frame_id = map_frame_;
  global_path_wrt_map_pub_.publish(path_wrt_map_msg);
  return path_wrt_map;
}

void RePlanner::TransformGlobalGoal() {
  // get the latest tf from map to odom reference frames
  geometry_msgs::TransformStamped transformStamped;
  try {
    // TF transform from odom frame to the map frame
    transformStamped = tfBuffer.lookupTransform(
        odom_frame_, map_frame_, ros::Time(0), ros::Duration(0.01));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("Failed to get tf from %s to %s",
              map_frame_.c_str(),
              odom_frame_.c_str());
    AbortReplan();
  }
  geometry_msgs::PoseStamped pose_in;
  geometry_msgs::PoseStamped pose_out;
  pose_in.pose = pose_goal_;
  tf2::doTransform(pose_in, pose_out, transformStamped);
  pose_goal_wrt_odom_ = pose_out.pose;

  // check if z-axis value is changed significantly, if yes, throw a warning
  if (std::abs(pose_goal_wrt_odom_.position.z - pose_goal_.position.z) >= 1) {
    ROS_WARN(
        "When transforming global goal, the goal can be tranformed outside "
        "your voxel map boundaries, if the drift is significant and the "
        "waypoints are far away (especially for z axis boundaries).");
  }

  // TODO(xu:) maybe we should replace this with some constraints to always keep
  // the goal position within the boundaries (clip its values)

  // when transforming global goal, keeping z-axis value the same to guaranteed
  // it's still within the voxel map

  // pose_goal_wrt_odom_.position.z = pose_goal_.position.z;
}

vec_Vec3f RePlanner::PathCropDist(const vec_Vec3f& path,
                                  double crop_dist_xyz,
                                  double crop_dist_z) {
  // return nonempty
  // precondition
  if (path.size() < 2 || crop_dist_xyz < 0 || crop_dist_z < 0) {
    return path;
  }

  double d = crop_dist_xyz;
  double dz = crop_dist_z;

  vec_Vec3f cropped_path;
  // crop_end will be exactly at accumulated distance d from the robot
  // (unless path is shorter than d crop_end will be default as the end of
  // path)
  Vec3f crop_end = path.back();

  double dist = 0;
  double dist_z = 0;

  // add path segments until the accumulated length is farther than distance d
  // from the robot
  for (unsigned int i = 1; i < path.size(); i++) {
    if (dist_z + abs(path[i][2] - path[i - 1][2]) > dz) {
      auto seg_normalized = (path[i] - path[i - 1]).normalized();
      double remaining_crop_dist = (dz - dist_z) / abs(seg_normalized[2]);
      // make sure don't crop more than d

      double min_crop_dist = std::min(d - dist, remaining_crop_dist);
      crop_end =
          path[i - 1] + min_crop_dist * (path[i] - path[i - 1]).normalized();
      cropped_path.push_back(path[i - 1]);
      break;
    } else if (dist + (path[i] - path[i - 1]).norm() > d) {
      // assign end to be exactly at accumulated distance d from the robot
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

  // path is shorter than d crop_end will be default as the end of path
  if ((cropped_path.back() - crop_end).norm() > 1e-1)
    cropped_path.push_back(crop_end);

  // post condition
  // CHECK(glog) cropped_path is not empty
  return cropped_path;
}

bool RePlanner::CloseToFinal(const vec_Vec3f& original_path,
                             const vec_Vec3f& cropped_path,
                             double dist_threshold) {
  // precondition: original_path and cropped_path are non-empty
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

void RePlanner::StateTriggerCb(const std_msgs::String::ConstPtr& msg) {
  std::string requested_state = msg->data;

  bool abort_full_mission = false;
  if (requested_state == "reset_mission") {
    ROS_WARN("Reset mission button is clicked! Now resetting the mission!");
    abort_full_mission = true;
    // reset waypoint index so that the user can send a new mission
    cur_cb_waypoint_idx_ = 0;
    pose_goals_.clear();
  } else if (requested_state == "skip_next_waypoint") {
    ROS_INFO("Skip next waypoint button is clicked!");

    // If current waypoint is the last waypoint in the mission, we will
    // abort the mission
    if (cur_cb_waypoint_idx_ >= (pose_goals_.size() - 1)) {
      ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      ROS_ERROR(
          "The next waypoint is the final waypoint in the mission, aborting "
          "this full mission! \n To re-start, you have to either click CLEAR "
          "ALL and then publish a new mission, or re-click EXECUTE WAYPOINT "
          "MISSION (which will start over executing the existing mission)!");
      ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      ROS_ERROR("++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      abort_full_mission = true;
      // reset waypoint index so that the user can send a new mission
      cur_cb_waypoint_idx_ = 0;
      pose_goals_.clear();
    } else {
      // Else, skip the next waypoint by adding 1 to cur_cb_waypoint_idx_

      // Note: If a new replan process is not started, it's probably due to
      // num_trials > max_replan_trials you set. You can just click
      // execute waypoint mission again to force re-start it...

      cur_cb_waypoint_idx_++;
      // to immediate make this in effect, we will abort current replan and have
      // the state machine re-enter the replan (after calling stopping policy)
      AbortReplan();
    }
  } else {
    ROS_ERROR_STREAM(
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
  // exit with abort full mission, will transit to hover
  state_machine::ReplanResult abort;
  abort.status = state_machine::ReplanResult::ABORT_FULL_MISSION;
  active_ = false;
  if (replan_server_->isActive()) {
    replan_server_->setAborted(abort);
  }
  ROS_WARN(
      "Now aborting mission and exiting replanner! \n If you want to restart "
      "with a new mission, click CLEAR ALL first in the RVIZ GUI to remove "
      "existing waypoints, before publishing new waypoints!");
}

void RePlanner::AbortReplan(void) {
  // exit with critical error, will transit to RetryWaypoints (i.e. will
  // re-enter this re-planner)
  active_ = false;
  if (replan_server_->isActive()) {
    replan_server_->setAborted(critical_);
  }
  ROS_WARN("Replanning terminated!");
}

RePlanner::RePlanner() : nh_("~") {
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  ros::NodeHandle priv_nh(nh_, "local_global_server");

  time_pub1 = priv_nh.advertise<sensor_msgs::Temperature>(
      "/timing/replanner/global_replan", 1);
  time_pub2 = priv_nh.advertise<sensor_msgs::Temperature>(
      "/timing/replanner/local_replan", 1);

  cropped_path_pub_ =
      priv_nh.advertise<kr_planning_msgs::Path>("cropped_local_path", 1, true);

  global_path_wrt_map_pub_ =
      priv_nh.advertise<kr_planning_msgs::Path>("global_path_wrt_map", 1, true);

  priv_nh.param("max_horizon", max_horizon_, 5);
  priv_nh.param("crop_radius", crop_radius_, 10.0);
  priv_nh.param("crop_radius_z", crop_radius_z_, 2.0);
  priv_nh.param("close_to_final_dist", close_to_final_dist_, 10.0);
  priv_nh.param(
      "final_goal_reach_xy_threshold", final_waypoint_threshold_, 5.0f);
  priv_nh.param("waypoint_reach_xy_threshold", waypoint_threshold_, 10.0f);
  priv_nh.param("local_plan_timeout_duration", local_timeout_duration_, 1.0);
  priv_nh.param("max_local_plan_trials", max_local_trials_, 3);
  priv_nh.param("odom_frame", odom_frame_, std::string("odom"));
  priv_nh.param("map_frame", map_frame_, std::string("map"));

  // replan action server
  replan_server_.reset(
      new actionlib::SimpleActionServer<state_machine::ReplanAction>(
          nh_, "replan", false));

  // plan global path action client, auto spin option set to true
  global_plan_client_.reset(
      new actionlib::SimpleActionClient<kr_planning_msgs::PlanTwoPointAction>(
          nh_, "plan_global_path", true));

  // plan local trajectory action client, auto spin option set to true
  local_plan_client_.reset(
      new actionlib::SimpleActionClient<kr_planning_msgs::PlanTwoPointAction>(
          nh_, "plan_local_trajectory", true));

  // run trajectory action client
  run_client_.reset(
      new actionlib::SimpleActionClient<action_trackers::RunTrajectoryAction>(
          nh_, "execute_trajectory", true));

  // subscriber of position command
  // command callback: setting cmd_pos_ to be the commanded position
  cmd_sub_ = nh_.subscribe("position_cmd", 1, &RePlanner::CmdCb, this);
  local_map_sub_ =
      nh_.subscribe("local_voxel_map", 2, &RePlanner::LocalMapCb, this);
  local_map_ptr_ = nullptr;

  // subscriber of epoch command, epoch is published by trajectory tracker
  // epoch callback: trigger replan, set horizon
  epoch_sub_ = nh_.subscribe("epoch", 1, &RePlanner::EpochCb, this);

  // subscriber of global path
  global_path_sub_ =
      nh_.subscribe("global_path", 1, &RePlanner::GlobalPathCb, this);

  // subscriber of state transition (skip waypoint, reset mission)
  state_trigger_sub_ = nh_.subscribe(
      "replan_state_trigger", 1, &RePlanner::StateTriggerCb, this);

  // create critical bug report
  critical_.status = state_machine::ReplanResult::CRITICAL_ERROR;

  // Goal callback
  replan_server_->registerGoalCallback(
      boost::bind(&RePlanner::ReplanGoalCb, this));

  replan_server_->start();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "replanner");
  ros::NodeHandle nh;
  RePlanner replanner;
  ros::Rate r(20);  // Should > max(local_replan_rate_, local_map_rate)
  while (nh.ok()) {
    r.sleep();
    replanner.setup_replanner();  // this function will only run AFTER the
                                  // ReplanGoalCb, and will only run ONCE
    replanner.update_status();
    ros::spinOnce();
  }
}

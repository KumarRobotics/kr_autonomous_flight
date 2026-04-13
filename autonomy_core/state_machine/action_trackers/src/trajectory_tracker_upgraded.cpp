#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// necessary changes after refactoring
#include <kr_mav_msgs/msg/position_command.hpp>
#include <kr_tracker_msgs/msg/tracker_status.hpp>

// quad control stuff
#include <kr_trackers_manager/Tracker.h>

// planning ros msgs stuff
#include <kr_planning_msgs/msg/spline_trajectory.hpp>

// traj_opt stuff
#include <action_trackers/traj_to_quad_cmd.h>
#include <std_msgs/msg/empty.hpp>
#include <traj_opt_ros/msg_traj.h>
#include <traj_opt_ros/ros_bridge.h>

// action stuff
#include <action_trackers/action/run_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// angles
#include <angles/angles.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <memory>
#include <string>

class ActionTrajectoryTracker : public kr_trackers_manager::Tracker {
 public:
  using RunTrajectory = action_trackers::action::RunTrajectory;
  using GoalHandleRunTrajectory =
      rclcpp_action::ServerGoalHandle<RunTrajectory>;

  // Initialize, Activate, Deactivate, update functions are all called by
  // trackers_manager.cpp Goal will have information about trajectory, epoch,
  // local and global replan rate, etc.

  /**
   * @brief initialize tracker
   */
  void Initialize(const rclcpp::Node::SharedPtr& parent_nh) override;

  /**
   * @brief activate tracker
   */
  bool Activate(
      const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr& cmd) override;

  /**
   * @brief deactivate tracker
   */
  void Deactivate(void) override;

  /**
   * @brief Update function.
   */
  kr_mav_msgs::msg::PositionCommand::ConstSharedPtr update(
      const nav_msgs::msg::Odometry::ConstSharedPtr& msg) override;

  /**
   * @brief Yaw alignment function.
   */
  void AlignYaw(const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg);

  uint8_t status() const override;

 private:
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const RunTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleRunTrajectory> goal_handle);
  void handleAccepted(
      const std::shared_ptr<GoalHandleRunTrajectory> goal_handle);

  rclcpp::Node::SharedPtr nh_;
  boost::shared_ptr<traj_opt::Trajectory> current_trajectory_;
  std::vector<boost::shared_ptr<traj_opt::Trajectory>> next_trajectory_;

  rclcpp::Time started_;
  // sequencing
  double execution_time{-1};
  int current_epoch_{0};
  std::vector<int> traj_epoch;

  bool active_;
  bool done_{false};
  // gains
  double kx_[3], kv_[3];
  double pos_err_max_;
  traj_opt::Vec3 K_lambda, Limits_lamda, state_lambda;
  kr_mav_msgs::msg::PositionCommand::SharedPtr cmd;

  kr_mav_msgs::msg::PositionCommand::SharedPtr init_cmd_;

  // action lib
  rclcpp_action::Server<RunTrajectory>::SharedPtr as_;
  std::shared_ptr<GoalHandleRunTrajectory> current_goal_handle_;
  bool prempted{false};
  double prempted_time;
  double yaw_thr_;
  bool use_lambda_;

  // align yaw or not
  bool align_yaw_;
  // align yaw with the direction of robot movement every yaw_align_dt seconds
  double last_yaw_ = 0.0;
  double align_time_passed_ = 0.0;
  rclcpp::Time prev_align_start_time_;  // current alignment start timestamp
  double yaw_dot_magnitude_;            // rad per second
  rclcpp::Time last_yaw_align_time_;
  bool yaw_alignment_initialized_ = false;
  bool alignment_ongoing_ = false;
  // calculated desired yaw direction every yaw_align_dt_ seconds
  double yaw_align_dt_ = 1.5;
  double last_yaw_align_x_ = 0.0;
  double last_yaw_align_y_ = 0.0;
  double yaw_des_ = 0.0;
  double ultimate_yaw_des_ = 0.0;
  double yaw_threshold_ =
      0.05;  // rad, if yaw error is larger than this, alignment will start
  double yaw_dot_des_ = 0.0;
  bool odom_yaw_recorded_ = false;

  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr epoch_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr lambda_pub_;

  static traj_opt::VecD make4d(const traj_opt::VecD& vec) {
    traj_opt::VecD out = traj_opt::VecD::Zero(4);
    long int rows = std::min(vec.rows(), static_cast<long int>(4));
    out.block(0, 0, rows, 1) = vec.block(0, 0, rows, 1);
    return out;
  }
};

void ActionTrajectoryTracker::Initialize(
    const rclcpp::Node::SharedPtr& parent_nh) {
  nh_ = parent_nh;
  nh_->declare_parameter("gains.pos.x", 2.5);
  nh_->declare_parameter("gains.pos.y", 2.5);
  nh_->declare_parameter("gains.pos.z", 5.0);
  nh_->declare_parameter("gains.vel.x", 2.2);
  nh_->declare_parameter("gains.vel.y", 2.2);
  nh_->declare_parameter("gains.vel.z", 4.0);
  nh_->get_parameter("gains.pos.x", kx_[0]);
  nh_->get_parameter("gains.pos.y", kx_[1]);
  nh_->get_parameter("gains.pos.z", kx_[2]);
  nh_->get_parameter("gains.vel.x", kv_[0]);
  nh_->get_parameter("gains.vel.y", kv_[1]);
  nh_->get_parameter("gains.vel.z", kv_[2]);

  nh_->declare_parameter("trajectory_tracker.max_pos_error", 1.0);
  nh_->declare_parameter("trajectory_tracker.yaw_thr", 3.14159);
  nh_->declare_parameter("trajectory_tracker.use_lambda", true);
  nh_->declare_parameter("trajectory_tracker.align_yaw", true);
  nh_->declare_parameter("trajectory_tracker.yaw_speed_magnitude", 0.5);
  nh_->get_parameter("trajectory_tracker.max_pos_error", pos_err_max_);
  nh_->get_parameter("trajectory_tracker.yaw_thr", yaw_thr_);
  nh_->get_parameter("trajectory_tracker.use_lambda", use_lambda_);
  nh_->get_parameter("trajectory_tracker.align_yaw", align_yaw_);
  nh_->get_parameter("trajectory_tracker.yaw_speed_magnitude",
                     yaw_dot_magnitude_);

  epoch_pub_ = nh_->create_publisher<std_msgs::msg::Int64>("epoch", 10);
  point_pub_ =
      nh_->create_publisher<geometry_msgs::msg::PointStamped>("roi", 10);
  lambda_pub_ =
      nh_->create_publisher<geometry_msgs::msg::Vector3>("lambda", 10);

  active_ = false;

  as_ = rclcpp_action::create_server<RunTrajectory>(
      nh_,
      "execute_trajectory",
      std::bind(&ActionTrajectoryTracker::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&ActionTrajectoryTracker::handleCancel, this,
                std::placeholders::_1),
      std::bind(&ActionTrajectoryTracker::handleAccepted, this,
                std::placeholders::_1));
}

bool ActionTrajectoryTracker::Activate(
    const kr_mav_msgs::msg::PositionCommand::ConstSharedPtr& msg) {
  // initialization
  started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  odom_yaw_recorded_ = false;
  yaw_alignment_initialized_ = false;
  alignment_ongoing_ = false;

  if (next_trajectory_.size() == 0) {
    if (msg == nullptr)
      return false;
    else {
      init_cmd_ = std::make_shared<kr_mav_msgs::msg::PositionCommand>(*msg);
      active_ = true;
      return true;
    }
  } else {
    active_ = true;
    if (msg != nullptr)
      init_cmd_ = std::make_shared<kr_mav_msgs::msg::PositionCommand>(*msg);
    return true;
  }
}

void ActionTrajectoryTracker::Deactivate(void) {
  active_ = false;
  init_cmd_ = nullptr;
  current_trajectory_ = boost::shared_ptr<traj_opt::Trajectory>();
  next_trajectory_.clear();
  traj_epoch.clear();
  current_epoch_ = 0;
  execution_time = -1;
  started_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  odom_yaw_recorded_ = false;
  yaw_alignment_initialized_ = false;
  alignment_ongoing_ = false;
}

kr_mav_msgs::msg::PositionCommand::ConstSharedPtr
ActionTrajectoryTracker::update(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  if (!active_) {
    return kr_mav_msgs::msg::PositionCommand::ConstSharedPtr();
  }

  // no current traj and next traj, return init_cmd_
  if (current_trajectory_ == nullptr && next_trajectory_.size() == 0) {
    init_cmd_->header.stamp = nh_->now();
    return init_cmd_;
  }

  const rclcpp::Time t_now = nh_->now();
  double duration = (t_now - started_).seconds();
  geometry_msgs::msg::Vector3 lambda_v;
  lambda_v.x = state_lambda(0);
  lambda_v.y = state_lambda(1);
  lambda_v.z = state_lambda(2);
  lambda_pub_->publish(lambda_v);

  // no current traj but have next traj, check continuity first
  if (current_trajectory_ == nullptr && next_trajectory_.size() > 0) {
    started_ = nh_->now();
    current_epoch_ = traj_epoch.front();
    duration = 0.0;
    // check for initial continuity
    traj_opt::VecD pos_old(4, 1), pos_new;
    pos_old << init_cmd_->position.x, init_cmd_->position.y,
        init_cmd_->position.z, init_cmd_->yaw;
    next_trajectory_.front()->evaluate(0.0, 0, pos_new);

    if (pos_new.rows() < 4) {
      pos_old.conservativeResize(pos_new.rows(), 1);
    } else if (pos_new.rows() == 9) {  // special sphere traj thing
      pos_old = pos_old.block(0, 0, 3, 1);
      pos_new = pos_new.block(0, 0, 3, 1);
    }

    if (pos_new.rows() >= 4) {
      // robust to wrapping by pi
      pos_old(3) = std::cos(pos_old(3));
      pos_new(3) = std::cos(pos_new(3));
    }

    // check continuity in position
    if ((pos_old - pos_new).norm() > 3.0) {
      // discontinuity is too large, abort
      RCLCPP_ERROR_STREAM(
          nh_->get_logger(),
          "Trajectories not continuous, aborting transition from "
              << pos_old.transpose() << " to " << pos_new.transpose());
      next_trajectory_.erase(next_trajectory_.begin());
      traj_epoch.erase(traj_epoch.begin());
      init_cmd_->header.stamp = nh_->now();
      if (next_trajectory_.size() == 0) done_ = true;

      // abort by publishing negative epoch so that replanner is aware
      std_msgs::msg::Int64 epoch_msg;
      epoch_msg.data = -1;
      epoch_pub_->publish(epoch_msg);
      return init_cmd_;
    } else {
      // discontinuity is within threshold, accept next_trajectory_ as new
      // trajectory
      current_trajectory_ = next_trajectory_.front();
      next_trajectory_.erase(next_trajectory_.begin());
      current_epoch_ = traj_epoch.front();
      traj_epoch.erase(traj_epoch.begin());
      state_lambda(0) = 0;
      RCLCPP_INFO_STREAM(nh_->get_logger(),
                         "Accepted new trajectory. Queue sizes: "
                             << next_trajectory_.size() << " and "
                             << traj_epoch.size());
    }
  }
  // both current traj and next traj exist
  else if (next_trajectory_.size() > 0) {
    int epoch_diff = traj_epoch.front() - current_epoch_;

    // check_time is the timestamp to stitch old traj and new traj together
    double check_time = execution_time * double(epoch_diff);
    if (duration >= check_time) {
      // only do something if trajectory is old
      traj_opt::VecD pos_old, pos_new;
      traj_opt::VecD vel_old, vel_new;

      // evaluate current traj at check_time
      current_trajectory_->evaluate(check_time, 0, pos_old);
      current_trajectory_->evaluate(check_time, 1, vel_old);

      // evaluate next traj at time=0
      next_trajectory_.front()->evaluate(0.0, 0, pos_new);
      next_trajectory_.front()->evaluate(0.0, 1, vel_new);

      if (pos_new.rows() < 4) {
        pos_old.conservativeResize(pos_new.rows(), 1);
        vel_old.conservativeResize(vel_new.rows(), 1);
      } else if (pos_new.rows() == 9) {  // special sphere traj thing
        pos_old = pos_old.block(0, 0, 3, 1);
        pos_new = pos_new.block(0, 0, 3, 1);
      }

      if (pos_new.rows() >= 4) {
        pos_old(3) = std::cos(pos_old(3));
        pos_new(3) = std::cos(pos_new(3));
      }

      // check for continuity
      if ((pos_old - pos_new).norm() > 1.0) {
        // discontinuity in position is too large, abort
        RCLCPP_ERROR_STREAM(nh_->get_logger(),
                            "Replan not continuous from " << pos_old.transpose()
                                                          << " to "
                                                          << pos_new.transpose());
        RCLCPP_ERROR_STREAM(
            nh_->get_logger(),
            "current epoch: " << current_epoch_
                              << " traj epoch: " << traj_epoch.front()
                              << " execution time: " << execution_time
                              << " check time " << check_time);
        next_trajectory_.erase(next_trajectory_.begin());
        traj_epoch.erase(traj_epoch.begin());
        // abort by publishing negative epoch so that replanner is aware
        std_msgs::msg::Int64 epoch_msg;
        epoch_msg.data = -1;
        epoch_pub_->publish(epoch_msg);

      } else if ((vel_old - vel_new).norm() > 1.0) {
        // discontinuity in velocity is too large, abort
        RCLCPP_ERROR_STREAM(nh_->get_logger(),
                            "Replan velocity not continuous from "
                                << vel_old.transpose() << " to "
                                << vel_new.transpose());
        RCLCPP_ERROR_STREAM(
            nh_->get_logger(),
            "current epoch: " << current_epoch_
                              << " traj epoch: " << traj_epoch.front()
                              << " execution time: " << execution_time);
        next_trajectory_.erase(next_trajectory_.begin());
        traj_epoch.erase(traj_epoch.begin());
        // abort by publishing negative epoch so that replanner is aware
        std_msgs::msg::Int64 epoch_msg;
        epoch_msg.data = -1;
        epoch_pub_->publish(epoch_msg);

      } else {
        // discontinuity in vel and position are both within threshold, accept
        // next_trajectory_ as new trajectory
        current_trajectory_ = next_trajectory_.front();
        next_trajectory_.erase(next_trajectory_.begin());
        current_epoch_ = traj_epoch.front();
        traj_epoch.erase(traj_epoch.begin());
        started_ = nh_->now();
        duration = 0.0;
        state_lambda(0) = 0;
      }
    }
  }

  // reset command
  cmd = std::make_shared<kr_mav_msgs::msg::PositionCommand>();
  cmd->header.stamp = t_now;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  // evaluate trajectory and get the cmd,  see traj_to_quad_cmd.h
  bool error_check_success = true;
  if (use_lambda_) {
    // evaluatePos will return false if difference in position between odometry
    // and trajectory is larger than pos_err_max_
    if (!traj_opt::EvaluateTrajectoryPos(current_trajectory_,
                                         msg,
                                         pos_err_max_,
                                         duration,
                                         0.01,
                                         cmd.get())) {
      RCLCPP_ERROR(
          nh_->get_logger(),
          "[Traj Tracker:] Max position error (between odom and trajectory) "
          "threshold violated (you need to slow down your execution or tune "
          "gains!)! Now aborting the mission!!!");
      RCLCPP_ERROR_STREAM(nh_->get_logger(),
                          "Max  error threshold is:" << pos_err_max_);
      error_check_success = false;
    }
  } else {
    // no pos or yaw check
    traj_opt::EvaluateTrajectory(current_trajectory_, duration, cmd.get(), 2);
  }

  if (align_yaw_) {
    AlignYaw(msg);
    cmd->yaw = yaw_des_, cmd->yaw_dot = yaw_dot_des_;
  } else {
    if (!odom_yaw_recorded_) {
      // get yaw from odom's quaternion
      tf2::Quaternion q(msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
      double roll, pitch;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_des_);
      odom_yaw_recorded_ = true;
    }
    cmd->yaw = yaw_des_, cmd->yaw_dot = 0.0;
  }

  init_cmd_ = cmd;

  // check if current trajectory is finished
  if (duration > current_trajectory_->getTotalTime()) {
    RCLCPP_INFO_STREAM(nh_->get_logger(),
                       "Finished trajectories at time "
                           << duration << " out of "
                           << current_trajectory_->getTotalTime());
    RCLCPP_INFO_STREAM(nh_->get_logger(),
                       "Null " << (next_trajectory_.size() == 0)
                               << " current epoch " << current_epoch_
                               << " traj epoch " << traj_epoch.front());
    init_cmd_->header.stamp = nh_->now();
    done_ = true;
    current_trajectory_ = boost::shared_ptr<traj_opt::Trajectory>();

    if (current_goal_handle_ && current_goal_handle_->is_active()) {
      auto result = std::make_shared<RunTrajectory::Result>();
      current_goal_handle_->succeed(result);
      current_goal_handle_.reset();
    }
  }

  // publish current epoch
  if (error_check_success) {
    std_msgs::msg::Int64 epoch_msg;
    epoch_msg.data = current_epoch_ +
                     static_cast<int>(std::floor(duration / execution_time));
    epoch_pub_->publish(epoch_msg);
  } else {
    // abort by publishing negative epoch so that replanner is aware
    std_msgs::msg::Int64 epoch_msg;
    epoch_msg.data = -1;
    epoch_pub_->publish(epoch_msg);
  }
  return cmd;
}

void ActionTrajectoryTracker::AlignYaw(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  // this part is for yaw_alignment
  double time_since_last_alignment;
  if (!yaw_alignment_initialized_) {
    yaw_alignment_initialized_ = true;
    // this means not initialized, initializing everything
    RCLCPP_INFO(nh_->get_logger(),
                "This is the very beginning of trajectory tracker, start yaw "
                "aligning!");
    last_yaw_align_time_ = nh_->now();
    time_since_last_alignment = 0.0;
    last_yaw_align_x_ = msg->pose.pose.position.x;
    last_yaw_align_y_ = msg->pose.pose.position.y;
    // get yaw from odom's quaternion
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, last_yaw_);
  } else {
    time_since_last_alignment = (nh_->now() - last_yaw_align_time_).seconds();
  }

  if (time_since_last_alignment > yaw_align_dt_) {
    // derive yaw alignment direction again
    last_yaw_align_time_ = nh_->now();
    double y_diff = msg->pose.pose.position.y - last_yaw_align_y_;
    double x_diff = msg->pose.pose.position.x - last_yaw_align_x_;
    // estimated velocity of the robot
    double est_velocity =
        std::sqrt(y_diff * y_diff + x_diff * x_diff) / yaw_align_dt_;
    // desired yaw direction should be arctan(dy, dx)
    double est_yaw_des = std::atan2(y_diff, x_diff);

    // if the robot is hovering (i.e. velocity is very small), skip alignment
    // (by not changing the value of ultimate_yaw_des_)
    if ((est_velocity) < 0.3) {
      RCLCPP_INFO_STREAM(nh_->get_logger(),
                         "[Yaw alignment:] Robot velocity is small, which is "
                             << est_velocity
                             << " m/s, skipping yaw alignment..");
    } else {
      // avoid oscillation when the robot moves backwards
      if (std::abs(est_yaw_des) > (3.14 * 0.9)) {
        // force the ultimate_yaw_des_ to be a positive number to avoid
        // oscillating between +pi and -pi
        ultimate_yaw_des_ = 3.14;
        RCLCPP_INFO_STREAM(nh_->get_logger(),
                           "[Yaw alignment:] Robot is moving backwards, and "
                           "yaw alignment is turned on...");
      } else {
        ultimate_yaw_des_ = est_yaw_des;
      }
    }

    // record x and y
    last_yaw_align_x_ = msg->pose.pose.position.x;
    last_yaw_align_y_ = msg->pose.pose.position.y;
  }

  if (std::abs(ultimate_yaw_des_ - last_yaw_) > yaw_threshold_) {
    // set yaw dot to move robot to align with desired yaw
    if (ultimate_yaw_des_ > last_yaw_) {
      yaw_dot_des_ = yaw_dot_magnitude_ * 1.0;
    } else {
      yaw_dot_des_ = yaw_dot_magnitude_ * -1.0;
    }
    // check if it's current alignment is already ongoing
    if (!alignment_ongoing_) {
      // no, start current alignment
      align_time_passed_ = 0.0;
      alignment_ongoing_ = true;
    } else {
      // yes, continue the alignment
      align_time_passed_ = (nh_->now() - prev_align_start_time_).seconds();
    }
    if (align_time_passed_ > 0.2) {
      RCLCPP_WARN_STREAM(nh_->get_logger(),
                         "alignment time passed is "
                             << align_time_passed_
                             << " seconds, which is large, this is not right, "
                                "setting it to 0.01! Go fix it!");
      align_time_passed_ = 0.01;  // just avoid aggressive movement
    }
    yaw_des_ = last_yaw_ + yaw_dot_des_ * align_time_passed_;
    // clip value to lie in -pi ~ +pi
    yaw_des_ = std::clamp(yaw_des_, -3.14, 3.14);

    // update the prev_align_start_time_
    prev_align_start_time_ = nh_->now();
    // update the prev_yaw_
    last_yaw_ = yaw_des_;
  } else {
    // maintaining the last yaw
    yaw_des_ = last_yaw_;
    yaw_dot_des_ = 0.0;
    alignment_ongoing_ = false;  // reset the align_time_passed_
  }
}

rclcpp_action::GoalResponse ActionTrajectoryTracker::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const RunTrajectory::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionTrajectoryTracker::handleCancel(
    const std::shared_ptr<GoalHandleRunTrajectory> goal_handle) {
  (void)goal_handle;
  prempted_time = (nh_->now() - started_).seconds();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionTrajectoryTracker::handleAccepted(
    const std::shared_ptr<GoalHandleRunTrajectory> goal_handle) {
  // equivalent to the old trajCB goal callback: triggered when a new goal
  // arrives.
  current_goal_handle_ = goal_handle;
  auto goal = goal_handle->get_goal();
  done_ = false;

  if (traj_epoch.size() > 0 && (traj_epoch.front() > goal->epoch)) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(),
                        "Traj epoch > goal epoch. Reset queue!");
    next_trajectory_.clear();
    traj_epoch.clear();
  }

  while (traj_epoch.size() > 0 && traj_epoch.back() >= goal->epoch) {
    next_trajectory_.pop_back();
    traj_epoch.pop_back();
  }
  // extract trajectory msg from goal->traj
  kr_planning_msgs::msg::SplineTrajectory msg = goal->traj;
  boost::shared_ptr<traj_opt::Trajectory> sp =
      boost::make_shared<traj_opt::MsgTrajectory>(
          traj_opt::TrajDataFromSplineTrajectory(msg));
  next_trajectory_.push_back(sp);
  prempted = false;
  if (goal->local_replan_rate > 0 && goal->epoch <= 0) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Replan rate and epoch must both be set! Rejecting Trajectory");
    return;
  }

  // change mode to either replanning or normal
  if (goal->local_replan_rate > 0) {
    execution_time = 1.0 / goal->local_replan_rate;
    // replanning mode will have non-zero traj_epoch (goal->epoch)
    traj_epoch.push_back(goal->epoch);
  } else {
    execution_time = -1;
    traj_epoch.push_back(0);
    current_epoch_ = 0;
  }

  // check for continuity
  traj_opt::TrajRosBridge::publish_msg(msg);

  auto result = std::make_shared<RunTrajectory::Result>();
  if (current_goal_handle_ && current_goal_handle_->is_active() &&
      !goal->block) {
    current_goal_handle_->succeed(result);
    current_goal_handle_.reset();
  }

  static uint prevSize = 0;
  if (next_trajectory_.size() > prevSize) {
    prevSize = next_trajectory_.size();
  }
}

uint8_t ActionTrajectoryTracker::status() const { return done_; }

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ActionTrajectoryTracker, kr_trackers_manager::Tracker)

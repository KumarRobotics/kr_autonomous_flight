#include <action_trackers/gain_params.hpp>
#include <action_trackers/action/take_off.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kr_tracker_msgs/msg/line_tracker_goal.hpp>
#include <kr_tracker_msgs/msg/tracker_status.hpp>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers_manager/Tracker.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <string>

/*
 * This controller is designed only to take the quad off the ground.
 * As a middle ground between the line trackers and the min-jerk takeoff,
 * the tracker ramps up the "thrust" (in m/s/s not Newtons) from thrust_min,
 * to thrust_max. One the quad is at max thrust, the tracker
 * switches to a min-jerk profile to the desired height.
 *
 * If the quad is not appove epsilon by the time it reaches max thrust, the
 * controller ramps the thurst down.
 *
 * This tracker is also desigend to idle the motors at thrust_min before the
 * take off action is called.
 *
 * Please note that the x-y position of the robot is defined at the time the z
 * position reaches epsilon!! So, this controller cannot take off to a specific
 * x-y location, but should be close.
 *
 */

using kr_mav_msgs::msg::PositionCommand;
using kr_tracker_msgs::msg::TrackerStatus;

class TakeOffTracker : public kr_trackers_manager::Tracker {
 public:
  using TakeOff = action_trackers::action::TakeOff;
  using GoalHandleTakeOff = rclcpp_action::ServerGoalHandle<TakeOff>;

  TakeOffTracker(void);

  void Initialize(const rclcpp::Node::SharedPtr& parent_nh) override;
  bool Activate(const PositionCommand::ConstSharedPtr& cmd) override;
  void Deactivate(void) override;

  PositionCommand::ConstSharedPtr update(
      const nav_msgs::msg::Odometry::ConstSharedPtr& msg) override;
  uint8_t status() const override;

 private:
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const TakeOff::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleTakeOff> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleTakeOff> goal_handle);

  void odomTocmd(const nav_msgs::msg::Odometry& msg, PositionCommand& cmd);
  void zeroGains(PositionCommand& cmd);
  void halfGains(PositionCommand& cmd);

  bool pos_set_, goal_set_, goal_reached_, ramp_up_done_, failed_, overshot_;
  bool params_ok;
  double epsilon_;
  double min_thrust_, max_thrust_, thrust_rate_;
  double take_off_h_, start_thrust_;
  double ground_z_, overshot_thrust_;
  bool active_;
  rclcpp::Time ramp_start_time_, traj_start_time_;

  Eigen::Vector3f start_pos_, goal_pos_, pos_;
  Eigen::Vector3f translation_dir_;
  float yaw_, start_yaw_;
  double kx_[3], kv_[3];

  // action lib
  rclcpp::Node::SharedPtr nh_;
  rclcpp_action::Server<TakeOff>::SharedPtr as_;
  std::shared_ptr<GoalHandleTakeOff> current_goal_handle_;
};

TakeOffTracker::TakeOffTracker(void)
    : pos_set_(false), goal_set_(false), goal_reached_(true), active_(false) {
  pos_ = Eigen::Vector3f::Zero();
  start_pos_ = Eigen::Vector3f::Zero();
  goal_pos_ = Eigen::Vector3f::Zero();
  translation_dir_ = Eigen::Vector3f::Zero();
  params_ok = true;
  goal_set_ = false;
  goal_reached_ = false;
  ramp_up_done_ = false;
  failed_ = false;
}
void TakeOffTracker::zeroGains(PositionCommand& cmd) {
  cmd.kx[0] = 0;
  cmd.kx[1] = 0;
  cmd.kx[2] = 0;
  cmd.kv[0] = 0;
  cmd.kv[1] = 0;
  cmd.kv[2] = 0;
}
void TakeOffTracker::halfGains(PositionCommand& cmd) {
  cmd.kx[0] *= 0.5;
  cmd.kx[1] *= 0.5;
  cmd.kx[2] *= 0.5;
}

void TakeOffTracker::Initialize(const rclcpp::Node::SharedPtr& parent_nh) {
  nh_ = parent_nh;
  action_trackers::declare_shared_gain_params(nh_);
  nh_->get_parameter("gains.pos.x", kx_[0]);
  nh_->get_parameter("gains.pos.y", kx_[1]);
  nh_->get_parameter("gains.pos.z", kx_[2]);
  nh_->get_parameter("gains.vel.x", kv_[0]);
  nh_->get_parameter("gains.vel.y", kv_[1]);
  nh_->get_parameter("gains.vel.z", kv_[2]);

  // see top of file, in m/s/s. This number should be negative! If it is 0,
  // the quad would be in hover.
  nh_->declare_parameter("take_off_tracker.min_thrust", -5.0);
  nh_->declare_parameter("take_off_tracker.max_thrust", 5.0);
  nh_->declare_parameter("take_off_tracker.thrust_rate", 10.0);
  nh_->declare_parameter("take_off_tracker.epsilon", 0.05);
  nh_->get_parameter("take_off_tracker.min_thrust", min_thrust_);
  nh_->get_parameter("take_off_tracker.max_thrust", max_thrust_);
  nh_->get_parameter("take_off_tracker.thrust_rate", thrust_rate_);
  nh_->get_parameter("take_off_tracker.epsilon", epsilon_);

  double min_takeoff = max_thrust_ * max_thrust_ * max_thrust_ / thrust_rate_ /
                           thrust_rate_ / 6.0 +
                       0.01;
  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "Minimum takeoff height should be " << min_takeoff);

  as_ = rclcpp_action::create_server<TakeOff>(
      nh_,
      "take_off",
      std::bind(&TakeOffTracker::handleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&TakeOffTracker::handleCancel, this, std::placeholders::_1),
      std::bind(&TakeOffTracker::handleAccepted, this, std::placeholders::_1));
}

bool TakeOffTracker::Activate(const PositionCommand::ConstSharedPtr& cmd) {
  RCLCPP_INFO_STREAM(nh_->get_logger(), "Activate with pos_set_ " << pos_set_);

  if (!params_ok) {
    RCLCPP_ERROR(nh_->get_logger(),
                 "Params have conflics!! Refusing to activate controller. "
                 "Please read take_off_tracker.cpp!");
    active_ = false;
  } else {
    active_ = true;
  }
  return active_;
}

void TakeOffTracker::Deactivate(void) {
  goal_set_ = false;
  goal_reached_ = false;
  ramp_up_done_ = false;
  active_ = false;
  failed_ = false;
}
// copies odom to position command
void TakeOffTracker::odomTocmd(const nav_msgs::msg::Odometry& msg,
                               PositionCommand& cmd) {
  cmd.position.x = msg.pose.pose.position.x;
  cmd.position.y = msg.pose.pose.position.y;
  cmd.position.z = msg.pose.pose.position.z;
  tf2::Quaternion q(
      msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  double roll_tmp, pitch_tmp, yaw_tmp;
  tf2::Matrix3x3(q).getRPY(roll_tmp, pitch_tmp, yaw_tmp);
  cmd.yaw = yaw_tmp;

  // damp velocities
  cmd.velocity.x = 0;
  cmd.velocity.y = 0;
  cmd.velocity.z = 0;
  cmd.yaw_dot = 0;

  // set these to be zero to be safe
  cmd.acceleration.x = 0;
  cmd.acceleration.y = 0;
  cmd.acceleration.z = 0;

  cmd.jerk.x = 0;
  cmd.jerk.y = 0;
  cmd.jerk.z = 0;
}

PositionCommand::ConstSharedPtr TakeOffTracker::update(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  tf2::Quaternion q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll_tmp, pitch_tmp, yaw_tmp;
  tf2::Matrix3x3(q).getRPY(roll_tmp, pitch_tmp, yaw_tmp);
  yaw_ = yaw_tmp;
  pos_set_ = true;

  if (!active_) return PositionCommand::ConstSharedPtr();

  auto cmd = std::make_shared<PositionCommand>();
  cmd->header.stamp = msg->header.stamp;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if (!goal_set_) {
    odomTocmd(*msg, *cmd);
    cmd->acceleration.z = min_thrust_;
    ground_z_ = cmd->position.z;
    ramp_start_time_ = rclcpp::Time(msg->header.stamp);
    zeroGains(*cmd);
    return cmd;
  }

  rclcpp::Time stamp(msg->header.stamp);

  // goal is set
  if (!ramp_up_done_) {
    odomTocmd(*msg, *cmd);
    halfGains(*cmd);
    double ramp_dt = (stamp - ramp_start_time_).seconds();
    double accel;
    if (!failed_)
      accel = std::min(min_thrust_ + ramp_dt * thrust_rate_, max_thrust_);
    else {
      // ramp down
      accel = std::max(
          max_thrust_ - (ramp_dt - max_thrust_ / thrust_rate_) * thrust_rate_,
          min_thrust_);
      if (accel == min_thrust_) {
        goal_set_ = false;
        failed_ = false;
        if (current_goal_handle_ && current_goal_handle_->is_active()) {
          auto res = std::make_shared<TakeOff::Result>();
          current_goal_handle_->abort(res);
          current_goal_handle_.reset();
        }
      }
    }
    cmd->acceleration.z = accel;
    if (accel == max_thrust_) {
      if (cmd->position.z - ground_z_ >= epsilon_) {
        RCLCPP_INFO_STREAM(nh_->get_logger(),
                           "Epsilon reached! Switching to min jerk trajectory");
        ramp_up_done_ = true;
        traj_start_time_ = stamp;
        start_thrust_ = accel;
        start_pos_ = pos_;
        start_yaw_ = yaw_;
      } else {
        RCLCPP_ERROR(nh_->get_logger(),
                     "Max thrust reached, but takeoff not detected");
        failed_ = true;
      }
    }
    if (cmd->position.z - ground_z_ > take_off_h_) {
      RCLCPP_WARN(nh_->get_logger(), "Overshot during take off, slowing down");
      ramp_up_done_ = true;
      overshot_ = true;
      cmd->position.z = take_off_h_ + ground_z_;
      // Set current pose as final
      ramp_start_time_ = stamp;
      start_pos_ = pos_;
      start_yaw_ = yaw_;
      overshot_thrust_ = cmd->acceleration.z;
    }

    return cmd;
  }

  if (overshot_) {
    double ramp_dt = (stamp - ramp_start_time_).seconds();
    double accel;
    if (overshot_thrust_ > 0)
      accel = std::max(overshot_thrust_ - ramp_dt * thrust_rate_, 0.0);
    else
      accel = std::min(overshot_thrust_ + ramp_dt * thrust_rate_, 0.0);
    cmd->acceleration.z = accel;

    cmd->position.z = take_off_h_ + ground_z_;
    cmd->position.x = start_pos_(0), cmd->position.y = start_pos_(1);
    cmd->velocity.x = 0.0, cmd->velocity.y = 0.0;
    cmd->acceleration.x = 0.0, cmd->acceleration.y = 0.0;
    cmd->yaw = start_yaw_, cmd->yaw_dot = 0.0;

    if (std::abs(accel) <= 1e-5) {
      goal_reached_ = true;
      RCLCPP_INFO_STREAM_THROTTLE(
          nh_->get_logger(), *nh_->get_clock(), 5000,
          "Take off complete! You are now free to switch trackers!");
      if (current_goal_handle_ && current_goal_handle_->is_active()) {
        auto res = std::make_shared<TakeOff::Result>();
        res->at_goal = true;
        current_goal_handle_->succeed(res);
        current_goal_handle_.reset();
      }
    }
    return cmd;
  }

  double a0 = start_thrust_;
  double pf = take_off_h_ + ground_z_ - start_pos_(2);

  double dt = (stamp - traj_start_time_).seconds();
  double T = std::sqrt(14 * pf / a0);

  if (dt > T) {
    goal_reached_ = true;
    RCLCPP_INFO_STREAM_THROTTLE(
        nh_->get_logger(), *nh_->get_clock(), 5000,
        "Take off complete! You are now free to switch trackers!");
    if (current_goal_handle_ && current_goal_handle_->is_active()) {
      auto res = std::make_shared<TakeOff::Result>();
      res->at_goal = true;
      current_goal_handle_->succeed(res);
      current_goal_handle_.reset();
    }
    dt = T;
  }

  // These were found using matlab
  cmd->position.z =
      start_pos_(2) + a0 * (dt * dt) * (1.0 / 2.0) -
      1.0 / (T * T * T * T * T * T * T) * (dt * dt * dt * dt * dt * dt * dt) *
          (pf * 2.0E1 - (T * T) * a0 * 2.0) +
      1.0 / (T * T * T * T) * (dt * dt * dt * dt) *
          (pf * 3.5E1 - (T * T) * a0 * 5.0) +
      1.0 / (T * T * T * T * T * T) * (dt * dt * dt * dt * dt * dt) *
          (pf * 7.0E1 - (T * T) * a0 * (1.5E1 / 2.0)) -
      1.0 / (T * T * T * T * T) * (dt * dt * dt * dt * dt) *
          (pf * 8.4E1 - (T * T) * a0 * 1.0E1);
  cmd->velocity.z = a0 * dt -
                    1.0 / (T * T * T * T * T * T * T) *
                        (dt * dt * dt * dt * dt * dt) *
                        (pf * 2.0E1 - (T * T) * a0 * 2.0) * 7.0 +
                    1.0 / (T * T * T * T) * (dt * dt * dt) *
                        (pf * 3.5E1 - (T * T) * a0 * 5.0) * 4.0 +
                    1.0 / (T * T * T * T * T * T) * (dt * dt * dt * dt * dt) *
                        (pf * 7.0E1 - (T * T) * a0 * (1.5E1 / 2.0)) * 6.0 -
                    1.0 / (T * T * T * T * T) * (dt * dt * dt * dt) *
                        (pf * 8.4E1 - (T * T) * a0 * 1.0E1) * 5.0;
  cmd->acceleration.z =
      a0 -
      1.0 / (T * T * T * T * T * T * T) * (dt * dt * dt * dt * dt) *
          (pf * 2.0E1 - (T * T) * a0 * 2.0) * 4.2E1 +
      1.0 / (T * T * T * T) * (dt * dt) * (pf * 3.5E1 - (T * T) * a0 * 5.0) *
          1.2E1 +
      1.0 / (T * T * T * T * T * T) * (dt * dt * dt * dt) *
          (pf * 7.0E1 - (T * T) * a0 * (1.5E1 / 2.0)) * 3.0E1 -
      1.0 / (T * T * T * T * T) * (dt * dt * dt) *
          (pf * 8.4E1 - (T * T) * a0 * 1.0E1) * 2.0E1;
  cmd->jerk.z =
      1.0 / (T * T * T * T) * dt * (pf * 3.5E1 - (T * T) * a0 * 5.0) * 2.4E1 -
      1.0 / (T * T * T * T * T * T * T) * (dt * dt * dt * dt) *
          (pf * 2.0E1 - (T * T) * a0 * 2.0) * 2.1E2 +
      1.0 / (T * T * T * T * T * T) * (dt * dt * dt) *
          (pf * 7.0E1 - (T * T) * a0 * (1.5E1 / 2.0)) * 1.2E2 -
      1.0 / (T * T * T * T * T) * (dt * dt) *
          (pf * 8.4E1 - (T * T) * a0 * 1.0E1) * 6.0E1;

  cmd->position.x = start_pos_(0), cmd->position.y = start_pos_(1);
  cmd->yaw = start_yaw_, cmd->yaw_dot = 0.0;

  if (current_goal_handle_ && current_goal_handle_->is_active()) {
    // publish feedback
    auto feedback = std::make_shared<TakeOff::Feedback>();
    feedback->time_remaining = 0;
    // feedback->time_remaining = T - dt;
    current_goal_handle_->publish_feedback(feedback);
  }

  return cmd;
}

rclcpp_action::GoalResponse TakeOffTracker::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TakeOff::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TakeOffTracker::handleCancel(
    const std::shared_ptr<GoalHandleTakeOff> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TakeOffTracker::handleAccepted(
    const std::shared_ptr<GoalHandleTakeOff> goal_handle) {
  // equivalent to ROS1 goal_callback: triggered when a new goal is accepted.
  current_goal_handle_ = goal_handle;
  take_off_h_ = goal_handle->get_goal()->height;

  if (goal_set_ || goal_reached_ || ramp_up_done_) {
    return;
  }

  start_pos_ = pos_;
  start_yaw_ = yaw_;
  goal_set_ = true;
  goal_reached_ = false;
  ramp_up_done_ = false;
  overshot_ = false;

  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "Accepted goal takeoff of " << take_off_h_);
}

uint8_t TakeOffTracker::status() const {
  return goal_reached_ ? TrackerStatus::SUCCEEDED : TrackerStatus::ACTIVE;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(TakeOffTracker, kr_trackers_manager::Tracker)

#include <action_trackers/TakeOffAction.h>
#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers/initial_conditions.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <cmath>

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

using kr_mav_msgs::PositionCommand;
using kr_tracker_msgs::TrackerStatus;

class TakeOffTracker : public kr_trackers_manager::Tracker {
 public:
  TakeOffTracker(void);

  void Initialize(const ros::NodeHandle& nh) override;
  bool Activate(const PositionCommand::ConstPtr& cmd) override;
  void Deactivate(void) override;

  PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr& msg) override;
  uint8_t status() const override;

 private:
  void goal_callback();
  void odomTocmd(const nav_msgs::Odometry& msg, PositionCommand& cmd);
  void zeroGains(PositionCommand& cmd);
  void halfGains(PositionCommand& cmd);

  ros::Subscriber sub_goal_;
  bool pos_set_, goal_set_, goal_reached_, ramp_up_done_, failed_, overshot_;
  bool params_ok;
  double epsilon_;
  double min_thrust_, max_thrust_, thrust_rate_;
  double take_off_h_, start_thrust_;
  double ground_z_, overshot_thrust_;
  bool active_;
  ros::Time ramp_start_time_, traj_start_time_;

  Eigen::Vector3f start_pos_, goal_pos_, pos_;
  Eigen::Vector3f translation_dir_;
  float yaw_, start_yaw_;
  double kx_[3], kv_[3];

  // action lib
  std::unique_ptr<actionlib::SimpleActionServer<action_trackers::TakeOffAction>>
      as_;
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

void TakeOffTracker::Initialize(const ros::NodeHandle& nh) {
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "take_off_tracker");

  priv_nh.param("min_thrust",
                min_thrust_,
                -5.0);  // see top of file, in m/s/s. This number should be
                        // negative! If it is 0, the quad would be in hover
  priv_nh.param("max_thrust", max_thrust_, 5.0);  // see top of file, in m/s/s
  priv_nh.param("thrust_rate",
                thrust_rate_,
                10.0);  // rate of change of thrust in ramp up (m/s/s/s)
  priv_nh.param("epsilon", epsilon_, 0.05);

  double min_takeoff = max_thrust_ * max_thrust_ * max_thrust_ / thrust_rate_ /
                           thrust_rate_ / 6.0 +
                       0.01;
  ROS_INFO_STREAM("Minimum takeoff height should be " << min_takeoff);

  as_.reset(new actionlib::SimpleActionServer<action_trackers::TakeOffAction>(
      nh, "take_off", false));
  // Register goal and preempt callbacks
  as_->registerGoalCallback(boost::bind(&TakeOffTracker::goal_callback, this));

  as_->start();
}

bool TakeOffTracker::Activate(const PositionCommand::ConstPtr& cmd) {
  ROS_INFO_STREAM("Activate with pos_set_ " << pos_set_);

  if (!params_ok) {
    ROS_ERROR(
        "Params have conflics!! Refusing to activate controller. Please read "
        "take_off_tracker.cpp!");
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
void TakeOffTracker::odomTocmd(const nav_msgs::Odometry& msg,
                               PositionCommand& cmd) {
  cmd.position.x = msg.pose.pose.position.x;
  cmd.position.y = msg.pose.pose.position.y;
  cmd.position.z = msg.pose.pose.position.z;
  cmd.yaw = tf::getYaw(msg.pose.pose.orientation);

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

PositionCommand::ConstPtr TakeOffTracker::update(
    const nav_msgs::Odometry::ConstPtr& msg) {
  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;

  if (!active_) return PositionCommand::Ptr();

  PositionCommand::Ptr cmd(new PositionCommand);
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
    ramp_start_time_ = msg->header.stamp;
    zeroGains(*cmd);
    return cmd;
  }

  // goal is set
  if (!ramp_up_done_) {
    odomTocmd(*msg, *cmd);
    halfGains(*cmd);
    double ramp_dt = (msg->header.stamp - ramp_start_time_).toSec();
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
        if (as_->isActive()) as_->setAborted();
      }
    }
    cmd->acceleration.z = accel;
    if (accel == max_thrust_) {
      if (cmd->position.z - ground_z_ >= epsilon_) {
        ROS_INFO_STREAM("Epsilon reached! Switching to min jerk trajectory");
        ramp_up_done_ = true;
        traj_start_time_ = msg->header.stamp;
        start_thrust_ = accel;
        start_pos_ = pos_;
        start_yaw_ = yaw_;
      } else {
        ROS_ERROR("Max thrust reached, but takeoff not detected");
        failed_ = true;
      }
    }
    if (cmd->position.z - ground_z_ > take_off_h_) {
      ROS_WARN("Overshot during take off, slowing down");
      ramp_up_done_ = true;
      overshot_ = true;
      cmd->position.z = take_off_h_ + ground_z_;
      // Set current pose as final
      ramp_start_time_ = msg->header.stamp;
      start_pos_ = pos_;
      start_yaw_ = yaw_;
      overshot_thrust_ = cmd->acceleration.z;
    }

    return cmd;
  }

  if (overshot_) {
    double ramp_dt = (msg->header.stamp - ramp_start_time_).toSec();
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
      ROS_INFO_STREAM_THROTTLE(
          1, "Take off complete! You are now free to swtich trackers!");
      if (as_->isActive()) {
        action_trackers::TakeOffResult res;
        res.at_goal = true;
        as_->setSucceeded(res);
      }
    }
    return cmd;
  }

  double a0 = start_thrust_;
  double pf = take_off_h_ + ground_z_ - start_pos_(2);

  double dt = (msg->header.stamp - traj_start_time_).toSec();
  double T = std::sqrt(14 * pf / a0);

  if (dt > T) {
    goal_reached_ = true;
    ROS_INFO_STREAM_THROTTLE(
        1, "Take off complete! You are now free to swtich trackers!");
    if (as_->isActive()) {
      action_trackers::TakeOffResult res;
      res.at_goal = true;
      as_->setSucceeded(res);
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

  if (as_->isActive()) {
    // publish feedback
    action_trackers::TakeOffFeedback feedback;
    feedback.time_remaining = 0;
    // feedback.time_remaining = T - dt;
    as_->publishFeedback(feedback);
  }

  return cmd;
}

void TakeOffTracker::goal_callback() {
  take_off_h_ = as_->acceptNewGoal()->height;

  if (goal_set_ || goal_reached_ || ramp_up_done_) {
    return;
  }

  start_pos_ = pos_;
  start_yaw_ = yaw_;
  goal_set_ = true;
  goal_reached_ = false;
  ramp_up_done_ = false;
  overshot_ = false;

  ROS_INFO_STREAM("Accepted goal takeoff of " << take_off_h_);
}

uint8_t TakeOffTracker::status() const {
  return goal_reached_ ? TrackerStatus::SUCCEEDED : TrackerStatus::ACTIVE;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TakeOffTracker, kr_trackers_manager::Tracker);

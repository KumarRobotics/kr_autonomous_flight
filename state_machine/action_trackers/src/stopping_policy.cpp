#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

using kr_mav_msgs::PositionCommand;
using kr_tracker_msgs::TrackerStatus;

class StoppingPolicy : public kr_trackers_manager::Tracker {
 public:
  StoppingPolicy() = default;
  void Initialize(const ros::NodeHandle &nh) override;
  bool Activate(const PositionCommand::ConstPtr &cmd) override;
  void Deactivate() override;

  PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg) override;
  uint8_t status() const override;

 private:
  bool active_{false};
  double j_des_, a_des_, a_yaw_des_;
  double prev_duration_;
  double kx_[3], kv_[3];
  // record the lastest cmd calculated in stopping policy
  Eigen::Vector4d cmd_pos_, cmd_vel_, cmd_acc_, cmd_jrk_;
  Eigen::VectorXd p0_, v0_, a0_, j0_, a0_dir_xyz_;
  double v0_dir_yaw_;
  ros::Time t0_;
};

void StoppingPolicy::Initialize(const ros::NodeHandle &nh) {
  ros::NodeHandle priv_nh(nh, "stopping_policy");

  priv_nh.param("acc_xyz_des", a_des_, 5.0);
  priv_nh.param("jerk_xyz_des", j_des_, 5.0);
  priv_nh.param("acc_yaw_des", a_yaw_des_, 0.1);
}

bool StoppingPolicy::Activate(const PositionCommand::ConstPtr &cmd) {
  if (cmd != NULL) {
    prev_duration_ = 0;
    p0_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(4, 1);
    v0_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(4, 1);
    a0_dir_xyz_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(3, 1);
    a0_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(4, 1);
    j0_ = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(4, 1);
    p0_ << cmd->position.x, cmd->position.y, cmd->position.z, cmd->yaw;
    cmd_pos_ = p0_;  // initialization of cmd_pos_
    v0_ << cmd->velocity.x, cmd->velocity.y, cmd->velocity.z, cmd->yaw_dot;
    cmd_vel_ = v0_;
    if (cmd->yaw_dot == 0) {
      v0_dir_yaw_ = 0.0;
    } else if (std::signbit(cmd->yaw_dot)) {
      v0_dir_yaw_ = -1.0;
    } else {
      v0_dir_yaw_ = 1.0;
    }
    a0_ << cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z, 0.0;
    cmd_acc_ = a0_;
    a0_dir_xyz_ << cmd->acceleration.x, cmd->acceleration.y,
        cmd->acceleration.z;
    a0_dir_xyz_.normalize();

    j0_ << cmd->jerk.x, cmd->jerk.y, cmd->jerk.z, 0.0;
    cmd_jrk_ = j0_;
    t0_ = cmd->header.stamp;
    active_ = true;
    return true;
  } else {
    ROS_ERROR("Need starting command to stop");
  }

  return false;
}

void StoppingPolicy::Deactivate() { active_ = false; }

PositionCommand::ConstPtr StoppingPolicy::update(
    const nav_msgs::Odometry::ConstPtr &msg) {
  ros::Time stamp = msg->header.stamp;
  double duration = (stamp - t0_).toSec();
  double dt = duration - prev_duration_;
  prev_duration_ = duration;

  if (!active_) return PositionCommand::Ptr();

  double a0_norm = Eigen::Vector3d{a0_(0), a0_(1), a0_(2)}.norm();
  double deacc_time;

  // check if there's initial acceleration
  if (a0_norm > 0.01) {
    deacc_time = a0_norm / j_des_;
  } else {
    deacc_time = 0.0;
  }

  double dur_remain;
  if (deacc_time != 0.0) {
    // phase 0: bring acceleration to 0 before decelerate
    // (not optimal, but negligible suboptimality)
    if (duration < deacc_time) {
      // evaluate xyz and their derivatives
      Eigen::Vector4d jrk_dir = Eigen::Vector4d{
          -a0_dir_xyz_(0), -a0_dir_xyz_(1), -a0_dir_xyz_(2), 0.0};
      Eigen::Vector4d new_cmd_jrk = jrk_dir * j_des_;

      // midpoint integration
      // get cmd at end of dt (note: dt happened right BEFORE current timestamp)
      Eigen::Vector4d new_cmd_acc = cmd_acc_ + cmd_jrk_ * dt;
      Eigen::Vector4d new_cmd_vel = cmd_vel_ + cmd_acc_ * dt;
      Eigen::Vector4d new_cmd_pos = cmd_pos_ + cmd_vel_ * dt;
      // use mean of the derivatives at beginning and end to do integration
      cmd_pos_ = cmd_pos_ + 0.5 * (new_cmd_vel + cmd_vel_) * dt;
      cmd_vel_ = cmd_vel_ + 0.5 * (new_cmd_acc + cmd_acc_) * dt;
      cmd_acc_ = cmd_acc_ + 0.5 * (new_cmd_jrk + cmd_jrk_) * dt;
      cmd_jrk_ = new_cmd_jrk;

      // update velocity direction, so that deceleration direction is
      // correct
      v0_ = cmd_vel_;
    }
    dur_remain = duration - deacc_time;
  } else {
    dur_remain = duration;
  }

  Eigen::Vector3d v0_dir_xyz = Eigen::Vector3d{v0_(0), v0_(1), v0_(2)};
  v0_dir_xyz.normalize();
  double v0_norm = Eigen::Vector3d{v0_(0), v0_(1), v0_(2)}.norm();
  // time needed to reach a_des_ using constant jerk
  double t_jerk = a_des_ / j_des_;
  // phase 1-3: increase |acceleration| -> const acc -> decrease |acc|
  double t_acc_change, t_acc_const;
  // calculate time duration for each phase
  // first check if acc will reach a_des_ i.e. (a_des_/2) * t_jerk <= v0_norm/2
  if (t_jerk * a_des_ <= v0_norm) {
    // case 1: will reach a_des_
    // phase 1 = phase 3 (increase and decrease |acceleration|)
    t_acc_change = t_jerk;
    // phase 2 (const acceleration)
    t_acc_const = (v0_norm - (t_jerk * a_des_)) / a_des_;
  } else {
    // case 2: will not reach a_des_
    // phase 1 = phase 3 = t, 0.5 * j * (t^2) = 0.5 * v => t = sqrt(v / j)
    t_acc_change = sqrt(v0_norm / j_des_);
    // phase 2 is 0
    t_acc_const = 0;
  }

  if ((0 <= dur_remain) && (dur_remain < t_acc_change * 2 + t_acc_const)) {
    // evaluate xyz and their derivatives
    Eigen::Vector4d new_cmd_jrk;
    // update jerk according to which phase it is now
    if (dur_remain < t_acc_change) {
      // phase 1: increase |acceleration|
      Eigen::Vector4d jrk_dir =
          Eigen::Vector4d{-v0_dir_xyz(0), -v0_dir_xyz(1), -v0_dir_xyz(2), 0.0};
      new_cmd_jrk = jrk_dir * j_des_;
    } else if (t_acc_change <= dur_remain &&
               dur_remain < t_acc_change + t_acc_const) {
      // phase 2: const acceleration
      new_cmd_jrk = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(4, 1);
    } else if (dur_remain >= t_acc_change + t_acc_const) {
      // phase 3: decrease |acceleration|
      Eigen::Vector4d jrk_dir =
          Eigen::Vector4d{v0_dir_xyz(0), v0_dir_xyz(1), v0_dir_xyz(2), 0.0};
      new_cmd_jrk = jrk_dir * j_des_;
    }
    // midpoint integration
    // get cmd at end of dt (note: dt happened right BEFORE current timestamp)
    Eigen::Vector4d new_cmd_acc = cmd_acc_ + cmd_jrk_ * dt;
    Eigen::Vector4d new_cmd_vel = cmd_vel_ + cmd_acc_ * dt;
    Eigen::Vector4d new_cmd_pos = cmd_pos_ + cmd_vel_ * dt;
    // use mean of the derivatives at beginning and end to do integration
    cmd_pos_ = cmd_pos_ + 0.5 * (new_cmd_vel + cmd_vel_) * dt;
    cmd_vel_ = cmd_vel_ + 0.5 * (new_cmd_acc + cmd_acc_) * dt;
    cmd_acc_ = cmd_acc_ + 0.5 * (new_cmd_jrk + cmd_jrk_) * dt;
    cmd_jrk_ = new_cmd_jrk;

  } else if (dur_remain >= t_acc_change * 2 + t_acc_const) {
    // stopping policy for XYZ finished, cmd_pos_(i) will remain the same
    for (int i = 0; i < 3; i++) {
      cmd_jrk_(i) = 0;
      cmd_acc_(i) = 0;
      cmd_vel_(i) = 0;
    }
  }

  double t_yaw = v0_(3) / a_yaw_des_;  // use consant acc for yaw
  // check whether already stopped (yaw)
  if (duration < t_yaw) {
    // evaluate yaw and yaw_dot
    cmd_vel_(3) = v0_(3) - v0_dir_yaw_ * a_yaw_des_;
    cmd_pos_(3) = p0_(3) + ((v0_(3) + cmd_vel_(3)) / 2.0) * duration;
  } else {
    // stopping policy for yaw finished, cmd_pos_(i) will remain the same
    cmd_vel_(3) = 0.0;
  }

  if ((dur_remain >= t_acc_change * 2 + t_acc_const) && (duration >= t_yaw))
    ROS_INFO_THROTTLE(1.0, "Stopping policy done!");

  PositionCommand::Ptr cmd(new PositionCommand);
  cmd->header.stamp = stamp;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  cmd->position.x = cmd_pos_(0), cmd->position.y = cmd_pos_(1),
  cmd->position.z = cmd_pos_(2);
  cmd->velocity.x = cmd_vel_(0), cmd->velocity.y = cmd_vel_(1),
  cmd->velocity.z = cmd_vel_(2);
  cmd->acceleration.x = cmd_acc_(0), cmd->acceleration.y = cmd_acc_(1),
  cmd->acceleration.z = cmd_acc_(2);
  cmd->jerk.x = cmd_jrk_(0), cmd->jerk.y = cmd_jrk_(1),
  cmd->jerk.z = cmd_jrk_(2);
  cmd->yaw = cmd_pos_(3), cmd->yaw_dot = cmd_vel_(3);

  return cmd;
}

uint8_t StoppingPolicy::status() const {
  return active_ ? TrackerStatus::ACTIVE : TrackerStatus::SUCCEEDED;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(StoppingPolicy, kr_trackers_manager::Tracker);

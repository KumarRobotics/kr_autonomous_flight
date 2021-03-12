#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <traj_opt_basic/types.h>
#include <traj_opt_pro/nonlinear_trajectory.h>
#include <traj_opt_ros/ros_bridge.h>

using kr_mav_msgs::PositionCommand;
using kr_tracker_msgs::TrackerStatus;

class StoppingPolicy : public kr_trackers_manager::Tracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StoppingPolicy();

  void Initialize(const ros::NodeHandle &nh) override;
  bool Activate(const PositionCommand::ConstPtr &cmd) override;
  void Deactivate() override;

  PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg) override;
  uint8_t status() const override;

 private:
  double j_des_, a_des_, a_yaw_des_;
  double prev_duration_;
  bool active_;
  double kx_[3], kv_[3];
  traj_opt::Vec4 cmd_pos_, cmd_vel_, cmd_acc_,
      cmd_jrk_;  // record the lastest cmd calculated in stopping policy
  traj_opt::VecD p0_, v0_, a0_, j0_, v0_dir_xyz_, a0_dir_xyz_;
  double v0_dir_yaw_;
  ros::Time t0_;
  boost::shared_ptr<traj_opt::NonlinearTrajectory> solver_;
};

StoppingPolicy::StoppingPolicy(void) : active_(false) {}

void StoppingPolicy::Initialize(const ros::NodeHandle &nh) {
  ros::NodeHandle priv_nh(nh, "stopping_policy");

  priv_nh.param("acc_xyz_des", a_des_, 5.0);
  priv_nh.param("jerk_xyz_des", j_des_, 2.0);
  priv_nh.param("acc_yaw_des", a_yaw_des_, 0.1);
}

bool StoppingPolicy::Activate(const PositionCommand::ConstPtr &cmd) {
  if (cmd != NULL) {
    prev_duration_ = 0;
    p0_ = traj_opt::VecD::Zero(4, 1);
    v0_ = traj_opt::VecD::Zero(4, 1);
    v0_dir_xyz_ = traj_opt::VecD::Zero(3, 1);
    a0_dir_xyz_ = traj_opt::VecD::Zero(3, 1);
    a0_ = traj_opt::VecD::Zero(4, 1);
    j0_ = traj_opt::VecD::Zero(4, 1);
    p0_ << cmd->position.x, cmd->position.y, cmd->position.z, cmd->yaw;
    cmd_pos_ = p0_;  // initialization of cmd_pos_
    v0_ << cmd->velocity.x, cmd->velocity.y, cmd->velocity.z, cmd->yaw_dot;
    cmd_vel_ = v0_;
    v0_dir_xyz_ << cmd->velocity.x, cmd->velocity.y, cmd->velocity.z;
    v0_dir_xyz_.normalize();
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
  } else
    ROS_ERROR("Need starting command to stop");

  return false;
}

void StoppingPolicy::Deactivate(void) { active_ = false; }

PositionCommand::ConstPtr StoppingPolicy::update(
    const nav_msgs::Odometry::ConstPtr &msg) {
  ros::Time stamp = msg->header.stamp;
  double duration = (stamp - t0_).toSec();
  double dt = duration - prev_duration_;
  prev_duration_ = duration;

  if (!active_) return PositionCommand::Ptr();

  double a0_norm = traj_opt::Vec3{a0_(0), a0_(1), a0_(2)}.norm();
  double deacc_time = 0.0;
  if (a0_norm > 0.1) {
    deacc_time = a0_norm / j_des_;
  }

  double v0_norm = traj_opt::Vec3{v0_(0), v0_(1), v0_(2)}.norm();
  // double t_xyz = traj_opt::Vec3{v0_(0), v0_(1), v0_(2)}.norm() / a_des_;
  // time for acceleration to build up to a_des_
  double t_jerk = a_des_ / j_des_;

  // check if acceleration will reach a_des_ based on j_des_ and v0
  double t_acc_change, t_acc_const;
  if (t_jerk * a_des_ <= v0_norm) {
    // start and end period where acceleration increases and decreases
    t_acc_change = t_jerk;
    // middle period with constant acceleration a_des_
    t_acc_const = (v0_norm - (t_jerk * a_des_)) / a_des_;
  } else {
    // 0.5 * j * (t^2) = 0.5 * v ==> t = sqrt(v / j)
    t_acc_change = sqrt(v0_norm / j_des_);
    // middle period is 0
    t_acc_const = 0;
  }

  if (deacc_time != 0) {
    if (duration < deacc_time) {
      for (int i = 0; i < 3; i++) {
        // evaluate xyz and their derivatives
        cmd_pos_(i) = cmd_pos_(i) + cmd_vel_(i) * dt;
        cmd_vel_(i) = cmd_vel_(i) + cmd_acc_(i) * dt;
        cmd_acc_(i) =
            cmd_acc_(i) + cmd_jrk_(i) * dt;  // build up from 0 inital acc
      }
      // duration < deacc_time, first bring existing acc to zero
      cmd_jrk_ = -a0_dir_xyz_ * j_des_;
    }
    v0_dir_xyz_ = cmd_vel_;
  }

  double dur_remain = duration - deacc_time;

  if ((0 = < dur_remain) && (dur_remain < t_acc_change * 2 + t_acc_const)) {
    for (int i = 0; i < 3; i++) {
      // evaluate xyz and their derivatives
      cmd_pos_(i) = cmd_pos_(i) + cmd_vel_(i) * dt;
      cmd_vel_(i) = cmd_vel_(i) + cmd_acc_(i) * dt;
      cmd_acc_(i) =
          cmd_acc_(i) + cmd_jrk_(i) * dt;  // build up from 0 inital acc
    }

    // only when dur_remain >= 0
    if (0 <= dur_remain && dur_remain < t_acc_change) {
      cmd_jrk_ = -v0_dir_xyz_ * j_des_;
    } else if (t_acc_change <= dur_remain &&
               dur_remain < t_acc_change + t_acc_const) {
      cmd_jrk_ = traj_opt::VecD::Zero(4, 1);
    } else if (dur_remain >= t_acc_change + t_acc_const) {
      cmd_jrk_ = v0_dir_xyz_ * j_des_;
    }
  } else if (dur_remain >= t_acc_change * 2 + t_acc_const) {
    // stage 4: stopping policy finished, cmd_pos_(i) will remain the same
    for (int i = 0; i < 3; i++) {
      cmd_jrk_(i) = 0;
      cmd_acc_(i) = 0;
      cmd_vel_(i) = 0;
    }
  }

  double t_yaw = v0_(3) / a_des_;  // use consant acc for yaw
  // check whether already stopped (yaw)
  if (duration < t_yaw) {
    // evaluate yaw and yaw_dot
    cmd_vel_(3) = v0_(3) - v0_dir_yaw_ * a_yaw_des_;
    cmd_pos_(3) = p0_(3) + ((v0_(3) + cmd_vel_(3)) / 2.0) * duration;
  } else {
    cmd_vel_(3) = 0.0;
    // cmd_pos_(3) remain the same
  }

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
#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>
#include <cmath>

using kr_mav_msgs::PositionCommand;
using kr_tracker_msgs::TrackerStatus;

/*
 * Constant jerk stopping policy
 *
 */
class StoppingPolicy : public kr_trackers_manager::Tracker {
 public:
  StoppingPolicy() = default;
  void Initialize(const ros::NodeHandle& nh) override;
  bool Activate(const PositionCommand::ConstPtr& cmd) override;
  void Deactivate() override;

  PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr& msg) override;
  uint8_t status() const override;

 private:
  bool active_{false};
  bool odom_first_call_{true};
  double j_xy_des_, a_xy_des_, j_z_des_, a_z_des_, a_yaw_des_;
  double prev_duration_;
  // record the lastest cmd calculated in stopping policy
  Eigen::Vector4d cmd_pos_, cmd_vel_, cmd_acc_, cmd_jrk_;
  Eigen::VectorXd p0_, v0_, a0_, j0_, a0_dir_xyz_;
  double v0_dir_yaw_;
  ros::Time t0_;
};

void StoppingPolicy::Initialize(const ros::NodeHandle& nh) {
  ros::NodeHandle priv_nh(nh, "stopping_policy");

  priv_nh.param("acc_xy_des", a_xy_des_, 10.0);
  priv_nh.param("jerk_xy_des", j_xy_des_, 10.0);
  priv_nh.param("acc_z_des", a_z_des_, 5.0);
  priv_nh.param("jerk_z_des", j_z_des_, 5.0);
  priv_nh.param("acc_yaw_des", a_yaw_des_, 0.1);
}

bool StoppingPolicy::Activate(const PositionCommand::ConstPtr& cmd) {
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

    // clip a0 to lie within [-a_des_abs, a_des_abs]
    double a_des_abs = 0.0;

    for (int axis = 0; axis < 3; ++axis) {
      if (axis == 0 || axis == 1) {
        a_des_abs = a_xy_des_;
      } else {
        a_des_abs = a_z_des_;
      }

      if (std::abs(a0_(axis)) > a_des_abs) {
        ROS_WARN_STREAM(
            "[StoppingPolicy:] initial acceleration is "
            << std::abs(a0_(axis))
            << ", which is larger than stopping policy max acceleration of "
            << a_des_abs
            << ". This is not normal since the stopping policy max "
               "acceleration "
               "should be no smaller than acceleration along the trajectory. "
               "Either plan less aggressive trajectories or increase stopping "
               "policy max acceleration!");
        a0_(axis) = std::clamp(a0_(axis), -a_des_abs, a_des_abs);
      }
    }

    cmd_acc_ = a0_;
    a0_dir_xyz_ << cmd->acceleration.x, cmd->acceleration.y,
        cmd->acceleration.z;
    a0_dir_xyz_.normalize();

    j0_ << cmd->jerk.x, cmd->jerk.y, cmd->jerk.z, 0.0;
    cmd_jrk_ = j0_;
    // t0_ = cmd->header.stamp;
    active_ = true;
    ROS_WARN("Stopping policy activated!");
    // IMPORTANT: resetting odom_first_call_ flag to make sure duration starts
    // from 0
    odom_first_call_ = true;
    ROS_WARN_STREAM("vx:" << cmd->velocity.x << "vy:" << cmd->velocity.y
                          << "vz:" << cmd->velocity.z);
    return true;
  } else {
    ROS_ERROR("Need starting command to stop");
  }

  return false;
}

void StoppingPolicy::Deactivate() {
  active_ = false;
  ROS_INFO("Stopping policy deactivated");
}

PositionCommand::ConstPtr StoppingPolicy::update(
    const nav_msgs::Odometry::ConstPtr& msg) {
  // this function is called whenever there's odometry msg, even if stopping
  // policy is deactivated
  if (!active_) {
    // return empty command, otherwise it will conflict with existing position
    // commands
    return PositionCommand::Ptr();
  }

  ros::Time stamp = msg->header.stamp;
  if (odom_first_call_) {
    ROS_INFO_STREAM("This is stopping first call, duration is reset as 0!");
    t0_ = stamp;
    odom_first_call_ = false;
  }

  double duration = (stamp - t0_).toSec();
  if (duration >= 2.0) {
    ROS_INFO_THROTTLE(
        1,
        "It has been %f seconds since the triggering of stopping policy.",
        duration);
  }
  double dt = duration - prev_duration_;
  prev_duration_ = duration;

  // a_des_abs and j_des_abs are absolute values of acceleration and jerk
  double a_des_abs, j_des_abs, deacc_time, a0, new_cmd_jrk_1d, new_cmd_acc_1d,
      new_cmd_vel_1d, new_cmd_pos_1d, phase0_time, t_phase1, t_phase2, t_phase3,
      v0_norm, avg_acc_phase1, avg_acc_phase3, total_deacc_abs, t_yaw, v0_dir;

  for (int axis = 0; axis < 3; ++axis) {
    if (axis == 0 || axis == 1) {
      a_des_abs = a_xy_des_;
      j_des_abs = j_xy_des_;
    } else {
      a_des_abs = a_z_des_;
      j_des_abs = j_z_des_;
    }

    // check the initial velocity direction
    v0_norm = std::abs(v0_(axis));
    if (v0_(axis) > 0) {
      v0_dir = 1.0;
    } else {
      v0_dir = -1.0;
    }

    a0 = a0_(axis);

    // first, change the acceleration from a0 to -a_des_abs
    // phase 1-3: increase |acceleration| -> const acc -> decrease |acc|
    // calculate time duration for each phase
    t_phase1 = std::abs(-v0_dir * a_des_abs - a0) /
               j_des_abs;  // from a0 to -v0_dir * a_des_abs
    t_phase3 = std::abs(0 - (-v0_dir * a_des_abs)) /
               j_des_abs;  // from -v0_dir *a_des_abs to 0

    // average acceleration for phase 1 and phase 3
    avg_acc_phase1 = (-v0_dir * a_des_abs + a0) / 2.0;
    avg_acc_phase3 = (-v0_dir * a_des_abs + 0) / 2.0;
    // absolute change in velocity during phase 1 and phase 3
    total_deacc_abs =
        std::abs((t_phase1 * avg_acc_phase1) + (t_phase3 * avg_acc_phase3));

    // 3 possible cases:
    if (v0_norm < 0.2) {
      ROS_INFO_STREAM_THROTTLE(1,
                               "magnitude of initial velocity along axis "
                                   << axis
                                   << " is very small, which is: " << v0_norm
                                   << " m/s, stopping policy directly setting "
                                      "velocity along this axis to 0");
      t_phase1 = 0;
      t_phase2 = 0;
      t_phase3 = 0;
    } else if ((-v0_dir * a0 > 0) && (pow(a0, 2) / (2 * j_des_abs) > v0_norm)) {
      // Case 1: a0 is same direction of change of v0 (which is -v0_dir), and
      // v0_norm is smaller than a0^2 / (2*j_des_abs), this means phase 3 will
      // completely stop the robot with a jerk > j_des, ad phase 1 or phase 2
      t_phase1 = 0;  //
      t_phase2 = 0;  // no constant acceleration phase in this case
      // update j_des so that the robot stops exactly when acc is is decreased
      // from a0 to 0
      ROS_WARN_STREAM_THROTTLE(
          1,
          "[Stopping policy:] Special case where phase 3 can completely stop "
          "the robot, original jerk is: "
              << j_des_abs);
      j_des_abs = pow(a0, 2) / (2 * v0_norm);
      ROS_WARN_STREAM("jerk is increased to: " << j_des_abs);
      t_phase3 = std::abs(0 - a0) / j_des_abs;  // from a0 to 0
    } else if (total_deacc_abs > v0_norm) {
      // Case 2: a_des_abs will not be reached, using geometric method to
      // solve this v_virtual is v0 + v_a0, where v_a0 is the change of v if
      // acceleration changes from a=0 to a=a0 with j_des_abs
      double v_a0_norm = pow(a0, 2) / (2 * j_des_abs);
      double v_virtual = v0_norm + v_a0_norm;
      // phase3: a_max_abs^2 / (2*j_des_abs)  = v_virtual / 2
      // => a_max_abs = sqrt(v_virtual * j_des_abs)
      // => a_max = -v0_dir * a_max_abs
      double a_max = -v0_dir * sqrt(v_virtual * j_des_abs);
      t_phase1 = std::abs(a_max - a0) / j_des_abs;  // from a0 to a_max
      t_phase2 = 0;  // no constant acceleration phase in this case
      t_phase3 = std::abs(0 - a_max) / j_des_abs;  // from a_max to 0
    } else {
      // Case 2: a_des_abs will be reached, t_phase1 and t_phase3 will be as
      // calculated above, and t_phase2 will be duration of constant acc
      t_phase2 = (v0_norm - total_deacc_abs) / a_des_abs;
    }

    // check: all durations should be non-negative
    if (t_phase1 < 0 || t_phase2 < 0 || t_phase3 < 0) {
      ROS_ERROR_STREAM(
          "[StoppingPolicy:] time for one of the phases is negative, "
          "t_phase1: "
          << t_phase1 << " t_phase2: " << t_phase2 << " t_phase3:" << t_phase3
          << " this is not correct!");
    }
    if (duration < 0) {
      ROS_ERROR_STREAM("[StoppingPolicy:] duration is negative:"
                       << duration << ", this is not correct!");
      ROS_ERROR_STREAM(
          "[StoppingPolicy:] duration is negative, investigate into this!!!");
    }

    if ((duration >= 0) && (duration < (t_phase1 + t_phase2 + t_phase3))) {
      // For different phases we have different cmd_jrk
      if (duration < t_phase1) {
        // phase 1: increase |acceleration|, cmd_jrk should be -v0_dir
        // *j_des_abs
        new_cmd_jrk_1d = -v0_dir * j_des_abs;
      } else if (duration < t_phase1 + t_phase2) {
        // phase 2: const acceleration, cmd_jrk should be 0
        new_cmd_jrk_1d = 0.0;
      } else if (duration < t_phase1 + t_phase2 + t_phase3) {
        // phase 3: decrease |acceleration|, cmd_jrk should be v0_dir *
        // j_des_abs
        new_cmd_jrk_1d = v0_dir * j_des_abs;
      }

      // midpoint integration
      // get cmd at end of dt
      // (note: dt happened right BEFORE current timestamp)
      new_cmd_acc_1d = cmd_acc_(axis) + cmd_jrk_(axis) * dt;
      new_cmd_vel_1d = cmd_vel_(axis) + cmd_acc_(axis) * dt;
      new_cmd_pos_1d = cmd_pos_(axis) + cmd_vel_(axis) * dt;
      // use mean of the derivatives at beginning and end to do integration
      cmd_pos_(axis) =
          cmd_pos_(axis) + 0.5 * (new_cmd_vel_1d + cmd_vel_(axis)) * dt;
      cmd_vel_(axis) =
          cmd_vel_(axis) + 0.5 * (new_cmd_acc_1d + cmd_acc_(axis)) * dt;
      cmd_acc_(axis) =
          cmd_acc_(axis) + 0.5 * (new_cmd_jrk_1d + cmd_jrk_(axis)) * dt;
      cmd_jrk_(axis) = new_cmd_jrk_1d;
    } else {
      // stopping policy for current axis finished, cmd_pos_(axis) will remain
      // the same
      cmd_jrk_(axis) = 0;
      cmd_acc_(axis) = 0;
      cmd_vel_(axis) = 0;
    }
  }  // loop over XYZ axes

  t_yaw = v0_(3) / a_yaw_des_;  // use consant acc for yaw
  // check whether already stopped (yaw)
  if (duration < t_yaw) {
    // evaluate yaw and yaw_dot
    cmd_vel_(3) = v0_(3) - v0_dir_yaw_ * a_yaw_des_;
    cmd_pos_(3) = p0_(3) + ((v0_(3) + cmd_vel_(3)) / 2.0) * duration;
  } else {
    // stopping policy for yaw finished, cmd_pos_(i) will remain the same
    cmd_vel_(3) = 0.0;
  }

  if ((duration >= t_phase1 + t_phase2 + t_phase3) && (duration >= t_yaw)) {
    ROS_INFO_THROTTLE(1.0,
                      "Stopping policy done! Keep publishing position command "
                      "to hover the quad...");
  }

  PositionCommand::Ptr cmd(new PositionCommand);
  cmd->header.stamp = stamp;
  cmd->header.frame_id = msg->header.frame_id;

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

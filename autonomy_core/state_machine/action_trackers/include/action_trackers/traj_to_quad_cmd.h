#pragma once

#include <kr_mav_msgs/msg/position_command.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "traj_opt_ros/trajectory.h"  // Trajectory

#include <memory>

namespace traj_opt {

kr_mav_msgs::msg::PositionCommand EvaluateTrajectory(
    const boost::shared_ptr<Trajectory> &traj, double dt,
    uint max_derr_eval = 3, double scaling = 1.0);

void EvaluateTrajectory(const boost::shared_ptr<Trajectory> &traj, double dt,
                        kr_mav_msgs::msg::PositionCommand *out,
                        uint max_derr_eval = 3, double scaling = 1.0);

// LQR based trajectory tracking
bool EvaluateTrajectoryPos(const boost::shared_ptr<Trajectory> &traj,
                           const nav_msgs::msg::Odometry::ConstSharedPtr &odom,
                           double err_max, double t_des, double ddt,
                           kr_mav_msgs::msg::PositionCommand *out);

}  // namespace traj_opt

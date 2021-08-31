#pragma once

#include <kr_mav_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>

#include "traj_opt_ros/trajectory.h"  // Trajectory

namespace traj_opt {

kr_mav_msgs::PositionCommand EvaluateTrajectory(
    const boost::shared_ptr<Trajectory> &traj, double dt,
    uint max_derr_eval = 3, double scaling = 1.0);

void EvaluateTrajectory(const boost::shared_ptr<Trajectory> &traj, double dt,
                        kr_mav_msgs::PositionCommand *out,
                        uint max_derr_eval = 3, double scaling = 1.0);

// LQR based trajectory tracking
bool EvaluateTrajectoryPos(const boost::shared_ptr<Trajectory> &traj,
                           const nav_msgs::Odometry::ConstPtr &odom,
                           double err_max, double t_des, double ddt,
                           kr_mav_msgs::PositionCommand *out);

}  // namespace traj_opt

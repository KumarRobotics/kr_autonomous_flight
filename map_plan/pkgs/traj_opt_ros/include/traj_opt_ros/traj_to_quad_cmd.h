#pragma once

#include <kr_mav_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>

#include "traj_opt_basic/trajectory.h"  // Trajectory

namespace traj_opt {

kr_mav_msgs::PositionCommand EvaluateTrajectory(
    const boost::shared_ptr<Trajectory> &traj, decimal_t dt,
    uint max_derr_eval = 3, decimal_t scaling = 1.0);

void EvaluateTrajectory(const boost::shared_ptr<Trajectory> &traj, decimal_t dt,
                        kr_mav_msgs::PositionCommand *out,
                        uint max_derr_eval = 3, decimal_t scaling = 1.0);

void EvaluateTrajectory(const boost::shared_ptr<Trajectory> &traj, decimal_t dt,
                        kr_mav_msgs::PositionCommand::Ptr &out,
                        uint max_derr_eval = 3, decimal_t scaling = 1.0);

// same as regular, just set yaw to tangent
void EvaluateTrajectoryTangentYaw(const boost::shared_ptr<Trajectory> &traj,
                                  decimal_t dt,
                                  kr_mav_msgs::PositionCommand::Ptr &out,
                                  double old_yaw, double max_yaw_speed,
                                  double ddt, double yaw_thr = M_PI);

// LQR based trajectory tracking
bool EvaluateTrajectoryPos(const boost::shared_ptr<Trajectory> &traj,
                           const nav_msgs::Odometry::ConstPtr &odom,
                           double err_max, double t_des, double ddt,
                           kr_mav_msgs::PositionCommand::Ptr &out);

}  // namespace traj_opt
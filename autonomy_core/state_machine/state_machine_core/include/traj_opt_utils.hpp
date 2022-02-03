#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <traj_opt_ros/msg_traj.h>

namespace state_machine {

// for new traj_opt backend, can only have 3d trajectories
traj_opt::VecD Make4d(const traj_opt::VecD& vec);

void VecToPose(const traj_opt::VecD& valn, geometry_msgs::Pose* pos);
void VecToTwist(const traj_opt::VecD& valn, geometry_msgs::Twist* vel);

// evaluate trajectory into pos, vel, acc, jrk
void EvaluateToMsgs(boost::shared_ptr<traj_opt::Trajectory> traj,
                    double dt,
                    geometry_msgs::Pose* pos,
                    geometry_msgs::Twist* vel,
                    geometry_msgs::Twist* acc,
                    geometry_msgs::Twist* jrk);

}  // namespace state_machine

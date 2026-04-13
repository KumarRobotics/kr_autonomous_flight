#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_opt_ros/msg_traj.h>

namespace state_machine {

// for new traj_opt backend, can only have 3d trajectories
traj_opt::VecD Make4d(const traj_opt::VecD& vec);

void VecToPose(const traj_opt::VecD& valn, geometry_msgs::msg::Pose* pos);
void VecToTwist(const traj_opt::VecD& valn, geometry_msgs::msg::Twist* vel);

// evaluate trajectory into pos, vel, acc, jrk
void EvaluateToMsgs(boost::shared_ptr<traj_opt::Trajectory> traj,
                    double dt,
                    geometry_msgs::msg::Pose* pos,
                    geometry_msgs::msg::Twist* vel,
                    geometry_msgs::msg::Twist* acc,
                    geometry_msgs::msg::Twist* jrk);

}  // namespace state_machine

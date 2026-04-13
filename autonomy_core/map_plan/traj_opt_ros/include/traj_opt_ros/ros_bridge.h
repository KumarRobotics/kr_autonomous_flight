// Copyright 2016 Michael Watterson
#pragma once

#include <kr_planning_msgs/msg/spline_trajectory.hpp>
#include <kr_planning_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

#include "traj_opt_ros/traj_data.h"

namespace traj_opt {

class TrajRosBridge {
 public:
  // make sure to run rclcpp::init() before calling this function or it won't
  // work
  static void publish_msg(const kr_planning_msgs::msg::SplineTrajectory &msg,
                          std::string frame_id = "map");

  static void publish_msg(const TrajData &data, std::string frame_id = "map");

  // Use global singleton paradignm.  All these things are private!
  // Keep your government out of my contructors!
 private:
  TrajRosBridge();
  static TrajRosBridge &instance();
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<kr_planning_msgs::msg::SplineTrajectory>::SharedPtr pub_;
};

kr_planning_msgs::msg::SplineTrajectory SplineTrajectoryFromTrajData(
    const TrajData &data);

TrajData TrajDataFromSplineTrajectory(
    const kr_planning_msgs::msg::SplineTrajectory &msg);

kr_planning_msgs::msg::SplineTrajectory SplineTrajectoryFromTrajectory(
    const kr_planning_msgs::msg::Trajectory &msg);

}  // namespace traj_opt

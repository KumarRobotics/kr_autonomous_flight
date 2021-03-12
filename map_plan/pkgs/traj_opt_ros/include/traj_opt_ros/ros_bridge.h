// Copyright 2016 Michael Watterson
#pragma once

#include <planning_ros_msgs/SplineTrajectory.h>
#include <planning_ros_msgs/Trajectory.h>
#include <ros/ros.h>

#include <string>

#include "traj_opt_ros/traj_data.h"

namespace traj_opt {
class TrajRosBridge {
 public:
  // make sure to run ros::init() before calling this function or it won't work
  static void publish_msg(const planning_ros_msgs::SplineTrajectory &msg,
                          std::string frame_id = "map");

  static void publish_msg(const TrajData &data, std::string frame_id = "map");

  // Use global singleton paradignm.  All these things are private!
  // Keep your government out of my contructors!
 private:
  TrajRosBridge();
  static TrajRosBridge &instance();
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

planning_ros_msgs::SplineTrajectory SplineTrajectoryFromTrajData(
    const TrajData &data);

TrajData TrajDataFromSplineTrajectory(
    const planning_ros_msgs::SplineTrajectory &msg);

planning_ros_msgs::SplineTrajectory SplineTrajectoryFromTrajectory(
    const planning_ros_msgs::Trajectory &msg);

}  // namespace traj_opt

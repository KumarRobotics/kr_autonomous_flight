// Copyright 2016 Michael Watterson
#pragma once

#include <kr_planning_msgs/SplineTrajectory.h>
#include <kr_planning_msgs/Trajectory.h>
#include <ros/ros.h>

#include <string>

#include "traj_opt_ros/traj_data.h"

namespace traj_opt {

class TrajRosBridge {
 public:
  // make sure to run ros::init() before calling this function or it won't work
  static void publish_msg(const kr_planning_msgs::SplineTrajectory &msg,
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

kr_planning_msgs::SplineTrajectory SplineTrajectoryFromTrajData(
    const TrajData &data);

TrajData TrajDataFromSplineTrajectory(
    const kr_planning_msgs::SplineTrajectory &msg);

kr_planning_msgs::SplineTrajectory SplineTrajectoryFromTrajectory(
    const kr_planning_msgs::Trajectory &msg);

}  // namespace traj_opt

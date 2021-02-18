// Copyright 2016 Michael Watterson
#ifndef TRAJ_OPT_ROS_ROS_BRIDGE_H_
#define TRAJ_OPT_ROS_ROS_BRIDGE_H_

#include <ros/ros.h>
#include <traj_opt_basic/traj_data.h>
#include <traj_opt_msgs/Polynomial.h>
#include <traj_opt_msgs/Spline.h>
#include <traj_opt_msgs/Trajectory.h>

#include <string>

class TrajRosBridge {
 public:
  // No need to instantiate pesky variables!
  static traj_opt_msgs::Trajectory convert(const traj_opt::TrajData &data);
  static traj_opt::TrajData convert(const traj_opt_msgs::Trajectory &msg);

  // make sure to run ros::init() before calling this function or it won't work
  static void publish_msg(const traj_opt_msgs::Trajectory &msg,
                          std::string frame_id = "map");
  static void publish_msg(const traj_opt::TrajData &data,
                          std::string frame_id = "map");

  // Use global singleton paradignm.  All these things are private!
  // Keep your government out of my contructors!
 private:
  TrajRosBridge();
  static TrajRosBridge &instance();
  ros::NodeHandle nh_;
  ros::Publisher pub_;
};

#endif  // TRAJ_OPT_ROS_ROS_BRIDGE_H_

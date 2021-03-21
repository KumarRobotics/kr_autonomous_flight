/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ROS
 */

#pragma once
#include <mpl_basis/trajectory.h>
#include <planning_ros_msgs/PrimitiveArray.h>
#include <planning_ros_msgs/Trajectory.h>

/// Primitive2D to primitive ROS message
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive2D &pr,
                                               double z = 0);

/// Primitive3D to primitive ROS message
planning_ros_msgs::Primitive toPrimitiveROSMsg(const Primitive3D &pr);

/// Multiple Primitive2D to Primitive array ROS message
planning_ros_msgs::PrimitiveArray toPrimitiveArrayROSMsg(
    const vec_E<Primitive2D> &prs, double z = 0);

/// Multiple Primitive3D to Primitive array ROS message
planning_ros_msgs::PrimitiveArray toPrimitiveArrayROSMsg(
    const vec_E<Primitive3D> &prs);

/// Trajectory2D class to trajectory ROS message
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory2D &traj,
                                                 double z = 0);

/// Trajectory3D class to trajectory ROS message
planning_ros_msgs::Trajectory toTrajectoryROSMsg(const Trajectory3D &traj);

/// ROS message to Primitive2D class
Primitive2D toPrimitive2D(const planning_ros_msgs::Primitive &pr);

/// ROS message to Primitive3D class
Primitive3D toPrimitive3D(const planning_ros_msgs::Primitive &pr);

/// ROS message to Trajectory2D class
Trajectory2D toTrajectory2D(const planning_ros_msgs::Trajectory &traj_msg);

/// ROS message to Trajectory3D class
Trajectory3D toTrajectory3D(const planning_ros_msgs::Trajectory &traj_msg);

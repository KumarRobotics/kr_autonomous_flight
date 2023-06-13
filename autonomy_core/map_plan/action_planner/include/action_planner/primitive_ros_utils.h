/**
 * @file primitive_ros_utils.h
 * @brief Interface between primitive classes and ROS
 */

#pragma once
#include <mpl_basis/trajectory.h>
#include <kr_planning_msgs/PrimitiveArray.h>
#include <kr_planning_msgs/Trajectory.h>

/// Primitive2D to primitive ROS message
kr_planning_msgs::Primitive toPrimitiveROSMsg(const MPL::Primitive2D &pr,
                                               double z = 0);

/// Primitive3D to primitive ROS message
kr_planning_msgs::Primitive toPrimitiveROSMsg(const MPL::Primitive3D &pr);

/// Multiple Primitive2D to Primitive array ROS message
kr_planning_msgs::PrimitiveArray toPrimitiveArrayROSMsg(
    const vec_E<MPL::Primitive2D> &prs, double z = 0);

/// Multiple Primitive3D to Primitive array ROS message
kr_planning_msgs::PrimitiveArray toPrimitiveArrayROSMsg(
    const vec_E<MPL::Primitive3D> &prs);

/// Trajectory2D class to trajectory ROS message
kr_planning_msgs::Trajectory toTrajectoryROSMsg(const MPL::Trajectory2D &traj,
                                                 double z = 0);

/// Trajectory3D class to trajectory ROS message
kr_planning_msgs::Trajectory toTrajectoryROSMsg(const MPL::Trajectory3D &traj);

/// ROS message to Primitive2D class
MPL::Primitive2D toPrimitive2D(const kr_planning_msgs::Primitive &pr);

/// ROS message to Primitive3D class
MPL::Primitive3D toPrimitive3D(const kr_planning_msgs::Primitive &pr);

/// ROS message to Trajectory2D class
MPL::Trajectory2D toTrajectory2D(const kr_planning_msgs::Trajectory &traj_msg);

/// ROS message to Trajectory3D class
MPL::Trajectory3D toTrajectory3D(const kr_planning_msgs::Trajectory &traj_msg);
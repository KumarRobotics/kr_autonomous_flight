#pragma once

#include <ros/ros.h>
#include <traj_opt_ros/msg_traj.h>

class TrajOptUtils {
 public:
  // for new traj_opt backend, can only have 3d trajectories
  static traj_opt::VecD make4d(const traj_opt::VecD &vec) {
    traj_opt::VecD out = traj_opt::VecD::Zero(4);
    long int rows = std::min(vec.rows(), static_cast<long int>(4));
    out.block(0, 0, rows, 1) = vec.block(0, 0, rows, 1);
    return out;
  }

  static void vec_to_pose(const traj_opt::VecD &valn,
                          geometry_msgs::Pose *pos) {
    if (pos != NULL) {
      traj_opt::VecD val = make4d(valn);
      pos->position.x = val(0), pos->position.y = val(1),
      pos->position.z = val(2);
      pos->orientation.w = std::cos(0.5 * val(3));
      pos->orientation.z = std::sin(0.5 * val(3));
    }
  }
  static void vec_to_twist(const traj_opt::VecD &valn,
                           geometry_msgs::Twist *vel) {
    if (vel != NULL) {
      traj_opt::VecD val = make4d(valn);
      vel->linear.x = val(0), vel->linear.y = val(1), vel->linear.z = val(2);
      vel->angular.z = val(3);
    }
  }

  // evaluate trajectory into pos, vel, acc, jrk
  static void evaluate_to_msgs(boost::shared_ptr<traj_opt::Trajectory> traj,
                               double dt, geometry_msgs::Pose *pos,
                               geometry_msgs::Twist *vel,
                               geometry_msgs::Twist *acc,
                               geometry_msgs::Twist *jrk) {
    if (traj == NULL) return;
    traj_opt::VecD val;
    traj->evaluate(dt, 0, val);
    vec_to_pose(val, pos);

    traj->evaluate(dt, 1, val);
    vec_to_twist(val, vel);

    traj->evaluate(dt, 2, val);
    vec_to_twist(val, acc);

    traj->evaluate(dt, 3, val);
    vec_to_twist(val, jrk);
  }
};

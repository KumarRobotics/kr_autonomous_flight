
#include <action_trackers/traj_to_quad_cmd.h>
#include <angles/angles.h>

namespace traj_opt {

using kr_mav_msgs::PositionCommand;

void EvaluateTrajectory(const boost::shared_ptr<Trajectory>& traj,
                        double dt,
                        PositionCommand* out,
                        uint max_derr_eval,
                        double scaling) {
  traj_opt::VecD val;

  traj->evaluate(dt, 0, val);

  if (val.rows() > 0) out->position.x = val(0);
  if (val.rows() > 1) out->position.y = val(1);
  if (val.rows() > 2) out->position.z = val(2);
  if (val.rows() > 3) out->yaw = val(3);
  // if (val.rows() == 5) out->chart = val(4) > 0.5;

  if (max_derr_eval < 1) return;
  traj->evaluate(dt, 1, val);
  if (val.rows() > 0) out->velocity.x = val(0);
  if (val.rows() > 1) out->velocity.y = val(1);
  if (val.rows() > 2) out->velocity.z = val(2);
  if (val.rows() > 3) out->yaw_dot = val(3);
  if (max_derr_eval < 2) return;

  traj->evaluate(dt, 2, val);
  if (val.rows() > 0) out->acceleration.x = val(0);
  if (val.rows() > 1) out->acceleration.y = val(1);
  if (val.rows() > 2) out->acceleration.z = val(2);
  if (max_derr_eval < 3) return;

  traj->evaluate(dt, 3, val);
  if (val.rows() > 0) out->jerk.x = val(0);
  if (val.rows() > 1) out->jerk.y = val(1);
  if (val.rows() > 2) out->jerk.z = val(2);
}

PositionCommand EvaluateTrajectory(const boost::shared_ptr<Trajectory>& traj,
                                   double dt,
                                   uint max_derr_eval,
                                   double scaling) {
  PositionCommand cmd;
  EvaluateTrajectory(traj, dt, &cmd, max_derr_eval, scaling);
  return cmd;
}

bool EvaluateTrajectoryPos(const boost::shared_ptr<Trajectory>& traj,
                           const nav_msgs::Odometry::ConstPtr& odom,
                           double err_max,
                           double t_des,
                           double ddt,
                           PositionCommand* out) {
  // return false if need to adjust time
  bool return_v = true;
  VecD pos = VecD(4, 1);
  pos << odom->pose.pose.position.x, odom->pose.pose.position.y,
      odom->pose.pose.position.z,
      2.0 * std::asin(odom->pose.pose.orientation.z);

  VecD val, vel;
  traj->evaluate(t_des, 0, val);  // position of traj
  traj->evaluate(t_des, 1, vel);  // velocity of traj

  Eigen::Vector2d diff_xy(val(0) - pos(0), val(1) - pos(1));
  if (diff_xy.norm() >= err_max) {
    printf("Distance between odom and traj in xy too large! It is: %f \n",
           diff_xy.norm());
    return_v = false;  // return false
  }

  EvaluateTrajectory(traj, t_des, out);
  return return_v;
}

}  // namespace traj_opt

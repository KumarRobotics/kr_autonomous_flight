#include <kr_mav_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <traj_opt_basic/trajectory.h>

class TrajToQuadCmd {
 public:
  // regular trajectory evaluation
  static void evaluate(const boost::shared_ptr<traj_opt::Trajectory> &traj,
                       traj_opt::decimal_t dt,
                       kr_mav_msgs::PositionCommand *out,
                       uint max_derr_eval = 3,
                       traj_opt::decimal_t scaling = 1.0);
  static void evaluate(const boost::shared_ptr<traj_opt::Trajectory> &traj,
                       traj_opt::decimal_t dt,
                       kr_mav_msgs::PositionCommand::Ptr &out,
                       uint max_derr_eval = 3,
                       traj_opt::decimal_t scaling = 1.0);
  static kr_mav_msgs::PositionCommand evaluate(
      const boost::shared_ptr<traj_opt::Trajectory> &traj,
      traj_opt::decimal_t dt, uint max_derr_eval = 3,
      traj_opt::decimal_t scaling = 1.0);
  // same as regular, just set yaw to tangent
  static void evaluateTagentYaw(
      const boost::shared_ptr<traj_opt::Trajectory> &traj,
      traj_opt::decimal_t dt, kr_mav_msgs::PositionCommand::Ptr &out,
      double old_yaw, double max_yaw_speed, double ddt, double yaw_thr = M_PI);
  // for werid gimbal stuff TODO: Cleanup this function
  static void handleSphere(const boost::shared_ptr<traj_opt::Trajectory> &traj,
                           traj_opt::decimal_t dt,
                           kr_mav_msgs::PositionCommand::Ptr &out,
                           geometry_msgs::Point &image_point);

  // LQR based trajectory tracking
  static bool evaluatePos(const boost::shared_ptr<traj_opt::Trajectory> &traj,
                          const nav_msgs::Odometry::ConstPtr &odom,
                          double err_max, double t_des, double ddt,
                          kr_mav_msgs::PositionCommand::Ptr &out);
};

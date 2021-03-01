#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <qp_traj_opt/solvers/gurobi_solver.h>
#include <qp_traj_opt/utils/types.h>
#include <qp_traj_ros/ros_bridge.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

using kr_mav_msgs::PositionCommand;
using kr_tracker_msgs::TrackerStatus;

class FallCatcher : public kr_trackers_manager::Tracker {
 public:
  FallCatcher(void);

  void Initialize(const ros::NodeHandle &nh);
  bool Activate(const PositionCommand::ConstPtr &cmd);
  void Deactivate(void);

  const PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg);
  const TrackerStatus::Ptr status();

 private:
  double default_v_des_, default_a_des_, epsilon_;
  float v_des_, a_des_;
  bool active_;

  Eigen::Vector3f start_, goal_, pos_;
  float yaw_, start_yaw_;
  double kx_[3], kv_[3];
  int window_{0};
  int index_;

  bool got_traj_{false};

  qp_traj_opt::MatD3 poses_;
  qp_traj_opt::MatD3 times_;
  std::vector<ros::Time> stamps_;
  ros::Time start_time_;
};

FallCatcher::FallCatcher(void) : active_(false) {}

void FallCatcher::Initialize(const ros::NodeHandle &nh) {
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "fall_catcher");

  priv_nh.param("default_v_des", default_v_des_, 0.5);
  priv_nh.param("default_a_des", default_a_des_, 2.5);
  priv_nh.param("epsilon", epsilon_, 0.1);
  priv_nh.param("window", window_, 50);

  v_des_ = default_v_des_;
  a_des_ = default_a_des_;

  poses_ = qp_traj_opt::MatD3::Zero(window_, 3);
  times_ = qp_traj_opt::MatD3::Ones(window_, 3);
  stamps_ = std::vector<ros::Time>(window_);
}

bool FallCatcher::Activate(const PositionCommand::ConstPtr &cmd) {
  if (index_ > 0) {
    active_ = true;
    return true;
  } else
    ROS_ERROR("Fall Catcher has no odom");

  return false;
}

void FallCatcher::Deactivate(void) { active_ = false; }

const PositionCommand::ConstPtr FallCatcher::update(
    const nav_msgs::Odometry::ConstPtr &msg) {
  poses_(index_ % window_, 0) = msg->pose.pose.position.x;
  poses_(index_ % window_, 1) = msg->pose.pose.position.y;
  poses_(index_ % window_, 2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  stamps_.at(index_ % window_) = msg->header.stamp;
  index_++;
  ros::Time stamp = msg->header.stamp;
  ros::Time start_calc = ros::Time::now();
  double duration = (stamp - start_time_).toSec();
  static boost::shared_ptr<qp_traj_opt::GurobiSolver> bezier;
  if (!active_) return PositionCommand::Ptr();

  // if have not got trajectory, check for free fall
  if (!got_traj_) {
    if (index_ < window_) return PositionCommand::Ptr();

    ros::Time t0 = stamps_.at((index_ - 1) % window_);
    for (uint i = 0; i < window_; i++) {
      times_(i, 1) = (stamps_.at(i) - t0).toSec();
      times_(i, 2) = 0.5 * std::pow(times_(i, 1), 2);
    }
    // this matrix is small, we can use inverse to solve A^TA = A^b
    qp_traj_opt::Mat3 ATA = times_.transpose() * times_;
    qp_traj_opt::Mat3 Ainv = ATA.inverse();

    qp_traj_opt::Vec3 acc, grav, diff, v0, p0;
    grav << 0.0, 0.0, -9.81;

    for (int i = 0; i < 3; i++) {
      qp_traj_opt::Vec3 coeff =
          Ainv * (times_.transpose() * poses_.block(0, i, window_, 1));
      acc(i) = coeff(2);
      v0(i) = coeff(1);
    }
    if (std::isnan(acc.norm())) {
      ROS_ERROR_STREAM("Matrix inversion failed");
      return PositionCommand::Ptr();
    }

    diff = acc - grav;
    if (v0.norm() < 1e-2) {
      ROS_WARN_STREAM_THROTTLE(1, "Quad is not moving");
      return PositionCommand::Ptr();
    }
    if (diff.norm() > epsilon_) {
      ROS_WARN_STREAM_THROTTLE(1, "Quad is not in freefall. acc << "
                                      << acc.transpose() << " v0 "
                                      << v0.norm());
      return PositionCommand::Ptr();
    }
    ROS_INFO_STREAM("Quad freefall detected, starting optimization. Acc is: "
                    << acc.transpose());

    // set bounding box to 2km box for now
    qp_traj_opt::VecD b(6);
    b << 1000, 1000, 1000, 1000, 1000, 1000;
    qp_traj_opt::MatD3 A = qp_traj_opt::MatD3::Zero(6, 3);
    A.block<3, 3>(0, 0) = -qp_traj_opt::Mat3::Identity();
    A.block<3, 3>(3, 0) = qp_traj_opt::Mat3::Identity();
    std::vector<qp_traj_opt::MatD3> As(1, A);
    std::vector<qp_traj_opt::VecD> bs(1, b);
    std::vector<qp_traj_opt::Waypoint> con(2);
    con.at(0).use_pos = true;
    con.at(0).use_vel = true;
    con.at(0).use_acc = true;
    con.at(1).use_pos = true;
    con.at(1).use_vel = true;
    con.at(1).use_acc = true;
    con.at(0).use_jrk = true;
    con.at(1).use_jrk = true;
    con.at(0).knot_id = 0;
    con.at(1).knot_id = -1;

    con.at(0).pos << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z, yaw_;
    con.at(0).vel << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z, msg->twist.twist.angular.z;
    con.at(0).acc.block<3, 1>(0, 0) = grav;
    double time = con.at(0).vel.block<3, 1>(0, 0).norm() / default_a_des_;
    qp_traj_opt::Vec3 vel_unit = con.at(0).vel.block<3, 1>(0, 0);
    vel_unit.normalize();
    vel_unit *= default_a_des_;

    qp_traj_opt::Vec3 pos_diff =
        con.at(0).vel.block<3, 1>(0, 0) * time + 0.5 * grav * time * time;
    // qp_traj_opt::Vec3 pos_diff = con.at(0).vel.block<3,1>(0,0)*time +
    // 0.5*vel_unit*time*time;
    con.at(1).pos = con.at(0).pos;
    con.at(1).pos.block<3, 1>(0, 0) += pos_diff;

    con.at(0).jrk(2) = 0.0 / 0.0;
    con.at(1).jrk(2) = 0.0 / 0.0;
    ROS_WARN_STREAM("Final pos will be: " << con.at(1).pos.transpose());
    // solver call:
    bezier = boost::make_shared<qp_traj_opt::GurobiSolver>();
    bool solved = false;
    bezier->setParams(0, 0, 0, 0, 0.15);
    std::vector<double> ds(1, 2 * time);

    bezier->setPolyParams(7, qp_traj_opt::LEGENDRE, 2);
    got_traj_ = bezier->solveTrajectory(con, As, bs, ds, 0, NULL, 0);

    if (!got_traj_) {
      ROS_ERROR_STREAM("Failed to optimize trajectory");
      return PositionCommand::Ptr();
    } else
      ROS_INFO("Got trajectory");
    QPRosBridge::publish_msg(bezier->getTrajectory()->serialize());
    stamp = msg->header.stamp + (ros::Time::now() - start_calc);
    start_time_ = msg->header.stamp;
    duration = (stamp - start_time_).toSec();
  }

  if (bezier == NULL) {
    ROS_ERROR_STREAM("Shared pointer dealocate?");
    return PositionCommand::Ptr();
  }

  PositionCommand::Ptr cmd(new PositionCommand);
  cmd->header.stamp = stamp;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  qp_traj_opt::Vec4 pos, vel, acc, jrk;
  //  if (durration < 1.0 || durration > 10.0)
  ROS_ERROR_STREAM_THROTTLE(1, "Evaluating duration"
                                   << duration << " total time "
                                   << bezier->getTrajectory()->getTotalTime());

  bezier->getTrajectory()->evaluate(duration, 0, pos);
  bezier->getTrajectory()->evaluate(duration, 1, vel);
  bezier->getTrajectory()->evaluate(duration, 2, acc);
  bezier->getTrajectory()->evaluate(duration, 3, jrk);

  cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
  cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
  cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1),
  cmd->acceleration.z = acc(2);
  cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);

  cmd->yaw = pos(3), cmd->yaw_dot = vel(3);

  return cmd;
}

const TrackerStatus::Ptr FallCatcher::status() {
#warning "Using status in place of action lib is a bad idea"
  if (!active_) return TrackerStatus::Ptr();

  TrackerStatus::Ptr msg(new TrackerStatus);

  msg->status = TrackerStatus::ACTIVE;

  return msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(FallCatcher, kr_trackers_manager::Tracker);

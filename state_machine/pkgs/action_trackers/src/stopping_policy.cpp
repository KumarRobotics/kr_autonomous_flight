#include <actionlib/server/simple_action_server.h>
#include <kr_tracker_msgs/LineTrackerGoal.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <traj_opt_basic/types.h>
#include <traj_opt_pro/nonlinear_trajectory.h>
#include <traj_opt_ros/ros_bridge.h>

using kr_mav_msgs::PositionCommand;
using kr_tracker_msgs::TrackerStatus;

class StoppingPolicy : public kr_trackers_manager::Tracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StoppingPolicy();

  void Initialize(const ros::NodeHandle &nh) override;
  bool Activate(const PositionCommand::ConstPtr &cmd) override;
  void Deactivate() override;

  PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg) override;
  uint8_t status() const override;

 private:
  double j_des_, a_des_;
  bool active_;

  float yaw_, start_yaw_;
  double kx_[3], kv_[3];
  int window_{0};

  bool got_odom_{false};
  bool got_traj_{false};

  traj_opt::VecD p0_, v0_, a0_, j0_;
  ros::Time t0_;
  PositionCommand cmd_0_;
  boost::shared_ptr<traj_opt::NonlinearTrajectory> solver_;
};

StoppingPolicy::StoppingPolicy(void) : active_(false) {}

void StoppingPolicy::Initialize(const ros::NodeHandle &nh) {
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);

  ros::NodeHandle priv_nh(nh, "stopping_policy");

  priv_nh.param("default_a_des", a_des_, 1.0);
  priv_nh.param("default_j_des", j_des_, 50.0);
  priv_nh.param("window", window_, 50);
}

bool StoppingPolicy::Activate(const PositionCommand::ConstPtr &cmd) {
  if (cmd != NULL) {
    p0_ = traj_opt::VecD::Zero(4, 1);
    v0_ = traj_opt::VecD::Zero(4, 1);
    a0_ = traj_opt::VecD::Zero(4, 1);
    j0_ = traj_opt::VecD::Zero(4, 1);
    p0_ << cmd->position.x, cmd->position.y, cmd->position.z, cmd->yaw;
    v0_ << cmd->velocity.x, cmd->velocity.y, cmd->velocity.z, cmd->yaw_dot;
    a0_ << cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z, 0.0;
    j0_ << cmd->jerk.x, cmd->jerk.y, cmd->jerk.z, 0.0;
    t0_ = cmd->header.stamp;
    cmd_0_ = *cmd;
    cmd_0_.velocity.x = 0.0;
    cmd_0_.velocity.y = 0.0;
    cmd_0_.velocity.z = 0.0;
    cmd_0_.acceleration.x = 0.0;
    cmd_0_.acceleration.y = 0.0;
    cmd_0_.acceleration.z = 0.0;
    cmd_0_.jerk.x = 0.0;
    cmd_0_.jerk.y = 0.0;
    cmd_0_.jerk.z = 0.0;
    active_ = true;
    return true;
  } else
    ROS_ERROR("Need starting command to stop");

  return false;
}

void StoppingPolicy::Deactivate(void) {
  active_ = false;
  got_traj_ = false;
}

PositionCommand::ConstPtr StoppingPolicy::update(
    const nav_msgs::Odometry::ConstPtr &msg) {
  got_odom_ = true;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  ros::Time stamp = msg->header.stamp;
  double duration = (stamp - t0_).toSec();

  if (!active_) return PositionCommand::Ptr();

  // if have not got trajectory, check for free fall
  if (!got_traj_) {
    // set bounding box to 2km box for now
    traj_opt::VecD b(6);
    b << 1000, 1000, 1000, 1000, 1000, 1000;
    traj_opt::MatD A = traj_opt::MatD::Zero(6, 4);
    A.block<3, 3>(0, 0) = -traj_opt::Mat3::Identity();
    A.block<3, 3>(3, 0) = traj_opt::Mat3::Identity();
    std::vector<traj_opt::MatD> As(1, A);
    std::vector<traj_opt::VecD> bs(1, b);
    std::vector<traj_opt::Waypoint> con(2);
    con.at(0).use_pos = true;
    con.at(0).use_vel = true;
    con.at(0).use_acc = true;
    con.at(1).use_pos = false;
    con.at(1).use_vel = true;
    con.at(1).use_acc = true;
    con.at(0).use_jrk = true;
    con.at(1).use_jrk = true;
    con.at(0).knot_id = 0;
    con.at(1).knot_id = -1;

    con.at(0).pos = p0_;
    con.at(0).vel = v0_;
    con.at(0).acc = a0_;
    con.at(0).jrk = j0_;

    double time_v = con.at(0).vel.block<3, 1>(0, 0).norm() / a_des_;
    double time_a = con.at(0).acc.block<3, 1>(0, 0).norm() / j_des_;
    double time = std::max(time_a, time_v);

    if (time <= 1e-5) {
      cmd_0_.header.stamp = stamp;
      return boost::make_shared<PositionCommand>(cmd_0_);
    }
    if (con.at(0).vel.norm() < 0.5) {
      return boost::make_shared<PositionCommand>(cmd_0_);
    }
    std::vector<std::pair<traj_opt::MatD, traj_opt::VecD>> polyhedra;
    polyhedra.push_back(std::make_pair(A, b));

    std::vector<double> ds(1, time);
    boost::shared_ptr<std::vector<traj_opt::decimal_t>> pds =
        boost::make_shared<std::vector<traj_opt::decimal_t>>(ds);

    // solver call:
    solver_ = boost::make_shared<traj_opt::NonlinearTrajectory>(con, polyhedra,
                                                                7, 3, pds);
    // solver_->setParams(0, 0, 0, 0, 0.15);

    // solver_->setPolyParams(7, traj_opt::LEGENDRE, 2);
    ROS_INFO_STREAM("Trying to stop with time: " << time);
    got_traj_ = solver_->isSolved();
    //    got_traj_ = solver_->solveTrajectory(con, As, bs, ds, 0, NULL, 0);

    if (!got_traj_) {
      ROS_ERROR_STREAM("Failed to optimize trajectory");
      return PositionCommand::Ptr();
    } else {
      ROS_INFO("Got trajectory");
      TrajRosBridge::publish_msg(solver_->serialize());
      t0_ = msg->header.stamp;
    }
  }

  if (solver_ == NULL) {
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

  traj_opt::VecD pos, vel, acc, jrk;
  // if (duration < 1.0 || duration > 10.0){
  //   ROS_ERROR_STREAM_THROTTLE(1, "Evaluating duration"
  //                                   << duration << " total time "
  //                                   << solver_->getTotalTime());
  // }

  solver_->evaluate(duration, 0, pos);
  solver_->evaluate(duration, 1, vel);
  solver_->evaluate(duration, 2, acc);
  solver_->evaluate(duration, 3, jrk);

  cmd->position.x = pos(0), cmd->position.y = pos(1), cmd->position.z = pos(2);
  cmd->velocity.x = vel(0), cmd->velocity.y = vel(1), cmd->velocity.z = vel(2);
  cmd->acceleration.x = acc(0), cmd->acceleration.y = acc(1),
  cmd->acceleration.z = acc(2);
  cmd->jerk.x = jrk(0), cmd->jerk.y = jrk(1), cmd->jerk.z = jrk(2);

  cmd->yaw = pos(3), cmd->yaw_dot = vel(3);

  return cmd;
}

uint8_t StoppingPolicy::status() const {
  return active_ ? TrackerStatus::ACTIVE : TrackerStatus::SUCCEEDED;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(StoppingPolicy, kr_trackers_manager::Tracker);

#include <action_trackers/LandAction.h>
#include <actionlib/server/simple_action_server.h>
#include <kr_mav_msgs/PositionCommand.h>
#include <kr_tracker_msgs/TrackerStatus.h>
#include <kr_trackers_manager/Tracker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Geometry>

/* This tracker should land no matter what height or mass offset
 *
 */

using kr_mav_msgs::PositionCommand;

class LandTracker : public kr_trackers_manager::Tracker {
 private:
  double done_epsilon_;  // will stop when the position command is _ m below the
                         // odom
  double land_vel_;  // landing velocity. With no ground effect, will hit the
                     // ground with this v
  double land_acc_;  // rate at which we reach the landing vel

 public:
  LandTracker() = default;

  void Initialize(const ros::NodeHandle &nh) override;
  bool Activate(const PositionCommand::ConstPtr &cmd) override;
  void Deactivate() override;

  PositionCommand::ConstPtr update(
      const nav_msgs::Odometry::ConstPtr &msg) override;
  uint8_t status() const override;

 private:
  void goal_callback();
  // action lib
  std::unique_ptr<actionlib::SimpleActionServer<action_trackers::LandAction>>
      as_;

  double kx_[3], kv_[3];
  Eigen::Vector3f pos_, cmd_pos_;
  double vel_z;
  float yaw_, start_yaw_;
  bool pos_set_{
      false};  // safety catch to make sure we don't activate with no odom
  bool active_{false};
  bool done_landing_{false};
  bool start_landing_{false};
  ros::Time old_t;
};

void LandTracker::Initialize(const ros::NodeHandle &nh) {
  nh.param("gains/pos/x", kx_[0], 2.5);
  nh.param("gains/pos/y", kx_[1], 2.5);
  nh.param("gains/pos/z", kx_[2], 5.0);
  nh.param("gains/vel/x", kv_[0], 2.2);
  nh.param("gains/vel/y", kv_[1], 2.2);
  nh.param("gains/vel/z", kv_[2], 4.0);
  kx_[0] *= 0.1;
  kx_[1] *= 0.1;
  kx_[2] *= 0.1;

  ros::NodeHandle priv_nh(nh, "land_tracker");
  priv_nh.param("epsilon", done_epsilon_, 2.0);  // see top of file
  priv_nh.param("vel", land_vel_, 1.0);          // see top of file
  priv_nh.param("acc", land_acc_, 2.0);          // see top of file

  if (kx_[2] * done_epsilon_ > 9.4) {
    done_epsilon_ = 9.4 / kx_[2];
    ROS_WARN_STREAM("Done Epsilon too large setting to " << done_epsilon_);
  }

  as_.reset(new actionlib::SimpleActionServer<action_trackers::LandAction>(
      nh, "land", false));
  // Register goal and preempt callbacks
  as_->registerGoalCallback(boost::bind(&LandTracker::goal_callback, this));

  as_->start();
}
void LandTracker::Deactivate() { active_ = false; }

bool LandTracker::Activate(const PositionCommand::ConstPtr &cmd) {
  if (!pos_set_) {
    ROS_ERROR("Do not have odom");
    return false;
  }
  done_landing_ = false;
  start_landing_ = false;
  vel_z = 0;

  if (cmd == NULL) {
    cmd_pos_ = pos_;
  } else {
    cmd_pos_ << cmd->position.x, cmd->position.y, cmd->position.z;
  }

  active_ = true;
  return true;
}

void LandTracker::goal_callback() {
  as_->acceptNewGoal();
  start_yaw_ = yaw_;

  if (!active_) {
    action_trackers::LandResult res;
    res.success = false;
    as_->setSucceeded(res);
    return;
  }
  start_landing_ = true;
}

PositionCommand::ConstPtr LandTracker::update(
    const nav_msgs::Odometry::ConstPtr &msg) {
  float dt = (msg->header.stamp - old_t).toSec();
  old_t = msg->header.stamp;

  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  yaw_ = tf::getYaw(msg->pose.pose.orientation);
  pos_set_ = true;
  if (!active_) return PositionCommand::Ptr();

  PositionCommand::Ptr cmd(new PositionCommand);
  cmd->header.stamp = msg->header.stamp;
  cmd->header.frame_id = msg->header.frame_id;
  cmd->yaw = start_yaw_;
  cmd->yaw_dot = 0;
  cmd->kx[0] = kx_[0], cmd->kx[1] = kx_[1], cmd->kx[2] = kx_[2];
  cmd->kv[0] = kv_[0], cmd->kv[1] = kv_[1], cmd->kv[2] = kv_[2];

  if (start_landing_ && !done_landing_) {
    if (vel_z > -land_vel_)
      vel_z -= land_acc_ * dt;
    else
      vel_z = -land_vel_;

    cmd_pos_(2) += vel_z * dt;

    // check epsilon
    if (cmd_pos_(2) + done_epsilon_ < pos_(2)) {
      ROS_INFO("Finished Landing");
      done_landing_ = true;
    }
  }
  if (as_->isActive() && done_landing_) {
    action_trackers::LandResult res;
    res.success = true;
    as_->setSucceeded(res);
  }

  cmd->position.x = cmd_pos_(0), cmd->position.y = cmd_pos_(1),
  cmd->position.z = cmd_pos_(2);
  cmd->velocity.z = vel_z;

  return cmd;
}

uint8_t LandTracker::status() const {
  return active_ ? kr_tracker_msgs::TrackerStatus::ACTIVE
                 : kr_tracker_msgs::TrackerStatus::SUCCEEDED;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(LandTracker, kr_trackers_manager::Tracker);

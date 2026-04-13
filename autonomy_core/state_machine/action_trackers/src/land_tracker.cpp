#include <action_trackers/gain_params.hpp>
#include <action_trackers/action/land.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <kr_mav_msgs/msg/position_command.hpp>
#include <kr_tracker_msgs/msg/tracker_status.hpp>
#include <kr_trackers_manager/Tracker.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <memory>
#include <string>

/* This tracker should land no matter what height or mass offset
 *
 */

using kr_mav_msgs::msg::PositionCommand;

class LandTracker : public kr_trackers_manager::Tracker {
 public:
  using Land = action_trackers::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;

  LandTracker() = default;

  void Initialize(const rclcpp::Node::SharedPtr &parent_nh) override;
  bool Activate(const PositionCommand::ConstSharedPtr &cmd) override;
  void Deactivate() override;

  PositionCommand::ConstSharedPtr update(
      const nav_msgs::msg::Odometry::ConstSharedPtr &msg) override;
  uint8_t status() const override;

 private:
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Land::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleLand> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandleLand> goal_handle);

  // action lib
  rclcpp::Node::SharedPtr nh_;
  rclcpp_action::Server<Land>::SharedPtr as_;
  // active goal handle (only one at a time is tracked)
  std::shared_ptr<GoalHandleLand> current_goal_handle_;

  double kx_[3], kv_[3];
  Eigen::Vector3f pos_, cmd_pos_;
  double vel_z;
  float yaw_, start_yaw_;
  bool pos_set_{
      false};  // safety catch to make sure we don't activate with no odom
  bool active_{false};
  bool done_landing_{false};
  bool start_landing_{false};
  rclcpp::Time old_t;

  double done_epsilon_;  // will stop when the position command is _ m below the
                         // odom
  double land_vel_;  // landing velocity. With no ground effect, will hit the
                     // ground with this v
  double land_acc_;  // rate at which we reach the landing vel
};

void LandTracker::Initialize(const rclcpp::Node::SharedPtr &parent_nh) {
  nh_ = parent_nh;
  action_trackers::declare_shared_gain_params(nh_);
  nh_->get_parameter("gains.pos.x", kx_[0]);
  nh_->get_parameter("gains.pos.y", kx_[1]);
  nh_->get_parameter("gains.pos.z", kx_[2]);
  nh_->get_parameter("gains.vel.x", kv_[0]);
  nh_->get_parameter("gains.vel.y", kv_[1]);
  nh_->get_parameter("gains.vel.z", kv_[2]);
  kx_[0] *= 0.1;
  kx_[1] *= 0.1;
  kx_[2] *= 0.1;

  nh_->declare_parameter("land_tracker.epsilon", 2.0);
  nh_->declare_parameter("land_tracker.vel", 1.0);
  nh_->declare_parameter("land_tracker.acc", 2.0);
  nh_->get_parameter("land_tracker.epsilon", done_epsilon_);
  nh_->get_parameter("land_tracker.vel", land_vel_);
  nh_->get_parameter("land_tracker.acc", land_acc_);

  if (kx_[2] * done_epsilon_ > 9.4) {
    done_epsilon_ = 9.4 / kx_[2];
    RCLCPP_WARN_STREAM(nh_->get_logger(),
                       "Done Epsilon too large setting to " << done_epsilon_);
  }

  as_ = rclcpp_action::create_server<Land>(
      nh_,
      "land",
      std::bind(&LandTracker::handleGoal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&LandTracker::handleCancel, this, std::placeholders::_1),
      std::bind(&LandTracker::handleAccepted, this, std::placeholders::_1));
}

void LandTracker::Deactivate() { active_ = false; }

bool LandTracker::Activate(const PositionCommand::ConstSharedPtr &cmd) {
  if (!pos_set_) {
    RCLCPP_ERROR(nh_->get_logger(), "Do not have odom");
    return false;
  }
  done_landing_ = false;
  start_landing_ = false;
  vel_z = 0;

  if (cmd == nullptr) {
    cmd_pos_ = pos_;
  } else {
    cmd_pos_ << cmd->position.x, cmd->position.y, cmd->position.z;
  }

  active_ = true;
  return true;
}

rclcpp_action::GoalResponse LandTracker::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const Land::Goal> goal) {
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandTracker::handleCancel(
    const std::shared_ptr<GoalHandleLand> goal_handle) {
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandTracker::handleAccepted(
    const std::shared_ptr<GoalHandleLand> goal_handle) {
  // equivalent to the old goal_callback: triggered when a new goal arrives.
  current_goal_handle_ = goal_handle;
  start_yaw_ = yaw_;

  if (!active_) {
    auto res = std::make_shared<Land::Result>();
    res->success = false;
    goal_handle->succeed(res);
    current_goal_handle_.reset();
    return;
  }
  start_landing_ = true;
}

PositionCommand::ConstSharedPtr LandTracker::update(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  rclcpp::Time stamp(msg->header.stamp);
  float dt = (stamp - old_t).seconds();
  old_t = stamp;

  pos_(0) = msg->pose.pose.position.x;
  pos_(1) = msg->pose.pose.position.y;
  pos_(2) = msg->pose.pose.position.z;
  tf2::Quaternion q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double roll_tmp, pitch_tmp, yaw_tmp;
  tf2::Matrix3x3(q).getRPY(roll_tmp, pitch_tmp, yaw_tmp);
  yaw_ = static_cast<float>(yaw_tmp);
  pos_set_ = true;
  if (!active_) return PositionCommand::ConstSharedPtr();

  auto cmd = std::make_shared<PositionCommand>();
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
      RCLCPP_INFO(nh_->get_logger(), "Finished Landing");
      done_landing_ = true;
    }
  }
  if (current_goal_handle_ && current_goal_handle_->is_active() &&
      done_landing_) {
    auto res = std::make_shared<Land::Result>();
    res->success = true;
    current_goal_handle_->succeed(res);
    current_goal_handle_.reset();
  }

  cmd->position.x = cmd_pos_(0), cmd->position.y = cmd_pos_(1),
  cmd->position.z = cmd_pos_(2);
  cmd->velocity.z = vel_z;

  return cmd;
}

uint8_t LandTracker::status() const {
  return active_ ? kr_tracker_msgs::msg::TrackerStatus::ACTIVE
                 : kr_tracker_msgs::msg::TrackerStatus::SUCCEEDED;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LandTracker, kr_trackers_manager::Tracker)

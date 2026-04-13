// Shared position / velocity gain parameter declarations for the
// action_trackers pluginlib tracker plugins.
//
// All tracker plugins (LandTracker, TakeOffTracker, ActionTrajectoryTracker,
// ...) load into the SAME parent trackers_manager rclcpp::Node. In ROS1 the
// individual plugins each did their own `nh_.param("gains/pos/x", ...)` calls
// — safe because nh_.param is idempotent. In ROS2 `declare_parameter()` on
// the same node throws `ParameterAlreadyDeclared` the second time, so this
// helper centralizes the declaration and wraps every call in a
// `has_parameter()` guard. The first tracker to Initialize() declares the
// parameters; subsequent trackers' calls become no-ops and then
// `get_parameter()` returns the value the first tracker (or the yaml
// override) set.
//
// If you are adding a new tracker plugin and you need the gains parameters,
// `#include <action_trackers/gain_params.hpp>` and call
// `action_trackers::declare_shared_gain_params(nh_)` from your Initialize().

#pragma once

namespace action_trackers {

// Templated so we don't care whether `nh_` is a rclcpp::Node::SharedPtr, a
// rclcpp::Node*, or anything else that has -> has_parameter / declare_parameter.
template <typename NodeLike>
inline void declare_shared_gain_params(NodeLike& node) {
  if (!node->has_parameter("gains.pos.x")) {
    node->declare_parameter("gains.pos.x", 2.5);
  }
  if (!node->has_parameter("gains.pos.y")) {
    node->declare_parameter("gains.pos.y", 2.5);
  }
  if (!node->has_parameter("gains.pos.z")) {
    node->declare_parameter("gains.pos.z", 5.0);
  }
  if (!node->has_parameter("gains.vel.x")) {
    node->declare_parameter("gains.vel.x", 2.2);
  }
  if (!node->has_parameter("gains.vel.y")) {
    node->declare_parameter("gains.vel.y", 2.2);
  }
  if (!node->has_parameter("gains.vel.z")) {
    node->declare_parameter("gains.vel.z", 4.0);
  }
}

}  // namespace action_trackers

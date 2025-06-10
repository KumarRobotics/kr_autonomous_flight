#include "mapper/tf_listener.h"

namespace mapper {

// std::optional<geometry_msgs::msg::Pose> TFListener::LookupTransform(
//     const std::string &target, const std::string &source,
//     const ros::Time &time) {
std::optional<geometry_msgs::msg::Pose> TFListener::LookupTransform(
    const std::string& target,
    const std::string& source,
    const rclcpp::Time& time){
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = buffer_->lookupTransform(target, source, time, rclcpp::Duration::from_seconds(0.4));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Fail to find transform from [%s] to [%s]",
                      source.c_str(), target.c_str());
    return {};
  }

  geometry_msgs::msg::Pose pose;

  pose.position.x = transformStamped.transform.translation.x;
  pose.position.y = transformStamped.transform.translation.y;
  pose.position.z = transformStamped.transform.translation.z;
  pose.orientation.w = transformStamped.transform.rotation.w;
  pose.orientation.x = transformStamped.transform.rotation.x;
  pose.orientation.y = transformStamped.transform.rotation.y;
  pose.orientation.z = transformStamped.transform.rotation.z;

  return pose;
}

}  // namespace mapper

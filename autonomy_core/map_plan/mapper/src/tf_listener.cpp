#include "mapper/tf_listener.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.hpp>

namespace mapper {

std::optional<geometry_msgs::msg::Pose> TFListener::LookupTransform(
    const std::string& target,
    const std::string& source,
    const rclcpp::Time& time,
    rclcpp::Logger logger) {
  geometry_msgs::msg::TransformStamped transformStamped;
  try {
    transformStamped = buffer_.lookupTransform(
        target, source, time, tf2::durationFromSec(0.4));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(logger,
                         *buffer_.getClock(),
                         1000,
                         "Fail to find transform from [%s] to [%s]",
                         source.c_str(),
                         target.c_str());
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

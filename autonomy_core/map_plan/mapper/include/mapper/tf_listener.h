#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <memory>
#include <optional>
#include <string>

namespace mapper {

class TFListener {
 public:
  explicit TFListener(rclcpp::Clock::SharedPtr clock)
      : buffer_(clock), listener_(buffer_) {}

  std::optional<geometry_msgs::msg::Pose> LookupTransform(
      const std::string& target,
      const std::string& source,
      const rclcpp::Time& time,
      rclcpp::Logger logger);

 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace mapper

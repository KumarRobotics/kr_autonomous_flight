#pragma once

// #include <geometry_msgs/Pose.h>
// #include <ros/time.h>
// #include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>

namespace mapper {

class TFListener {
 public:
  TFListener(rclcpp::Node::SharedPtr node) : buffer_(node->get_clock()), listener_(buffer_) {}

  // std::optional<geometry_msgs::Pose> LookupTransform(const std::string& target,
  //                                                    const std::string& source,
  //                                                    const ros::Time& time);
  std::optional<geometry_msgs::msg::Pose> LookupTransform(
      const std::string& target, const std::string& source,
      const rclcpp::Time& time);


 private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

}  // namespace mapper

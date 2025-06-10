#pragma once

// #include <geometry_msgs/Pose.h>
// #include <ros/time.h>
// #include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/time.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <optional>
#include <memory>

namespace mapper {

class TFListener {
 public:
  TFListener(rclcpp::Node::SharedPtr node) : node_(node), 
                                              buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
                                              listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_)) {}

  // std::optional<geometry_msgs::msg::Pose> LookupTransform(const std::string& target,
  //                                                    const std::string& source,
  //                                                    const ros::Time& time);
  std::optional<geometry_msgs::msg::Pose> LookupTransform(
      const std::string& target, const std::string& source,
      const rclcpp::Time& time);


 private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace mapper

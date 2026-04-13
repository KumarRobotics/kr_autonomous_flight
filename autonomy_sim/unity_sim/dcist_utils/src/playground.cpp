#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("playground");

  // ros::names::append has no direct equivalent in ROS 2; use simple string
  // concatenation similar to what it did.
  RCLCPP_INFO_STREAM(node->get_logger(), std::string("") + "right");
  RCLCPP_INFO_STREAM(node->get_logger(), std::string("left/") + "right");

  rclcpp::shutdown();
  return 0;
}

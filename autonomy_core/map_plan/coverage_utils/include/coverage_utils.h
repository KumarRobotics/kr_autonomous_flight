#pragma once
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <kr_planning_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utils.h>

#include <Eigen/StdVector>
#include <string>

class CoveragePlanner : public rclcpp::Node {
 public:
  CoveragePlanner();
  void RunPlanner();

 private:
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<kr_planning_msgs::msg::Path>::SharedPtr all_points_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      initial_polygon_publisher_;
  std::string fname_;
};

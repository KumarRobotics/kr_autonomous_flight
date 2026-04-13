#include <coverage_utils.h>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <math.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

CoveragePlanner::CoveragePlanner() : rclcpp::Node("coverage_planner") {
  initial_polygon_publisher_ =
      this->create_publisher<geometry_msgs::msg::PolygonStamped>(
          "original_polygon", 1);
  path_pub_ = this->create_publisher<kr_planning_msgs::msg::Path>(
      "coverage_path", rclcpp::QoS(1).transient_local());
  all_points_pub_ = this->create_publisher<kr_planning_msgs::msg::Path>(
      "all_input_points", rclcpp::QoS(1).transient_local());
  this->declare_parameter<std::string>("input_file_path",
                                       std::string("input.txt"));
  this->get_parameter("input_file_path", fname_);
}

void CoveragePlanner::RunPlanner() {
  // load all points
  auto pt_vec = PreprocessData(fname_);

  std::vector<pt>::iterator it;
  vec_Vec3f all_points;
  for (it = pt_vec.begin(); it != pt_vec.end(); it++) {
    all_points.push_back(Vec3f(it->x, it->y, 5));
  }

  // convex hull of all points
  convex_hull(&pt_vec);
  geometry_msgs::msg::PolygonStamped stamped_poly;
  stamped_poly.header.frame_id = "map";
  stamped_poly.header.stamp = this->now();
  for (it = pt_vec.begin(); it != pt_vec.end(); it++) {
    geometry_msgs::msg::Point32 point;
    point.x = static_cast<float>(it->x);
    point.y = static_cast<float>(it->y);
    point.z = static_cast<float>(0.0);
    stamped_poly.polygon.points.push_back(point);
  }

  auto all_points_msg = path_to_ros(all_points);
  all_points_msg.header.frame_id = "map";
  all_points_pub_->publish(all_points_msg);

  initial_polygon_publisher_->publish(stamped_poly);
}

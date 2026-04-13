#include <coverage_utils.h>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<CoveragePlanner>();
  rclcpp::Rate r(0.5);
  r.sleep();
  planner->RunPlanner();
  rclcpp::spin_some(planner);
  RCLCPP_INFO(planner->get_logger(), "Publishing data for coverage...");
  RCLCPP_INFO(
      planner->get_logger(),
      "You can visualize data in RVIZ by subscribing to /all_input_points "
      "topic, and change the fixed frame to map. The topic has the type: "
      "planning_ros_utils/Path (open RVIZ before launching this node).");
  rclcpp::shutdown();
  return 0;
}

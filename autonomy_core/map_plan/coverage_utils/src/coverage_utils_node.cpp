#include <coverage_utils.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "coverage_planner");
  ros::NodeHandle nh;
  CoveragePlanner planner;
  ros::Rate r(0.5);
  // while (nh.ok()) {
  r.sleep();
  planner.RunPlanner();
  ros::spinOnce();
  ROS_INFO("Publishing data for coverage...");
  ROS_INFO(
      "You can visualize data in RVIZ by subscribing to /all_input_points "
      "topic, and change the fixed frame to map. The topic has the type: "
      "planning_ros_utils/Path (open RVIZ before launching this node).");
  // }
}
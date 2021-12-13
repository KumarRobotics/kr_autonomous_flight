#pragma once
#include <planning_ros_msgs/Path.h>
#include <ros/ros.h>
#include <utils.h>

#include <Eigen/StdVector>

class CoveragePlanner {
 public:
  CoveragePlanner();
  void RunPlanner();

 private:
  ros::NodeHandle private_node_handle_;
  ros::Publisher path_pub_;
  ros::Publisher all_points_pub_;
  ros::Publisher initial_polygon_publisher_;
};

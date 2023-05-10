#pragma once
#include <kr_planning_msgs/Path.h>
#include <ros/ros.h>
#include <utils.h>

#include <Eigen/StdVector>
#include <string>

class CoveragePlanner {
 public:
  CoveragePlanner();
  void RunPlanner();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher path_pub_;
  ros::Publisher all_points_pub_;
  ros::Publisher initial_polygon_publisher_;
  std::string fname_;
};

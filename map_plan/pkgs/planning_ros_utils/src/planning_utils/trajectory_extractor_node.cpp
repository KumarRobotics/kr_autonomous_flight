#include <ros/ros.h>

#include "bag_writter.hpp"
#include "trajectory_extractor.hpp"

std::string file_name_, topic_name_;

void trajCallback(const planning_ros_msgs::Trajectory::ConstPtr& msg) {
  TrajectoryExtractor extractor(*msg, 0.01);
  const auto cmds = extractor.getCommands();

  write_bag<planning_ros_msgs::TrajectoryCommand>(file_name_, topic_name_,
                                                  cmds);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_extractor_node");

  ros::NodeHandle nh("~");
  nh.param("file", file_name_, std::string("trajectory_command.bag"));
  nh.param("topic", topic_name_, std::string("command"));

  ros::Subscriber traj_sub = nh.subscribe("trajectory", 1, trajCallback);

  ros::spin();

  return 0;
}

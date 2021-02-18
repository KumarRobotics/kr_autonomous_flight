#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "playground");

  ROS_INFO_STREAM(ros::names::append("", "right"));
  ROS_INFO_STREAM(ros::names::append("left", "right"));

  return 0;
}
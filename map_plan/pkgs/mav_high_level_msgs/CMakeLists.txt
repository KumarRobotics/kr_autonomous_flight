cmake_minimum_required(VERSION 3.10)
project(mav_high_level_msgs)

find_package(catkin REQUIRED COMPONENTS planning_ros_msgs nav_msgs
                                        message_generation)

add_message_files(
  DIRECTORY
  msg
  FILES
  StateTransition.msg
  Trajectory.msg
  TrajRequest.msg
  TrajResponse.msg)

generate_messages(DEPENDENCIES planning_ros_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS planning_ros_msgs nav_msgs message_runtime)

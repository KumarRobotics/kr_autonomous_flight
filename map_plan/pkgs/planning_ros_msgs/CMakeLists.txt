cmake_minimum_required(VERSION 3.10)
project(planning_ros_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation)

add_message_files(
  DIRECTORY
  msg
  FILES
  LambdaSeg.msg
  Log.msg
  Path.msg
  PathArray.msg
  Polynomial.msg
  Primitive.msg
  PrimitiveArray.msg
  Spline.msg
  SplineTrajectory.msg
  Trajectory.msg
  TrajectoryCommand.msg
  VoxelMap.msg)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime)

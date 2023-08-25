cmake_minimum_required(VERSION 3.10)
project(planning_ros_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs
                                        message_generation actionlib_msgs)

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
  TrajectoryPoints.msg
  StateTransition.msg
  VoxelMap.msg)

add_action_files(DIRECTORY action FILES PlanLocalPath.action PlanPath.action PlanTwoPoint.action)

generate_messages(DEPENDENCIES geometry_msgs nav_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs actionlib_msgs
                              message_runtime)

cmake_minimum_required(VERSION 3.10)
project(traj_opt_ros)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS roscpp traj_opt_basic planning_ros_msgs
                                        kr_mav_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  kr_mav_msgs
  traj_opt_basic
  planning_ros_msgs)

add_library(${PROJECT_NAME} src/ros_bridge.cpp src/traj_to_quad_cmd.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

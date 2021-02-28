cmake_minimum_required(VERSION 3.10)
project(traj_opt_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_HOPF=Off")

find_package(catkin REQUIRED COMPONENTS roscpp traj_opt_basic planning_ros_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  traj_opt_basic
  planning_ros_msgs)

add_library(${PROJECT_NAME} src/ros_bridge.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

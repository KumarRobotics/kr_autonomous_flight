cmake_minimum_required(VERSION 3.10)
project(coverage_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  planning_ros_msgs
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  planning_ros_msgs
)

add_library(${PROJECT_NAME} 
  src/coverage_utils.cpp
  src/utils.cpp
  )
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(coverage_utils_node
  src/coverage_utils_node.cpp)
target_link_libraries(coverage_utils_node PUBLIC
${PROJECT_NAME} ${catkin_LIBRARIES})
target_include_directories(coverage_utils_node PUBLIC 
  ${PROJECT_NAME} 
  ${catkin_INCLUDE_DIRS})
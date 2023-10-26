cmake_minimum_required(VERSION 3.10)
project(coverage_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  kr_planning_msgs
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  kr_planning_msgs
)

include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} 
  src/coverage_utils.cpp
  src/utils.cpp
  )
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  Eigen3::Eigen)

add_executable(coverage_utils_node
  src/coverage_utils_node.cpp)
target_link_libraries(coverage_utils_node PUBLIC
${PROJECT_NAME} ${catkin_LIBRARIES})
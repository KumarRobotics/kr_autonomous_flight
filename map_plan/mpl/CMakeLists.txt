cmake_minimum_required(VERSION 3.10)
project(motion_primitive_library)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} DEPENDS EIGEN3)

add_library(
  ${PROJECT_NAME}
  src/math.cpp
  src/lambda.cpp
  src/primitive.cpp
  src/trajectory.cpp
  src/waypoint.cpp
  src/map_util.cpp
  src/env_base.cpp
  src/env_map.cpp
  src/map_planner.cpp
  src/state_space.cpp
  src/graph_search.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

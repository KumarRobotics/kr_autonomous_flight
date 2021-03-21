cmake_minimum_required(VERSION 3.10)
project(motion_primitive_library)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  mpl_poly_solver
  mpl_map_planner
  DEPENDS
  EIGEN3)

add_library(mpl_poly_solver src/mpl_traj_solver/poly_solver.cpp
                            src/mpl_traj_solver/poly_traj.cpp)
target_include_directories(mpl_poly_solver PUBLIC include)
target_link_libraries(mpl_poly_solver PUBLIC Eigen3::Eigen)

add_library(mpl_map_planner src/mpl_planner/map_planner.cpp)
target_include_directories(mpl_map_planner PUBLIC include)
target_link_libraries(mpl_map_planner PUBLIC Eigen3::Eigen)

# add_subdirectory(test)

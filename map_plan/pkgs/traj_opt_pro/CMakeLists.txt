cmake_minimum_required(VERSION 3.10)
project(traj_opt_pro)

set(CMAKE_CXX_STANDARD 11)

# Find required packages
find_package(catkin_simple REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_simple()

# Add libraries
cs_add_library(common src/polynomial_basis.cpp src/trajectory_solver.cpp)

cs_add_library(
  fancy_custom_backend
  src/nonlinear_polynomial.cpp
  src/nonlinear_solver.cpp
  src/nonlinear_trajectory.cpp
  src/costs.cpp
  src/cheby_fit.cpp
  src/manifold_cost_exprs.cpp)
target_link_libraries(fancy_custom_backend common ${OpenCV_LIBRARIES} nlopt m)

cs_install()

cs_export(DEPENDS EIGEN3 OpenCV)

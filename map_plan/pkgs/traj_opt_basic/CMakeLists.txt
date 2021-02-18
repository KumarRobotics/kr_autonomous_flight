cmake_minimum_required(VERSION 2.8.3)
project(traj_opt_basic)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_simple()

cs_add_library(traj_opt_basic src/msg_traj.cpp src/polynomial_basis.cpp
               src/trajectory.cpp src/hopf_helper.cpp)

cs_install()

cs_export(DEPENDS EIGEN3 Boost)

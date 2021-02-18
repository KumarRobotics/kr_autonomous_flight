cmake_minimum_required(VERSION 2.8.3)
project(primitive_to_traj_opt)


find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(primitive_to_traj_opt src/convert)

cs_install()
cs_export()
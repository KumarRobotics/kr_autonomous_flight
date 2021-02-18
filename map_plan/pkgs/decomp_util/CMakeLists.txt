cmake_minimum_required(VERSION 3.10)
project(decomp_util)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED)
catkin_package(INCLUDE_DIRS include)

add_library(${PROJECT_NAME} src/dummy.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)

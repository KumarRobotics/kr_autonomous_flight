cmake_minimum_required(VERSION 3.16)
project(px4_interface_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

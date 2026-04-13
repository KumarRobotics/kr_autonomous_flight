cmake_minimum_required(VERSION 3.16)
project(real_experiment_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY util
  DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)

ament_package()

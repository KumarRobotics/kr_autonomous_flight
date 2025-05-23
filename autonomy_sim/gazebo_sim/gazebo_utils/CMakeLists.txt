cmake_minimum_required(VERSION 3.10)
project(gazebo_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
ament_package()

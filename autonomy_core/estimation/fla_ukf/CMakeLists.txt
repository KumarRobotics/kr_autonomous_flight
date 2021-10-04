cmake_minimum_required(VERSION 3.10)
project(fla_ukf)

set(CMAKE_CXX_STANDARD 17)
add_definitions(-D VIO_NO_POSITION=0 -D ENABLE_VIO_YAW_OFFSET=0)

find_package(
  catkin REQUIRED
  COMPONENTS eigen_conversions
             nav_msgs
             nodelet
             roscpp
             sensor_msgs
             tf2_ros
             angles)

find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include)

add_library(${PROJECT_NAME}_nodelet src/fla_ukf_nodelet.cpp src/fla_ukf.cpp)
target_include_directories(${PROJECT_NAME}_nodelet
                           PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_nodelet PRIVATE ${catkin_LIBRARIES})

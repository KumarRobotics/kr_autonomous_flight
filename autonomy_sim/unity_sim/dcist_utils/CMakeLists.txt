cmake_minimum_required(VERSION 3.16)
project(dcist_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

set(DCIST_DEPS
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs)

add_executable(odom2tf src/odom2tf.cpp)
ament_target_dependencies(odom2tf ${DCIST_DEPS})
target_link_libraries(odom2tf Eigen3::Eigen)

add_executable(msckf_calib_gen src/msckf_calib_gen.cpp)
target_include_directories(msckf_calib_gen PRIVATE ${YAML_CPP_INCLUDE_DIR})
ament_target_dependencies(msckf_calib_gen ${DCIST_DEPS})
target_link_libraries(msckf_calib_gen ${YAML_CPP_LIBRARIES} Eigen3::Eigen)

add_executable(playground src/playground.cpp)
ament_target_dependencies(playground rclcpp)

install(TARGETS odom2tf msckf_calib_gen playground
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.16)
project(mapper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(kr_planning_rviz_plugins REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/tf_listener.cpp
  src/voxel_mapper.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  kr_planning_msgs
  kr_planning_rviz_plugins
  motion_primitive_library
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(local_global_mapper src/local_global_mapper.cpp)
target_link_libraries(local_global_mapper
  ${PROJECT_NAME}
  Boost::timer
  ${PCL_LIBRARIES}
)
ament_target_dependencies(local_global_mapper
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_ros
  pcl_conversions
  kr_planning_msgs
  kr_planning_rviz_plugins
  motion_primitive_library
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS local_global_mapper
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_mapper test/test_voxel_mapper.cpp)
  target_link_libraries(test_voxel_mapper ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  kr_planning_msgs
  kr_planning_rviz_plugins
  motion_primitive_library
  Eigen3
  PCL
)

ament_package()

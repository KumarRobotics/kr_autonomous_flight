cmake_minimum_required(VERSION 3.16)
project(action_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
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
find_package(mapper REQUIRED)
find_package(jps3d REQUIRED)
find_package(kr_planning_rviz_plugins REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(traj_opt_ros REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(opt_planner REQUIRED)
find_package(decomp_ros_utils REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(motion_primitives REQUIRED)
find_package(gcopter REQUIRED)
find_package(kr_ilqr_optimizer REQUIRED)
find_package(kr_tracker_msgs REQUIRED)
find_package(param_env_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

set(common_deps
  rclcpp
  rclcpp_action
  action_msgs
  std_msgs
  std_srvs
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
  mapper
  jps3d
  kr_planning_rviz_plugins
  kr_planning_msgs
  traj_opt_ros
  motion_primitive_library
  opt_planner
  decomp_ros_utils
  decomp_ros_msgs
  motion_primitives
  gcopter
  kr_ilqr_optimizer
  kr_tracker_msgs
  param_env_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(planners SHARED
  src/planner_details.cpp
  src/data_conversions.cpp
  src/primitive_ros_utils.cpp
)
ament_target_dependencies(planners ${common_deps})
target_link_libraries(planners
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

add_executable(global_plan_server src/global_plan_server.cpp)
target_link_libraries(global_plan_server planners Boost::timer)
ament_target_dependencies(global_plan_server ${common_deps})

add_executable(local_plan_server src/local_plan_server.cpp)
target_link_libraries(local_plan_server planners Boost::timer)
ament_target_dependencies(local_plan_server ${common_deps})

install(TARGETS planners
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS global_plan_server local_plan_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(planners)
ament_export_dependencies(${common_deps} Eigen3 PCL)

ament_package()

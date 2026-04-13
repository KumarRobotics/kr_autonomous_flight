cmake_minimum_required(VERSION 3.16)
project(state_machine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

find_package(kr_mav_msgs REQUIRED)
find_package(kr_planning_rviz_plugins REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(traj_opt_ros REQUIRED)
find_package(action_trackers REQUIRED)
find_package(action_planner REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/Replan.action
  DEPENDENCIES std_msgs geometry_msgs action_msgs builtin_interfaces)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_lib SHARED src/traj_opt_utils.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp geometry_msgs traj_opt_ros)

add_executable(local_global_replan_server src/local_global_replan_server.cpp)
ament_target_dependencies(local_global_replan_server
  rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros
  tf2_geometry_msgs kr_mav_msgs kr_planning_msgs kr_planning_rviz_plugins
  traj_opt_ros action_trackers action_planner)
target_link_libraries(local_global_replan_server
  ${PROJECT_NAME}_lib Boost::timer Eigen3::Eigen)

# Link this package's generated Replan interfaces into the executable.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(local_global_replan_server "${cpp_typesupport_target}")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS local_global_replan_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs nav_msgs
  kr_mav_msgs kr_planning_msgs traj_opt_ros action_trackers
  rosidl_default_runtime)

ament_package()

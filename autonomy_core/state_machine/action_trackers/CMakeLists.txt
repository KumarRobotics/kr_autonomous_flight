cmake_minimum_required(VERSION 3.16)
project(action_trackers)

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
find_package(pluginlib REQUIRED)

find_package(kr_mav_msgs REQUIRED)
find_package(kr_tracker_msgs REQUIRED)
find_package(kr_trackers REQUIRED)
find_package(kr_trackers_manager REQUIRED)
find_package(kr_planning_rviz_plugins REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(traj_opt_ros REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(mapper REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/Land.action
  action/TakeOff.action
  action/RunTrajectory.action
  action/ShortRange.action
  action/GoalDistance.action
  action/RunPath.action
  DEPENDENCIES std_msgs geometry_msgs nav_msgs kr_planning_msgs kr_mav_msgs
               kr_tracker_msgs action_msgs builtin_interfaces)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Core traj_to_quad_cmd library
add_library(${PROJECT_NAME} SHARED src/traj_to_quad_cmd.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action kr_mav_msgs nav_msgs traj_opt_ros)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# So downstream C++ includes generated interface headers from this package:
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME} "${cpp_typesupport_target}")

# stopping_policy tracker plugin
add_library(stopping_policy SHARED src/stopping_policy.cpp)
ament_target_dependencies(stopping_policy
  rclcpp rclcpp_action kr_mav_msgs kr_tracker_msgs kr_trackers_manager
  nav_msgs pluginlib)
target_link_libraries(stopping_policy ${PROJECT_NAME} Eigen3::Eigen)

# take_off_tracker plugin
add_library(take_off_tracker SHARED src/take_off_tracker.cpp)
ament_target_dependencies(take_off_tracker
  rclcpp rclcpp_action kr_mav_msgs kr_tracker_msgs kr_trackers
  kr_trackers_manager nav_msgs tf2 tf2_geometry_msgs pluginlib)
target_link_libraries(take_off_tracker ${PROJECT_NAME} Eigen3::Eigen
  "${cpp_typesupport_target}")

# land_tracker plugin
add_library(land_tracker SHARED src/land_tracker.cpp)
ament_target_dependencies(land_tracker
  rclcpp rclcpp_action kr_mav_msgs kr_tracker_msgs kr_trackers_manager
  nav_msgs tf2 tf2_geometry_msgs pluginlib)
target_link_libraries(land_tracker ${PROJECT_NAME} Eigen3::Eigen
  "${cpp_typesupport_target}")

# action_trajectory_tracker plugin
add_library(action_trajectory_tracker SHARED src/trajectory_tracker_upgraded.cpp)
ament_target_dependencies(action_trajectory_tracker
  rclcpp rclcpp_action kr_mav_msgs kr_tracker_msgs kr_trackers_manager
  kr_planning_msgs nav_msgs geometry_msgs std_msgs tf2 tf2_geometry_msgs
  traj_opt_ros pluginlib)
target_link_libraries(action_trajectory_tracker ${PROJECT_NAME} Eigen3::Eigen
  "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(kr_trackers_manager nodelet_plugin.xml)

install(TARGETS
  ${PROJECT_NAME}
  stopping_policy
  take_off_tracker
  land_tracker
  action_trajectory_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(FILES nodelet_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} stopping_policy take_off_tracker
  land_tracker action_trajectory_tracker)
ament_export_dependencies(rclcpp rclcpp_action kr_mav_msgs kr_tracker_msgs
  kr_trackers_manager kr_planning_msgs nav_msgs tf2 tf2_geometry_msgs
  traj_opt_ros rosidl_default_runtime)

ament_package()

cmake_minimum_required(VERSION 3.10)
project(fla_state_machine)

set(CMAKE_CXX_STANDARD 14)

find_package(
  catkin REQUIRED
  COMPONENTS kr_mav_msgs
             planning_ros_utils
             traj_opt_ros
             planning_ros_msgs
             actionlib
             actionlib_msgs
             action_planner
             action_trackers
             geometry_msgs)

add_action_files(DIRECTORY action FILES Replan.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs)

add_library(${PROJECT_NAME} src/traj_opt_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(local_global_replan_server src/local_global_replan_server.cpp)
target_link_libraries(local_global_replan_server PUBLIC ${PROJECT_NAME})
add_dependencies(local_global_replan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})


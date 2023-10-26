cmake_minimum_required(VERSION 3.10)
project(action_trackers)

set(CMAKE_CXX_STANDARD 17)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             actionlib
             actionlib_msgs
             mapper
             kr_mav_msgs
             kr_trackers
             kr_tracker_msgs
             kr_trackers_manager
             kr_planning_rviz_plugins
             kr_planning_msgs
             traj_opt_ros
             motion_primitive_library)

find_package(Eigen3 REQUIRED)

add_action_files(
  DIRECTORY
  action
  FILES
  Land.action
  TakeOff.action
  RunTrajectory.action
  ShortRange.action
  GoalDistance.action
  RunPath.action)
generate_messages(DEPENDENCIES actionlib_msgs kr_planning_msgs kr_mav_msgs
                  kr_tracker_msgs)

catkin_package(CATKIN_DEPENDS kr_mav_msgs actionlib_msgs kr_planning_msgs
                              kr_tracker_msgs)

# src/projector.cpp only used for previous experiments where we directly track A* (JPS) planned path
# add_library(${PROJECT_NAME} src/projector.cpp src/traj_to_quad_cmd.cpp)
add_library(${PROJECT_NAME} src/traj_to_quad_cmd.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_library(stopping_policy src/stopping_policy.cpp)
target_link_libraries(stopping_policy PRIVATE ${PROJECT_NAME})

# src/path_tracker.cpp only used for previous experiments where we directly track A* (JPS) planned path
# add_library(action_path_tracker src/path_tracker.cpp)
# target_link_libraries(action_path_tracker PRIVATE ${PROJECT_NAME})

add_library(take_off_tracker src/take_off_tracker.cpp)
target_link_libraries(take_off_tracker PRIVATE ${PROJECT_NAME})

add_library(land_tracker src/land_tracker.cpp)
target_link_libraries(land_tracker PRIVATE ${PROJECT_NAME})

add_library(action_trajectory_tracker src/trajectory_tracker_upgraded.cpp)
target_link_libraries(action_trajectory_tracker PRIVATE ${PROJECT_NAME})

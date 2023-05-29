cmake_minimum_required(VERSION 3.10)
project(state_machine)

set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(
  catkin REQUIRED
  COMPONENTS kr_mav_msgs
             kr_planning_rviz_plugins
             traj_opt_ros
             kr_planning_msgs
             actionlib
             sensor_msgs
             actionlib_msgs
             action_planner
             tf2_ros
             action_trackers
             geometry_msgs
             eigen_conversions
             tf_conversions)

add_action_files(DIRECTORY action FILES Replan.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs)

add_library(${PROJECT_NAME} src/traj_opt_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(local_global_replan_server src/local_global_replan_server.cpp)
target_include_directories(local_global_replan_server
                           PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  local_global_replan_server PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES}
                                    Boost::timer Eigen3::Eigen)
add_dependencies(local_global_replan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

# JPS + line tracker based replanner, no longer used since we are using motion primitive planner
# add_executable(path_replanner src/path_replanner.cpp)
# target_link_libraries(path_replanner PUBLIC ${PROJECT_NAME})
# add_dependencies(path_replanner ${${PROJECT_NAME}_EXPORTED_TARGETS}
# ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 3.10)
project(fla_state_machine)

set(CMAKE_CXX_STANDARD 14)

find_package(
  catkin REQUIRED
  COMPONENTS kr_mav_msgs
             planning_ros_utils
             traj_opt_ros
             traj_opt_msgs
             actionlib
             actionlib_msgs
             action_planner
             action_trackers
             geometry_msgs
             )
            
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_action_files(DIRECTORY action FILES Replan.action)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs)

catkin_package(
   CATKIN_DEPENDS actionlib_msgs geometry_msgs)

add_executable(local_global_replan_server src/local_global_replan_server.cpp)
target_include_directories(local_global_replan_server PUBLIC include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(local_global_replan_server PUBLIC ${catkin_LIBRARIES})
add_dependencies(local_global_replan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
                 
                 
add_executable(path_replanner src/path_replanner.cpp)
target_include_directories(path_replanner PUBLIC include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
target_link_libraries(path_replanner PUBLIC ${catkin_LIBRARIES})
add_dependencies(path_replanner ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})


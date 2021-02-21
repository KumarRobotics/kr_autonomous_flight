cmake_minimum_required(VERSION 3.10)
project(action_planner)

set(CMAKE_CXX_STANDARD 14)

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             genmsg
             actionlib_msgs
             actionlib
             eigen_conversions
             tf2_ros
             tf_conversions
             pcl_ros
             mapper
             jps3d
             planning_ros_utils
             planning_ros_msgs
             mav_high_level_msgs
             traj_opt_ros
             dynamic_reconfigure
             primitive_to_traj_opt
             motion_primitive_library)

find_package(Eigen3 REQUIRED)

add_action_files(DIRECTORY action FILES PlanPath.action PlanTwoPoint.action
                 PlanWaypoints.action)
generate_messages(DEPENDENCIES actionlib_msgs planning_ros_msgs)
generate_dynamic_reconfigure_options(cfg/ActionPlanner.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure planning_ros_msgs
               actionlib_msgs)

add_executable(global_plan_server src/global_plan_server.cpp)
target_include_directories(global_plan_server PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(global_plan_server PUBLIC ${catkin_LIBRARIES})
add_dependencies(global_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(local_plan_server src/local_plan_server.cpp)
target_include_directories(local_plan_server PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(local_plan_server PUBLIC ${catkin_LIBRARIES})
add_dependencies(local_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
                 
                 

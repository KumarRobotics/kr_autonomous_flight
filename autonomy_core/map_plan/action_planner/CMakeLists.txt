cmake_minimum_required(VERSION 3.10)
project(action_planner)

set(CMAKE_CXX_STANDARD 17)
find_package(Boost REQUIRED COMPONENTS timer)
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
             kr_planning_rviz_plugins
             kr_planning_msgs
             traj_opt_ros
             dynamic_reconfigure
             motion_primitive_library)

generate_dynamic_reconfigure_options(cfg/ActionPlanner.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure kr_planning_msgs)

add_library(${PROJECT_NAME} src/data_conversions.cpp src/primitive_ros_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC src ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(global_plan_server src/global_plan_server.cpp)
target_link_libraries(global_plan_server PUBLIC ${PROJECT_NAME} Boost::timer)
add_dependencies(global_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(local_plan_server src/local_plan_server.cpp)
target_link_libraries(local_plan_server PUBLIC ${PROJECT_NAME} Boost::timer)
add_dependencies(local_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

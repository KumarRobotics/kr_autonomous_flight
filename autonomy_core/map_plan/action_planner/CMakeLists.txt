cmake_minimum_required(VERSION 3.10)
project(action_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
             kr_ilqr_optimizer
             traj_opt_ros
             dynamic_reconfigure
             motion_primitive_library
             opt_planner
             decomp_ros_utils
             decomp_ros_msgs
             motion_primitives
             gcopter
            )

generate_dynamic_reconfigure_options(cfg/ActionPlanner.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure kr_planning_msgs gcopter kr_ilqr_optimizer)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(planners src/planner_details.cpp src/data_conversions.cpp src/primitive_ros_utils.cpp)
target_link_libraries(planners PUBLIC ${catkin_LIBRARIES} )

add_executable(global_plan_server src/global_plan_server.cpp)
target_link_libraries(global_plan_server PUBLIC planners Boost::timer )
add_dependencies(global_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(local_plan_server src/local_plan_server.cpp )
target_link_libraries(local_plan_server PUBLIC Boost::timer planners)
add_dependencies(local_plan_server ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

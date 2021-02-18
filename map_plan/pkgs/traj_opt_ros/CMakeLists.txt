cmake_minimum_required(VERSION 3.10)
project(traj_opt_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILD_HOPF=Off")

find_package(catkin REQUIRED COMPONENTS roscpp traj_opt_basic traj_opt_msgs)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  traj_opt_basic
  traj_opt_msgs)

add_library(${PROJECT_NAME} src/ros_bridge.cpp)
# src/trajectory_visual.cpp src/trajectory_display.cpp ${MOC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
# ${QT_QTCORE_INCLUDE_DIR} ${QT_QTGUI_INCLUDE_DIR}
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
# ${QT_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES}

# Python stuff find_package(PythonLibs 2.7) # sorry kartik, python 3.0 doesn't
# work with the # external library if(${PYTHONLIBS_FOUND})
# include_directories(${PYTHON_INCLUDE_DIRS}) cs_add_library(traj_plot
# src/traj_plot.cpp) target_link_libraries(traj_plot ${PYTHON_LIBRARIES} X11)

# cs_add_executable(traj_ros_debug src/trajectory_debug.cpp)
# target_link_libraries(traj_ros_debug traj_plot traj_ros_bridge) endif()

# add_executable(quad_sim src/quad_sim.cpp) target_link_libraries(quad_sim
# traj_ros_bridge)

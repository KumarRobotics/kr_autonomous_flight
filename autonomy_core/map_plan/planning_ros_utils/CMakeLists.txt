cmake_minimum_required(VERSION 3.10)
project(planning_ros_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  catkin REQUIRED
  COMPONENTS rviz
             roscpp
             pcl_ros
             eigen_conversions
             planning_ros_msgs
             tf_conversions
             motion_primitive_library)

find_package(Eigen3 REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_AUTOMOC ON)
# This setting causes Qt's "MOC" generation to happen automatically. this does
# not moc things in include!!!!!!! only in src

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} planning_rviz_plugins
  CATKIN_DEPENDS planning_ros_msgs motion_primitive_library)

add_library(${PROJECT_NAME} src/data_ros_utils.cpp src/primitive_ros_utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS}
                                                  include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

set(SOURCE_FILES
    src/planning_rviz_plugins/bound_visual.cpp
    src/planning_rviz_plugins/mesh_visual.cpp
    src/planning_rviz_plugins/primitive_visual.cpp
    src/planning_rviz_plugins/trajectory_visual.cpp
    src/planning_rviz_plugins/path_visual.cpp
    src/planning_rviz_plugins/map_display.cpp
    src/planning_rviz_plugins/path_display.cpp
    src/planning_rviz_plugins/path_array_display.cpp
    src/planning_rviz_plugins/primitive_array_display.cpp
    src/planning_rviz_plugins/trajectory_display.cpp
    src/planning_rviz_plugins/spline_trajectory_display.cpp
    src/planning_rviz_plugins/spline_trajectory_visual.cpp
    ${MOC_FILES})

add_library(planning_rviz_plugins ${SOURCE_FILES})
target_include_directories(planning_rviz_plugins PUBLIC include
                                                        ${catkin_INCLUDE_DIRS})
target_link_libraries(
  planning_rviz_plugins
  PUBLIC ${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES}
         ${rviz_DEFAULT_PLUGIN_LIBRARIES})

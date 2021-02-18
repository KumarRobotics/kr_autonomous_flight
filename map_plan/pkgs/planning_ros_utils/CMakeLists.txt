cmake_minimum_required(VERSION 3.10)
project(planning_ros_utils)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  catkin REQUIRED
  COMPONENTS rviz
             roscpp
             rviz
             pcl_ros
             eigen_conversions
             planning_ros_msgs
             tf_conversions
             motion_primitive_library)

find_package(Eigen3 REQUIRED)
find_package(VTK REQUIRED)
find_package(SDL REQUIRED)
find_package(SDL_image REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

set(CMAKE_AUTOMOC ON)
# This setting causes Qt's "MOC" generation to happen automatically. this does
# not moc things in include!!!!!!! only in src

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

include(${VTK_USE_FILE})
# add_executable(mesh_sampling src/mapping_utils/mesh_sampling.cpp)
# target_link_libraries(mesh_sampling PUBLIC ${catkin_LIBRARIES})

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  planning_rviz_plugins
  voxel_grid
  CATKIN_DEPENDS
  planning_ros_msgs
  motion_primitive_library)

add_library(voxel_grid src/mapping_utils/voxel_grid.cpp)
target_include_directories(voxel_grid PUBLIC ${catkin_INCLUDE_DIRS} include)
target_link_libraries(voxel_grid PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(cloud_to_map src/mapping_utils/cloud_to_map.cpp)
target_link_libraries(cloud_to_map PUBLIC voxel_grid ${catkin_LIBRARIES})

# add_executable(trajectory_extractor_node
# src/planning_utils/trajectory_extractor_node.cpp)
# target_include_directories(trajectory_extractor_node PUBLIC
# ${catkin_INCLUDE_DIRS}) target_link_libraries(trajectory_extractor_node PUBLIC
# ${catkin_LIBRARIES})

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
    ${MOC_FILES})

add_library(planning_rviz_plugins ${SOURCE_FILES})
target_include_directories(planning_rviz_plugins PUBLIC include
                                                        ${catkin_INCLUDE_DIRS})
target_link_libraries(
  planning_rviz_plugins PUBLIC ${QT_LIBRARIES} ${catkin_LIBRARIES}
                               ${rviz_DEFAULT_PLUGIN_LIBRARIES})

cmake_minimum_required(VERSION 3.10)
project(decomp_ros_utils)

set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS rviz roscpp decomp_util decomp_ros_msgs)
find_package(Eigen3 REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

# This setting causes Qt's "MOC" generation to happen automatically. this does
# not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS decomp_util decomp_ros_msgs)

set(SOURCE_FILES
    src/bound_visual.cpp
    src/mesh_visual.cpp
    src/vector_visual.cpp
    src/ellipsoid_array_visual.cpp
    src/ellipsoid_array_display.cpp
    src/polyhedron_array_display.cpp
    ${MOC_FILES})

add_library(decomp_rviz_plugins ${SOURCE_FILES})
target_include_directories(decomp_rviz_plugins PUBLIC ${catkin_INCLUDE_DIRS}
                                                      include)
target_link_libraries(
  decomp_rviz_plugins
  PUBLIC ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES} Eigen3::Eigen
         Qt5::Core Qt5::Widgets)

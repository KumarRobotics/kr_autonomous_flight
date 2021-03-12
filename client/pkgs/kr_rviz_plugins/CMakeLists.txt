cmake_minimum_required(VERSION 3.10)
project(kr_rviz_plugins)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS rviz rosbag planning_ros_msgs)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# This plugin includes Qt widgets, so we must include Qt. We'll use the version
# that rviz used so they are compatible.
message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Widgets)
# make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)

# I prefer the Qt signals and slots to avoid defining "emit", "slots", etc
# because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frounding-math -Werror=return-type")

add_subdirectory(plugin_waypoint_navigation)

# Install rules
install(FILES waypoint_nav_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

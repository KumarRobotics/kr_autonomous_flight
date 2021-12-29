cmake_minimum_required(VERSION 3.10)
project(mavros_interface)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs
                                        mavros_msgs kr_mav_msgs nodelet)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  nav_msgs
  geometry_msgs
  kr_mav_msgs
  mavros_msgs
  nodelet)

add_library(so3cmd_to_mavros_nodelet src/so3cmd_to_mavros_nodelet.cpp)
target_include_directories(so3cmd_to_mavros_nodelet
                           PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(so3cmd_to_mavros_nodelet PUBLIC ${catkin_LIBRARIES}
                                                      Eigen3::Eigen)

install(
  TARGETS so3cmd_to_mavros_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

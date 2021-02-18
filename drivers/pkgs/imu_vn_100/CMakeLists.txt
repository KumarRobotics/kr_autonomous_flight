cmake_minimum_required(VERSION 3.10)
project(imu_vn_100)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs diagnostic_updater)

catkin_package(
  INCLUDE_DIRS
  include
  vncpplib/include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  diagnostic_updater
  roscpp
  sensor_msgs
  DEPENDS)

add_library(
  ${PROJECT_NAME}
  vncpplib/src/arch/linux/vncp_services.c vncpplib/src/vndevice.c
  vncpplib/src/vn100.c src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include vncpplib/include
                                                  ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE vncpplib/include/vncpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_cont.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h**")

install(DIRECTORY vncpplib/include/vncpp
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

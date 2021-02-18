cmake_minimum_required(VERSION 3.10)
project(flea3)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet camera_base
                                        dynamic_reconfigure message_generation)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)

find_package(FlyCapture2 REQUIRED)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(cfg/Flea3Dyn.cfg)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  nodelet
  camera_base
  dynamic_reconfigure
  message_runtime # std_msgs
  DEPENDS
  FlyCapture2)

include_directories(include ${catkin_INCLUDE_DIRS} ${FlyCapture2_INCLUDE_DIRS})

add_library(
  ${PROJECT_NAME}
  src/flea3_camera.cpp src/flea3_setting.cpp src/flea3_ros.cpp
  src/single/single_node.cpp src/stereo/stereo_node.cpp
  src/single/single_nodelet.cpp)
target_include_directories(
  ${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS}
                         ${FlyCapture2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES}
                                             ${FlyCapture2_LIBRARIES})

add_executable(${PROJECT_NAME}_single_node src/single/single_main.cpp)
target_link_libraries(${PROJECT_NAME}_single_node PUBLIC ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_stereo_node src/stereo/stereo_main.cpp)
target_link_libraries(${PROJECT_NAME}_stereo_node PUBLIC ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_single_node
          ${PROJECT_NAME}_stereo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

install(
  DIRECTORY flycapture/include
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

install(DIRECTORY flycapture/lib/ DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.*~" EXCLUDE)

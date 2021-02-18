cmake_minimum_required(VERSION 3.10)
project(exposure_control)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS roscpp nodelet sensor_msgs
                                        dynamic_reconfigure camera_base flea3)

generate_dynamic_reconfigure_options(cfg/ExposureControlDyn.cfg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/ExposureControl.cpp src/ExposureController.cpp
                            src/MeasureSensor.cpp src/DefaultController.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# nodelet
add_library(${PROJECT_NAME}_nodelet src/${PROJECT_NAME}_nodelet.cpp)
add_dependencies(${PROJECT_NAME}_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_nodelet ${PROJECT_NAME}
                      ${catkin_LIBRARIES})

# build the test node as well add_executable(cam_test_node src/cam_test_node.cpp
# src/CamTest.cpp) target_link_libraries(cam_test_node ${PROJECT_NAME}
# ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

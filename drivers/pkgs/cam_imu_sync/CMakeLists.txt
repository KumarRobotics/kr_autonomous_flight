cmake_minimum_required(VERSION 3.10)
project(cam_imu_sync)

find_package(catkin REQUIRED COMPONENTS camera_base flea3 imu_vn_100 roscpp
                                        nodelet sensor_msgs)

catkin_package()

add_library(${PROJECT_NAME} src/CamImuSynchronizer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/cam_imu_sync_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})

add_library(${PROJECT_NAME}_nodelet src/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet PRIVATE ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.10)
project(mapper)

set(CMAKE_CXX_STANDARD 17)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             pcl_ros
             image_geometry
             planning_ros_msgs
             planning_ros_utils
             decomp_util
             decomp_ros_utils
             decomp_ros_msgs
             depth_image_proc
             eigen_conversions
             jps3d
             tf_conversions)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME} src/tf_listener.cpp src/data_conversions.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS}
                                                  include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(cloud_to_map src/cloud_to_map.cpp src/voxel_mapper.cpp)
target_link_libraries(cloud_to_map PUBLIC ${PROJECT_NAME})

add_executable(local_global_mapper src/local_global_mapper.cpp
                                   src/voxel_mapper.cpp)
target_link_libraries(local_global_mapper PUBLIC ${PROJECT_NAME})

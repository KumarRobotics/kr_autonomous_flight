cmake_minimum_required(VERSION 3.10)
project(fake_lidar)

set(CMAKE_CXX_STANDARD 17)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             sensor_msgs
             image_geometry
             image_transport
             message_filters
             cv_bridge
             tf2_eigen
             pcl_ros
             tf2_ros)

find_package(Boost REQUIRED COMPONENTS timer)
find_package(TBB REQUIRED)

catkin_package()

add_executable(fake_lidar_node src/fake_lidar_node.cpp)
target_include_directories(fake_lidar_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(fake_lidar_node PRIVATE ${catkin_LIBRARIES} Boost::timer)

add_executable(fake_lidar_node2 src/fake_lidar_node2.cpp)
target_include_directories(fake_lidar_node2 PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(fake_lidar_node2 PRIVATE ${catkin_LIBRARIES} Boost::timer
                                               TBB::tbb)

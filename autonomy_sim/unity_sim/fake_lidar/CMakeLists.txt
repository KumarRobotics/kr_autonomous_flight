cmake_minimum_required(VERSION 3.16)
project(fake_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(TBB REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

set(FAKE_LIDAR_DEPS
  rclcpp
  sensor_msgs
  geometry_msgs
  image_transport
  image_geometry
  cv_bridge
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  message_filters)

add_executable(fake_lidar_node src/fake_lidar_node.cpp)
target_include_directories(fake_lidar_node PRIVATE src)
ament_target_dependencies(fake_lidar_node ${FAKE_LIDAR_DEPS})
target_link_libraries(fake_lidar_node
  Eigen3::Eigen
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Boost::timer)

add_executable(fake_lidar_node2 src/fake_lidar_node2.cpp)
target_include_directories(fake_lidar_node2 PRIVATE src)
ament_target_dependencies(fake_lidar_node2 ${FAKE_LIDAR_DEPS})
target_link_libraries(fake_lidar_node2
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::timer
  TBB::tbb)

install(TARGETS fake_lidar_node fake_lidar_node2
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

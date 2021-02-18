cmake_minimum_required(VERSION 3.10)
project(dcist_demo)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp arl_unity_ros geometry_msgs
                                        tf2_eigen tf2_ros mav_high_level_msgs)

catkin_package()

add_executable(car_det_node src/car_det_node.cpp)
target_include_directories(car_det_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(car_det_node PUBLIC ${catkin_LIBRARIES})

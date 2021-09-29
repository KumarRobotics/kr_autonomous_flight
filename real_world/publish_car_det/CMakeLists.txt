cmake_minimum_required(VERSION 3.10)
project(publish_car_det)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

add_executable(pub_car_det_node src/pub_car_det_node.cpp)
target_include_directories(pub_car_det_node PRIVATE src ${catkin_INCLUDE_DIRS})
target_link_libraries(pub_car_det_node PRIVATE ${catkin_LIBRARIES})

cmake_minimum_required(VERSION 3.10)
project(jps3d)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME} src/graph_search.cpp src/jps_planner.cpp
                            src/map_util.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

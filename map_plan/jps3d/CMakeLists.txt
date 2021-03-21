cmake_minimum_required(VERSION 3.10)
project(jps3d)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME} src/graph_search.cpp src/jps_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${YAMLCPP_LIBRARIES})

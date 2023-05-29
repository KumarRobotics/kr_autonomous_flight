cmake_minimum_required(VERSION 3.10)
project(jps3d)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
            COMPONENTS kr_planning_msgs)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS kr_planning_msgs)

add_library(${PROJECT_NAME} src/graph_search.cpp src/jps_planner.cpp
                            src/map_util.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen Boost::boost)

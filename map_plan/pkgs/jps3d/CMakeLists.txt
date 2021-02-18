cmake_minimum_required(VERSION 2.8)
project(jps3d)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(INCLUDE_DIRS include LIBRARIES jps_lib dmp_lib)

add_library(jps_lib src/jps_planner/graph_search.cpp
                    src/jps_planner/jps_planner.cpp)
target_include_directories(jps_lib PUBLIC include ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(jps_lib PUBLIC Eigen3::Eigen ${YAMLCPP_LIBRARIES})

add_library(dmp_lib src/distance_map_planner/graph_search.cpp
                    src/distance_map_planner/distance_map_planner.cpp)
target_include_directories(dmp_lib PUBLIC include ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(dmp_lib PUBLIC Eigen3::Eigen ${YAMLCPP_LIBRARIES})

# add_executable(create_map test/create_map.cpp)
# target_link_libraries(create_map ${YAMLCPP_LIBRARIES})

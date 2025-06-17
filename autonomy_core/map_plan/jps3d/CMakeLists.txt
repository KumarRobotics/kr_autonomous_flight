cmake_minimum_required(VERSION 3.10)
project(jps3d)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake)
find_package(kr_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_library(${PROJECT_NAME} src/graph_search.cpp src/jps_planner.cpp
                            src/map_util.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen Boost::boost)

ament_target_dependencies(${PROJECT_NAME} kr_planning_msgs)


# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Export targets and declare package
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  kr_planning_msgs
  Eigen3
  Boost
)
ament_package()
cmake_minimum_required(VERSION 3.16)
project(jps3d)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/graph_search.cpp
  src/jps_planner.cpp
  src/map_util.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} kr_planning_msgs)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen Boost::boost)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(kr_planning_msgs Eigen3 Boost)

ament_package()

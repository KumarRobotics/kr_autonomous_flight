cmake_minimum_required(VERSION 3.16)
project(coverage_utils)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kr_planning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/coverage_utils.cpp
  src/utils.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  kr_planning_msgs
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

add_executable(coverage_utils_node src/coverage_utils_node.cpp)
target_link_libraries(coverage_utils_node ${PROJECT_NAME})
ament_target_dependencies(coverage_utils_node
  rclcpp
  std_msgs
  geometry_msgs
  kr_planning_msgs
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS coverage_utils_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs geometry_msgs kr_planning_msgs Eigen3)

ament_package()

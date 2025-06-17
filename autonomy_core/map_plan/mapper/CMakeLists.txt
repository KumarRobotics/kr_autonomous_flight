# cmake_minimum_required(VERSION 3.10)
# project(mapper)

# set(CMAKE_CXX_STANDARD 17)
# find_package(
#   catkin REQUIRED
#   COMPONENTS roscpp
#              pcl_ros
#              image_geometry
#              kr_planning_msgs
#              kr_planning_rviz_plugins
#              depth_image_proc
#              sensor_msgs
#              eigen_conversions
#              tf_conversions
#              motion_primitive_library)
             
# find_package(Boost REQUIRED COMPONENTS timer)

# catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

# include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include)

# add_library(${PROJECT_NAME} src/tf_listener.cpp src/voxel_mapper.cpp)
# target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

# add_executable(local_global_mapper src/local_global_mapper.cpp)
# target_link_libraries(local_global_mapper PUBLIC ${PROJECT_NAME} Boost::timer)


# #----------------------------------------------------------------------------------
# # Add testing with GoogleTest

# # To run test:
# #   catkin test mapper
# enable_testing()

# catkin_add_gtest(test_voxel_mapper test/test_voxel_mapper.cpp)
# target_link_libraries(test_voxel_mapper ${catkin_LIBRARIES} ${PROJECT_NAME})

#----------------------------------------------------------------------------------
# Add microbenchmarking with Google Benchmark

# To run benchmark:
#   rosrun mapper benchmarking
# if you want to save the results to a file: 
#   rosrun mapper benchmarking --benchmark_out=file_name.json
# if you want to compare two benchmark files, use the compare.py script located
# in the build directory under mapper/_deps/benchmark-src/tools/ (you will need
# to have pandas installed). Also changed the shebang to use python3
# e.g. from the catkin root directory run:
#   ./build/mapper/_deps/benchmark-src/tools/compare.py benchmarks file1.json file2.json 

# include(FetchContent)
# FetchContent_Declare(
#     benchmark
#     URL https://github.com/google/benchmark/archive/b7afda2cd2d81230737caa1073e160b6406798d7.zip
# )
# FetchContent_MakeAvailable(benchmark)

# add_executable(benchmarking test/benchmark_voxel_mapper.cpp)
# target_link_libraries(benchmarking benchmark::benchmark ${PROJECT_NAME})


cmake_minimum_required(VERSION 3.10)
project(mapper)

# Use C++17
set(CMAKE_CXX_STANDARD 17)

# -----------------------------------------------------------------------------
# 1) ament & ROS 2 packages instead of catkin
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(pcl_ros            REQUIRED)
find_package(image_geometry     REQUIRED)
find_package(kr_planning_msgs   REQUIRED)
find_package(kr_planning_rviz_plugins REQUIRED)
find_package(depth_image_proc   REQUIRED)
find_package(sensor_msgs        REQUIRED)
# find_package(eigen_conversions  REQUIRED) # Eigen conversions for ROS 2 is not supported yet
find_package(tf2_eigen          REQUIRED)
find_package(motion_primitive_library REQUIRED)
find_package(Boost              REQUIRED COMPONENTS timer)

# -----------------------------------------------------------------------------
# 2) Build your library
add_library(${PROJECT_NAME}
  src/tf_listener.cpp
  src/voxel_mapper.cpp
)
# Declare dependencies for include dirs and link targets
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pcl_ros
  image_geometry
  kr_planning_msgs
  kr_planning_rviz_plugins
  depth_image_proc
  sensor_msgs
  # eigen_conversions
  tf2_eigen
  motion_primitive_library
)
# Link the Boost::timer target
target_link_libraries(${PROJECT_NAME} Boost::timer)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# -----------------------------------------------------------------------------
# 3) Build your node
add_executable(local_global_mapper src/local_global_mapper.cpp)
target_link_libraries(local_global_mapper PUBLIC ${PROJECT_NAME} Boost::timer)
target_include_directories(local_global_mapper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# -----------------------------------------------------------------------------
# 4) Testing with ament_cmake_gtest
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   ament_add_gtest(test_voxel_mapper test/test_voxel_mapper.cpp)
#   if(TARGET test_voxel_mapper)
#     ament_target_dependencies(test_voxel_mapper
#       rclcpp
#       pcl_ros
#       image_geometry
#       kr_planning_msgs
#       kr_planning_rviz_plugins
#       depth_image_proc
#       sensor_msgs
#       # eigen_conversions
#       tf2_eigen
#       motion_primitive_library
#     )
#     target_link_libraries(test_voxel_mapper PUBLIC ${PROJECT_NAME} Boost::timer)
#   endif()
# endif()

# -----------------------------------------------------------------------------
# 5) Install rules
install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS
    local_global_mapper
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# 6) Export & finalize
ament_export_dependencies(
  rclcpp
  pcl_ros
  image_geometry
  kr_planning_msgs
  kr_planning_rviz_plugins
  depth_image_proc
  sensor_msgs
  # eigen_conversions
  tf2_eigen
  motion_primitive_library
  Boost
)
ament_package()

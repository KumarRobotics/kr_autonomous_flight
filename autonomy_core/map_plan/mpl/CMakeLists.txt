cmake_minimum_required(VERSION 3.10)
project(motion_primitive_library)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED
              COMPONENTS kr_planning_msgs)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS kr_planning_msgs
  DEPENDS EIGEN3)

add_library(
  ${PROJECT_NAME}
  src/math.cpp
  src/lambda.cpp
  src/primitive.cpp
  src/trajectory.cpp
  src/waypoint.cpp
  src/map_util.cpp
  src/env_base.cpp
  src/env_map.cpp
  src/map_planner.cpp
  src/planner_base.cpp
  src/state_space.cpp
  src/graph_search.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen Boost::boost)

#----------------------------------------------------------------------------------
# Add testing with GoogleTest

# To run test:
#   catkin test motion_primitive_library
enable_testing()

catkin_add_gtest(test_map_util test/test_map_util.cpp)
target_link_libraries(test_map_util ${catkin_LIBRARIES} ${PROJECT_NAME})

#----------------------------------------------------------------------------------
# Add microbenchmarking with Google Benchmark

# To run benchmark:
#   rosrun motion_primitive_library map_util_benchmarking
# if you want to save the results to a file:
#   rosrun motion_primitive_library map_util benchmarking --benchmark_out=file_name.json
# if you want to compare two benchmark files, use the compare.py script located
# in the build directory under motion_primitive_library/_deps/benchmark-src/tools/
# (you will need to have pandas installed). Also changed the shebang to use python3
# e.g. from the catkin root directory run:
#   ./build/motion_primitive_library/_deps/benchmark-src/tools/compare.py benchmarks file1.json file2.json


# include(FetchContent)
# FetchContent_Declare(
#     benchmark
#     URL https://github.com/google/benchmark/archive/b7afda2cd2d81230737caa1073e160b6406798d7.zip
# )
# FetchContent_MakeAvailable(benchmark)

# add_executable(map_util_benchmarking test/benchmark_map_util.cpp)
# target_link_libraries(map_util_benchmarking benchmark::benchmark ${PROJECT_NAME})
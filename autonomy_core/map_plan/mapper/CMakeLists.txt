cmake_minimum_required(VERSION 3.10)
project(mapper)

set(CMAKE_CXX_STANDARD 17)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             pcl_ros
             image_geometry
             planning_ros_msgs
             planning_ros_utils
             depth_image_proc
             sensor_msgs
             eigen_conversions
             tf_conversions)
find_package(Boost REQUIRED COMPONENTS timer)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS} include)

add_library(${PROJECT_NAME} src/tf_listener.cpp src/voxel_mapper.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

add_executable(local_global_mapper src/local_global_mapper.cpp)
target_link_libraries(local_global_mapper PUBLIC ${PROJECT_NAME} Boost::timer)


#----------------------------------------------------------------------------------
# Add testing with GoogleTest
# Should update the zip file to the most recent Git commit hash of the GoogleTest version
#include(FetchContent)
#FetchContent_Declare(
#    googletest
#    URL https://github.com/google/googletest/archive/8d51dc50eb7e7698427fed81b85edad0e032112e.zip
#)
#
## For Windows: Prevent overriding the parent project's compiler/linker settings
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
#FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
    test_voxel_mapper
    src/test_voxel_mapper.cpp
)

target_link_libraries(
    test_voxel_mapper PUBLIC
    ${PROJECT_NAME}
    gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_voxel_mapper)



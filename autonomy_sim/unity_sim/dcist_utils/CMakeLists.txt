cmake_minimum_required(VERSION 3.10)
project(dcist_utils)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs tf2_ros)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

catkin_package()

add_executable(odom2tf src/odom2tf.cpp)
target_include_directories(odom2tf PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(odom2tf PRIVATE ${catkin_LIBRARIES} Eigen3::Eigen)

add_executable(msckf_calib_gen src/msckf_calib_gen.cpp)
target_include_directories(msckf_calib_gen PRIVATE ${YAML_CPP_INCLUDE_DIR}
                                                   ${catkin_INCLUDE_DIRS})
target_link_libraries(
  msckf_calib_gen PRIVATE ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES}
                          Eigen3::Eigen)

add_executable(playground src/playground.cpp)
target_include_directories(playground PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(playground PRIVATE ${catkin_LIBRARIES})

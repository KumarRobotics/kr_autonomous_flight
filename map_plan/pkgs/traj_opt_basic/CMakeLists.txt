cmake_minimum_required(VERSION 3.10)
project(traj_opt_basic)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} DEPENDS EIGEN3)

add_library(${PROJECT_NAME} src/msg_traj.cpp src/polynomial_basis.cpp
                            src/trajectory.cpp src/hopf_helper.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} Eigen3::Eigen)

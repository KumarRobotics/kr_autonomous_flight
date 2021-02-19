cmake_minimum_required(VERSION 2.8.3)
project(traj_opt_quadrotor)

set(CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED COMPONENTS traj_opt_basic kr_mav_msgs)

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

add_library(${PROJECT_NAME} src/convert.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

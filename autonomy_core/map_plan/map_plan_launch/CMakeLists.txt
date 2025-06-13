cmake_minimum_required(VERSION 3.10)
project(map_plan_launch)

# find_package(catkin REQUIRED)
# catkin_package()
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

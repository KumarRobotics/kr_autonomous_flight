cmake_minimum_required(VERSION 3.10)
project(rqt_quadrotor_safety)

find_package(catkin REQUIRED)
catkin_python_setup()

catkin_package()

install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY resource DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/rqt_quadrotor_safety
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

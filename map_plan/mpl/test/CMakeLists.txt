include(CTest)
find_package(OpenCV REQUIRED)

add_executable(test_traj_solver test_traj_solver.cpp)
target_link_libraries(test_traj_solver poly_solver ${OpenCV_LIBS})

add_test(test_traj_solver test_traj_solver)
add_executable(test_planner_2d test_planner_2d.cpp)
target_link_libraries(test_planner_2d map_planner ${YAMLCPP_LIBRARIES}
                      ${OpenCV_LIBS})
add_test(test_planner_2d test_planner_2d ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_with_prior_traj
               test_planner_2d_with_prior_traj.cpp)
target_link_libraries(test_planner_2d_with_prior_traj map_planner
                      ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS})
add_test(test_planner_2d_with_prior_traj test_planner_2d_with_prior_traj
         ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_planner_2d_with_yaw test_planner_2d_with_yaw.cpp)
target_link_libraries(test_planner_2d_with_yaw map_planner ${YAMLCPP_LIBRARIES}
                      ${OpenCV_LIBS})
add_test(test_planner_2d_with_yaw test_planner_2d_with_yaw
         ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d test_distance_map_planner_2d.cpp)
target_link_libraries(test_distance_map_planner_2d map_planner
                      ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS})
add_test(test_distance_map_planner_2d test_distance_map_planner_2d
         ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d_with_yaw
               test_distance_map_planner_2d_with_yaw.cpp)
target_link_libraries(test_distance_map_planner_2d_with_yaw map_planner
                      ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS})
add_test(test_distance_map_planner_2d_with_yaw
         test_distance_map_planner_2d_with_yaw
         ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

add_executable(test_distance_map_planner_2d_iterative
               test_distance_map_planner_2d_iterative.cpp)
target_link_libraries(test_distance_map_planner_2d_iterative map_planner
                      ${YAMLCPP_LIBRARIES} ${OpenCV_LIBS})
add_test(test_distance_map_planner_2d_iterative
         test_distance_map_planner_2d_iterative
         ${CMAKE_SOURCE_DIR}/data/corridor.yaml)

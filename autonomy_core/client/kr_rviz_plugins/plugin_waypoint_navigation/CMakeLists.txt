# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
qt5_wrap_cpp(MOC_FILES waypoint_nav_frame.h waypoint_nav_tool.h)

# Convert the Qt Signals and Slots for QWidget events
qt5_wrap_ui(UIC_FILES ui/WaypointNavigation.ui)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Plugin Source
set(SOURCE_FILES waypoint_nav_frame.cpp waypoint_nav_tool.cpp ${MOC_FILES})

set(KR_LIB_NAME kr_waypoint_nav_plugin)
add_library(${KR_LIB_NAME} ${SOURCE_FILES} ${MOC_SOURCES} ${UIC_FILES})
target_link_libraries(${KR_LIB_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${YAML_CPP_LIBRARIES})

# Install rules

install(
  DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h")

install(
  TARGETS ${KR_LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

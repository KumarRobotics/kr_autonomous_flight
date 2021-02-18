cmake_minimum_required(VERSION 2.8.3)
project(rosflight_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  OutputRaw.msg
  RCRaw.msg
  Command.msg
  Barometer.msg
  Airspeed.msg
  Attitude.msg
  Status.msg
  AuxCommand.msg
  Error.msg
  GNSS.msg
  GNSSRaw.msg
  BatteryStatus.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ParamFile.srv
  ParamGet.srv
  ParamSet.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs std_msgs
)

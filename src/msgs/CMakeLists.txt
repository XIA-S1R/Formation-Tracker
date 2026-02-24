cmake_minimum_required(VERSION 3.0.2)
project(tracking_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  nav_msgs
  geometry_msgs
)

add_message_files(
  FILES
  TargetObservation.msg
  TargetState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs nav_msgs geometry_msgs
)
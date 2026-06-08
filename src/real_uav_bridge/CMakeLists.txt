cmake_minimum_required(VERSION 3.0.2)
project(real_uav_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  quadrotor_msgs
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED COMPONENTS io)

catkin_package(
  CATKIN_DEPENDS rospy roscpp geometry_msgs nav_msgs sensor_msgs quadrotor_msgs pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(cmd_odom_ground_bridge
  src/cmd_odom_ground_bridge.cpp
)
add_dependencies(cmd_odom_ground_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_odom_ground_bridge ${catkin_LIBRARIES})

add_executable(cmd_odom_onboard_bridge
  src/cmd_odom_onboard_bridge.cpp
)
add_dependencies(cmd_odom_onboard_bridge ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_odom_onboard_bridge ${catkin_LIBRARIES})

add_executable(trigger_all_ready_node
  src/trigger_all_ready_node.cpp
)
add_dependencies(trigger_all_ready_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(trigger_all_ready_node ${catkin_LIBRARIES})

add_executable(static_pointcloud_map_publisher
  src/static_pointcloud_map_publisher.cpp
)
add_dependencies(static_pointcloud_map_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_pointcloud_map_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/mocap_udp_odom_receiver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

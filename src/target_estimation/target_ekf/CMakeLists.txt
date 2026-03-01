cmake_minimum_required(VERSION 2.8.3)
project(target_ekf)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tracking_msgs
  object_detection_msgs
  quadrotor_msgs
  message_filters
  pcl_conversions
  pcl_ros
  Eigen3 REQUIRED
)

catkin_package(
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS object_detection_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(target_ekf_node
 src/target_ekf_node.cpp
)

target_link_libraries(target_ekf_node
   ${catkin_LIBRARIES}
)

add_dependencies(target_ekf_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(target_ekf_sim_node
 src/target_ekf_sim_node.cpp
)

target_link_libraries(target_ekf_sim_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_dependencies(target_ekf_sim_node
  ${catkin_EXPORTED_TARGETS}
)
cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_sim)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(quadrotor_sim_node src/quadrotor_sim_node.cpp)
target_link_libraries(quadrotor_sim_node ${catkin_LIBRARIES})
cmake_minimum_required(VERSION 3.0.2)
project(spot_ros_srvs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  diagnostic_msgs
  spot_ros_msgs
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Stand.srv
  Trajectory.srv
  Velocity.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  diagnostic_msgs
  spot_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs spot_ros_msgs
)
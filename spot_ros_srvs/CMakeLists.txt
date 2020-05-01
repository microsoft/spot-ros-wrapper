cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_srvs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
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
  spot_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
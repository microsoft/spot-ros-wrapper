cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  KinematicState.msg
  ImageCapture.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
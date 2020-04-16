cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs genmsg)

## Generate added messages and services (if custom msgs are used)
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build
## include_directories(include ${catkin_INCLUDE_DIRS})
cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg spot_ros_msgs cv_bridge)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_python_setup()
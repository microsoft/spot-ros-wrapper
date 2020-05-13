cmake_minimum_required(VERSION 2.8.3)
project(spot_ros_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs genmsg spot_ros_msgs)

## Declare a catkin package
catkin_package()

catkin_python_setup()

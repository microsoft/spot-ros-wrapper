cmake_minimum_required(VERSION 3.0.2)
project(spot_ros_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg spot_ros_msgs cv_bridge)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/grid_utils.py
                               scripts/keyboard_teleop.py
                               scripts/spot_ros_interface.py
                               scripts/tf_remapper.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
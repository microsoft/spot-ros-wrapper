#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import spot_ros_srvs.srv

import time
import math
import readchar
import sys
import os


line='\u2500'
instructions="\n\
\u250C{} SPOT KEYBOARD TELEOP {}\u2510 \n\
\u2502                            \u2502\n\
\u2502     wasd - Move            \u2502\n\
\u2502     qe - Turn              \u2502\n\
\u2502     r - Self-right         \u2502\n\
\u2502     j - Height down        \u2502\n\
\u2502     k - Height up          \u2502\n\
\u2502                            \u2502\n\
\u2502     SPACE - E-Stop (TODO)  \u2502\n\
\u2502     Q - Quit               \u2502\n\
\u2502                            \u2502\n\
\u2514{}\u2518\
".format(line*3,line*3,line*28)

# Get size of terminal window
rows, columns = os.popen('stty size', 'r').read().split()

# Define velocity at which Spot will move
lin_vel = 1.0 # m/s
ang_vel = 1.0
height_up = 1.0
height_down = -1.0

def self_right_service(key):
    tf = geometry_msgs.msg.Transform()

    self_right_srv_req.body_pose.translation = tf.translation
    self_right_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("self_right_cmd", timeout=2.0)
        self_right_srv_pub(self_right_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def stand_service(key):
    tf = geometry_msgs.msg.Transform()

    if key=='j':
        tf.translation.z = height_up
    elif key=='k':
        tf.translation.z = height_down

    stand_srv_req.body_pose.translation = tf.translation
    stand_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("stand_cmd", timeout=2.0)
        stand_srv_pub(stand_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def vel_service(key):
    twist = geometry_msgs.msg.Twist()

    if key=='w':
        twist.linear.x = lin_vel
    elif key=='a':
        twist.linear.y = lin_vel
    elif key=='s':
        twist.linear.x = -lin_vel
    elif key=='d':
        twist.linear.y = -lin_vel
    elif key=='q':
        twist.angular.z = ang_vel
    elif key=='e':
        twist.angular.z = -ang_vel

    vel_srv_req.velocity.linear = twist.linear
    vel_srv_req.velocity.angular = twist.angular

    try:
        rospy.wait_for_service("velocity_cmd", timeout=2.0)
        vel_srv_pub(vel_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e, end='')


rospy.init_node('keyboard_teleop')
rate = rospy.Rate(60)

# Define service proxies
self_right_srv_pub = rospy.ServiceProxy("self_right_cmd", spot_ros_srvs.srv.Stand)
stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
vel_srv_pub = rospy.ServiceProxy("velocity_cmd", spot_ros_srvs.srv.Velocity)

# Define service requests
self_right_srv_req = spot_ros_srvs.srv.StandRequest()
stand_srv_req = spot_ros_srvs.srv.StandRequest()
vel_srv_req = spot_ros_srvs.srv.VelocityRequest()

print(instructions)
while not rospy.is_shutdown():
    key = readchar.readkey()
    print('{}\rKey pressed: {}\r'.format(' '*int(columns), key), end="")

    if key=="Q":
        sys.exit()

    if key in 'wasdqe':
        vel_service(key)
    elif key in 'jk':
        stand_service(key)
    elif key in 'r':
        self_right_service(key)


    rate.sleep()
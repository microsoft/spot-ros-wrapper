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


line='\u2500'
instructions="\n\
\u250C{} SPOT KEYBOARD TELEOP {}\u2510 \n\
\u2502                            \u2502\n\
\u2502     wasd - Move            \u2502\n\
\u2502     qe - Turn              \u2502\n\
\u2502     WASD - Rotate in place \u2502\n\
\u2502     r - Self-right         \u2502\n\
\u2502     j - height down        \u2502\n\
\u2502     k - height up          \u2502\n\
\u2502                            \u2502\n\
\u2502     SPACE - E-Stop (TODO)  \u2502\n\
\u2502     Q - Quit               \u2502\n\
\u2502                            \u2502\n\
\u2514{}\u2518\
".format(line*3,line*3,line*28)


# Define velocity at which Spot will move
lin_vel = 1.0 # m/s
ang_vel = 1.0
height_up = 1.0
height_down = -1.0

def self_right_service(key):
    tf.translation.x = 0
    tf.translation.y = 0
    tf.translation.z = 0
    tf.rotation.x = 0
    tf.rotation.y = 0
    tf.rotation.z = 0
    tf.rotation.w = 1

    self_right_srv_req.body_pose.translation = tf.translation
    self_right_srv_req.body_pose.rotation = tf.rotation

    try:
        rospy.wait_for_service("self_right_cmd", timeout=2.0)
        self_right_srv_pub(self_right_srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def stand_service(key):
    tf.translation.x = 0
    tf.translation.y = 0
    tf.translation.z = 0
    tf.rotation.x = 0
    tf.rotation.y = 0
    tf.rotation.z = 0
    tf.rotation.w = 1

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
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

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

tf = geometry_msgs.msg.Transform()
twist = geometry_msgs.msg.Twist()

# start_time = time.time()


    # print(tf.translation.z)

    # time.sleep(1)


print(instructions)
while not rospy.is_shutdown():
    key = readchar.readkey()
    print('{}\rKey pressed: {}\r'.format(' '*45, key), end="")

    if key=="Q":
        sys.exit()
    # key = ''

    if key in 'wasdqe':
        vel_service(key)
    elif key in 'jk':
        stand_service(key)
    elif key in 'r':
        self_right_service(key)


    rate.sleep()
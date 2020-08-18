#!/usr/bin/env python3

'''
Test script that calls the stand service from the spot_ros_interface at 0.5 Hz.
Moves Spot in a sinusoidal shape up and down.

Note: spot_ros_interface must be running in order to accept service calls.
'''

import rospy
import geometry_msgs.msg
import spot_ros_srvs.srv

import time
import math

# Initialize ROS node
rospy.init_node('stand_srv_caller')
rate = rospy.Rate(2)

# Declare stnad service proxy
stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)

tf = geometry_msgs.msg.Transform()
srv_req = spot_ros_srvs.srv.StandRequest()


start_time = time.time()

while not rospy.is_shutdown():

    # Set target height to +/- 0.15 m from the resting height
    tf.translation.z = 0.15*math.sin((time.time()-start_time)%(2*math.pi))

    # Construct service request
    srv_req.body_pose = tf

    # Call service, if available
    try:
        rospy.wait_for_service("stand_cmd", timeout=0.5)
        stand_srv_pub(srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print("Height: {}".format(tf.translation.z))

    rate.sleep()
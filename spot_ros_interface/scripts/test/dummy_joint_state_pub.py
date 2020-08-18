#!/usr/bin/env python3

'''
Test script that publishes simulated joint angles to test the Rviz simulation.
To visualize, run `roslaunch spot_urdf rviz_display.launch` 
'''

import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import tf2_ros 
import math
import time

# Set up joint publisher
joint_state_pub = rospy.Publisher(
    "joint_state_from_spot", sensor_msgs.msg.JointState, queue_size=20)

# RViz visualization expects a transform from Spot's `base_link` to
# the world frame `vision_odometry_frame`
spot_tf_broadcaster = tf2_ros.TransformBroadcaster()

# Initialize ROS node
rospy.init_node('dummy_spot_ros_interface_py')
rate = rospy.Rate(10)  # Update at 10hz


start_time = time.time()
while not rospy.is_shutdown():
    print("Looping...")

    # Populate and broadcast transform from base_link to vision_odometry_frame
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "vision_odometry_frame"
    t.child_frame_id = "base_link"
    t.transform.rotation.w = 1
    spot_tf_broadcaster.sendTransform(t)

    # Populate joint state msg
    js = sensor_msgs.msg.JointState()
    js.name.append("fl.hx")
    js.name.append("fl.hy")
    js.name.append("fl.kn")
    js.name.append("fr.hx")
    js.name.append("fr.hy")
    js.name.append("fr.kn")
    js.name.append("hl.hx")
    js.name.append("hl.hy")
    js.name.append("hl.kn")
    js.name.append("hr.hx")
    js.name.append("hr.hy")
    js.name.append("hr.kn")

    # Animate joints in sinusoidal shape. Note: Not a real gait, just for testing the visualization
    js.position = [
        0.0 ,                                                                       #fl.hx
        0.5*math.sin((time.time()-start_time)%(2*math.pi) + math.pi)+math.pi/4,     #fl.hy
        -math.pi/3 + 0.5*math.sin((time.time()-start_time)%(2*math.pi)+math.pi),    #fl.kn
        0.0,                                                                        #fr.hx
        0 + 0.5*math.sin((time.time()-start_time)%(2*math.pi))+math.pi/4,           #fr.hy
        -math.pi/2. + math.pi/6+ 0.5*math.sin((time.time()-start_time)%(2*math.pi)),#fr.kn
        0.0,                                                                        #hl.hx
        0.5*math.sin((time.time()-start_time)%(2*math.pi))+math.pi/4,               #hl.hy
        -math.pi/3 + 0.5*math.sin((time.time()-start_time)%(2*math.pi)),            #hl.kn
        0.0,                                                                        #hr.hx
        0.5*math.sin((time.time()-start_time)%(2*math.pi) + math.pi)+math.pi/4,     #hr.hy
        -math.pi/3 + 0.5*math.sin((time.time()-start_time)%(2*math.pi)+math.pi)     #hr.kn
        ]
    
    # Publish joint state
    joint_state_pub.publish(js)

    rate.sleep()

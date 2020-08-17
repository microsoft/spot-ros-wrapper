import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import math
import time

'''
This script publishes simulated joint angles to test out the Rviz simulation
'''

joint_state_pub = rospy.Publisher(
    "joint_state_from_spot", sensor_msgs.msg.JointState, queue_size=20)

rospy.init_node('dummy_spot_ros_interface_py')
rate = rospy.Rate(10)  # Update at 10hz
start_time = time.time()
while not rospy.is_shutdown():
    print(time.time()-start_time)
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

    js.position = [
        0.0 ,
        0 + 0.5*math.cos((time.time()-start_time)%(2*math.pi)),
        -math.pi/2. + math.pi/6+ 0.5*math.cos((time.time()-start_time)%(2*math.pi)),
        0.0,
        0 + 0.5*math.sin((time.time()-start_time)%(2*math.pi)),
        -math.pi/2. + math.pi/6+ 0.5*math.sin((time.time()-start_time)%(2*math.pi)),
        0.0,
        0 + 0.5*math.sin((time.time()-start_time)%(2*math.pi)),
        -math.pi/2. + math.pi/6+ 0.5*math.sin((time.time()-start_time)%(2*math.pi)),
        0.0,
        0 + 0.5*math.cos((time.time()-start_time)%(2*math.pi)),
        -math.pi/2. + math.pi/6+ 0.5*math.cos((time.time()-start_time)%(2*math.pi))
        ]
    
    joint_state_pub.publish(js)

    rate.sleep()

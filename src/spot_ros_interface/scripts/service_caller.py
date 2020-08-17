#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import spot_ros_srvs.srv

import time
import math

rospy.init_node('service_caller_py')

stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
srv_req = spot_ros_srvs.srv.StandRequest()
tf = geometry_msgs.msg.Transform()


start_time = time.time()

while not rospy.is_shutdown():
    print("Looping...")
    tf.translation.x = 0
    tf.translation.y = 0
    tf.translation.z = math.sin(((time.time()-start_time)/2)%(2*math.pi))
    tf.rotation.x = 0
    tf.rotation.y = 0
    tf.rotation.z = 0
    tf.rotation.w = 1

    srv_req.body_pose.translation = tf.translation
    srv_req.body_pose.rotation = tf.rotation
    # rospy.wait_for_service('stand_cmd')

    try:
    #     stand_srv_pub = rospy.ServiceProxy("stand_cmd", spot_ros_srvs.srv.Stand)
        stand_srv_pub(srv_req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    print(tf.translation.z)

    time.sleep(1)
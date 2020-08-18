# import rospy
# import tf2_ros
# import geometry_msgs.msg

# unity_cam_broadcaster = tf2_ros.TransformBroadcaster()

# t = geometry_msgs.msg.TransformStamped()
# t.header.stamp = rospy.Time.now()
# t.header.frame_id = "vision_odometry_frame"
# t.child_frame_id = "base_link"
# t.transform.translation.x = kinematic_state.vision_tform_body.translation.x
# t.transform.translation.y = kinematic_state.vision_tform_body.translation.y
# t.transform.translation.z = kinematic_state.vision_tform_body.translation.z
# t.transform.rotation.x = kinematic_state.vision_tform_body.rotation.x
# t.transform.rotation.y = kinematic_state.vision_tform_body.rotation.y
# t.transform.rotation.z = kinematic_state.vision_tform_body.rotation.z
# t.transform.rotation.w = kinematic_state.vision_tform_body.rotation.w

# unity_cam_broadcaster.sendTransform(t)

import rospy
import math
import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

t = geometry_msgs.msg.TransformStamped()
t.header.frame_id = "7e62ccfd-eb9a-46f8-a221-acbbcb906838"
t.child_frame_id = "unity_cam"
t.transform.translation.x = 1
t.transform.translation.y = 1
t.transform.translation.z = 1
t.transform.rotation.x = 0
t.transform.rotation.y = 0
t.transform.rotation.z = 0
t.transform.rotation.w = 1

def callback(data):
    rospy.loginfo("UPDATING!")

    t.header.frame_id = "7e62ccfd-eb9a-46f8-a221-acbbcb906838"
    t.child_frame_id = "unity_cam"
    t.transform.translation.x = data.transforms[0].transform.translation.x
    t.transform.translation.y = data.transforms[0].transform.translation.y
    t.transform.translation.z = data.transforms[0].transform.translation.z
    t.transform.rotation.x = data.transforms[0].transform.rotation.x
    t.transform.rotation.y = data.transforms[0].transform.rotation.y
    t.transform.rotation.z = data.transforms[0].transform.rotation.z
    t.transform.rotation.w = data.transforms[0].transform.rotation.w

    print(t)


if __name__ == '__main__':
    rospy.init_node('tf_remapper')

    listener = tf.TransformListener()
    publisher = tf2_ros.TransformBroadcaster()
    msg_listener = rospy.Subscriber("unity_cam_tf", tf2_msgs.msg.TFMessage, callback)

    rate = rospy.Rate(60.0)
    while not rospy.is_shutdown():
        # try:
        #     trans, rot = listener.lookupTransform('/unity_cam', '/7e62ccfd-eb9a-46f8-a221-acbbcb906838', rospy.Time(0)) #('/base_link', '/vision_odometry_frame', rospy.Time(0))
        #     print(trans)
        #     print(rot)

            
        #     print("SENT")
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue
                    
        t.header.stamp = rospy.Time.now()
        publisher.sendTransform(t)
        print(".")

        rate.sleep()
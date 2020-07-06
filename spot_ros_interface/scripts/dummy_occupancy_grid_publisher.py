import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import visualization_msgs.msg
import math
import time
import numpy as np

'''
This script publishes dummy 3D occupancy voxel grid for debugging
'''

occ_grid_pub = rospy.Publisher(
    "occupancy_grid", visualization_msgs.msg.MarkerArray, queue_size=20)

rospy.init_node('dummy_occupancy_grid_pub')
rate = rospy.Rate(1)  # Update at 10hz

def convert_np_list_to_marker_array(grid, tf_to_trf_cell):
    marker = visualization_msgs.msg.Marker()
    marker_array = visualization_msgs.msg.MarkerArray()
    
    # tf_to_trf_cell is the transform (just position) to the top right front cell
    # TODO: This

start_time = int(rospy.Time().now().secs)+5
while not rospy.is_shutdown():
    print('. {} '.format(1 + int(rospy.Time().now().secs)-start_time))

    marker = visualization_msgs.msg.Marker()

    # Populate actual data
    grid = visualization_msgs.msg.MarkerArray()

    marker.header.seq=0
    marker.header.stamp= rospy.Time()
    marker.header.frame_id= "base_link" #Must be map or another frame that exists (e.g. Spot's ko_frame)
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.pose.position.x = 1 + int(rospy.Time().now().secs)-start_time
    marker.pose.position.y = 1
    marker.pose.position.z = 1
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 1.0
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    
    grid.markers.append(marker)
    occ_grid_pub.publish(grid)

    rate.sleep()

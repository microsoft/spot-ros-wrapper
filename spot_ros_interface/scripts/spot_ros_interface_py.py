#!/usr/bin/env python3

import argparse
import sys
import time

# Bosdyn specific imports
import bosdyn.client
import bosdyn.client.estop
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry

from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient

# ROS specific imports
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
from sensor_msgs import Image

class RobotCallback:
    '''Callbacks for an instance of a Spot robot'''

    VELOCITY_CMD_DURATION = 0.6  # seconds

    def __init__(self, robot_instance):
        self.robot = robot_instance
        self.command_client = self.robot.ensure_client(
            RobotCommandClient.default_service_name)

    # Callback functions
    # TODO: Should each callback check that inputs are within bounds?
    def stand_cb(self, height):
        """Callback that sends stand cmd at a given height delta [m] from standard configuration"""
        # TODO: Pick a msg type that allows for body rotations while standing

        self.robot.logger.info("Commanding robot to stand...")
        cmd = RobotCommandBuilder.stand_command(body_height=height)
        self.command_client.robot_command(cmd)
        self.robot.logger.info("Robot standing cmd sent.")

    def trajectory_cb(self, pose):
        '''Callback that specifies a waypoint (Point) [m] with a final orientation [rad]'''
        # TODO: Allow for an array of waypoints to follow
        # TODO: Pose msg has many fields that do not go unused. Consider changing msg type

        x = pose.position.x
        y = pose.position.y
        heading = 0 # TODO: Convert pose.orientation (quaterion) into EulerZYX (y,p,r)

        cmd = RobotCommandBuilder.trajectory_command(
            goal_x =x,
            goal_y =y,
            goal_heading = heading,
            frame = ,

        )
        self.command_client.robot_command_async(
            cmd,
            end_time_secs=time.time() + self.VELOCITY_CMD_DURATION
        )



    def velocity_cb(self, twist):
        """Callback that sends instantaneous velocity [m/s] commands to Spot"""
        # TODO:Twist msg has many fields that do not go unused. Consider changing msg type

        v_x = twist.linear.x
        v_y = twist.linear.y
        v_rot = twist.angular.z

        cmd = RobotCommandBuilder.velocity_command(
            v_x=v_x,
            v_y=v_y,
            v_rot=v_rot
        )
        self.command_client.robot_command_async(
            cmd,
            end_time_secs=time.time() + self.VELOCITY_CMD_DURATION
        )
        self.robot.logger.info("Robot velocity cmd sent: v_x=${},v_y=${},v_rot${}".format(v_x, v_y, v_rot))


def spot_ros_interface(config):

    # Set up SDK
    bosdyn.client.util.setup_logging(config.verbose)
    sdk = bosdyn.client.create_standard_sdk('spot_ros_interface_sdk')
    sdk.load_app_token(config.app_token)

    # Create instance of a robot
    robot = sdk.create_robot(config.hostname)
    robot.authenticate(config.username, config.password)
    robot.time_sync.wait_for_sync()

    # ROS Node initialization
    rospy.init_node('spot_ros_interface_py')
    rate = rospy.Rate(10) # Update at 10hz

    # Specify topics interface will subscribe to
    # Each subscriber/topic will handle a specific command to Spot instance
    robot_callback = RobotCallback(robot)
    
    rospy.Subscriber("velocity_cmd", Twist, robot_callback.velocity_cb)
    rospy.Subscriber("stand_cmd", Float32, robot_callback.stand_cb)
    rospy.Subscriber("trajectory_cmd", Pose, robot_callback.trajectory_cb)

    # TODO: Best practices on streaming sensor data from multiple sensors? Bandwidth concern?
    # image_pub = rospy.Publisher("image", , queue_size=10) # TODO: Publish Image(s)
    # state_pub = rospy.Publisher("state", ,queue_size=10) # TODO: Publish robot state

    # Spot requires a software estop to be activated.
    estop_client = robot.ensure_client(
        bosdyn.client.estop.EstopClient.default_service_name)
    estop_endpoint = bosdyn.client.estop.EstopEndpoint(
        client=estop_client, name='HelloSpot', estop_timeout=9.0)
    estop_endpoint.force_simple_setup()

    # Only one client at a time can operate a robot.
    lease_client = robot.ensure_client(
        bosdyn.client.lease.LeaseClient.default_service_name)
    lease = lease_client.acquire()
    try:
        with bosdyn.client.lease.LeaseKeepAlive(lease_client), bosdyn.client.estop.EstopKeepAlive(
                estop_endpoint):
            robot.logger.info("Powering on robot... This may take a several seconds.")
            robot.power_on(timeout_sec=20)
            assert robot.is_powered_on(), "Robot power on failed."
            robot.logger.info("Robot powered on.")

            while not rospy.is_shutdown():
                # TODO: Publish
                # image_pub.publish() 
                # state_pub.publish()
                rate.sleep()

    finally:
        # If we successfully acquired a lease, return it.
        lease_client.return_lease(lease)

if __name__ == '__main__':
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    options = parser.parse_args(argv[1:])
    try:
        spot_ros_interface(options)
        return True
    except rospy.ROSInterruptException:
        pass
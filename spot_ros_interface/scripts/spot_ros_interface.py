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
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import trajectory_pb2, image_pb2, robot_state_pb2

# ROS specific imports
import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg


class SpotInterface:
    '''Callbacks for an instance of a Spot robot'''

    # 0.6 s is the standard duration for cmds in boston dynamics Spot examples
    VELOCITY_CMD_DURATION = 0.6  # [seconds]

    def __init__(self, config):

        # Set up SDK
        bosdyn.client.util.setup_logging(config.verbose)
        self.sdk = bosdyn.client.create_standard_sdk('spot_ros_interface_sdk')
        self.sdk.load_app_token(config.app_token)

        # Create instance of a robot
        self.robot = self.sdk.create_robot(config.hostname)
        self.robot.authenticate(config.username, config.password)
        self.robot.time_sync.wait_for_sync()

        # Client to send cmds to Spot
        self.command_client = self.robot.ensure_client(
            RobotCommandClient.default_service_name)

        # Client to request images from Spot
        self.image_client = self.robot.ensure_client(
            ImageClient.default_service_name)

        self.image_source_names = [
            src.name for src in self.image_client.list_image_sources() if "image" in src.name
        ]

        self.depth_image_sources = [
            src.name for src in self.image_client.list_image_sources() if "depth" in src.name
        ]

        # Client to request robot state
        self.robot_state_client = self.robot.ensure_client(
            RobotStateClient.default_service_name)

        # Spot requires a software estop to be activated.
        estop_client = self.robot.ensure_client(
            bosdyn.client.estop.EstopClient.default_service_name)
        self.estop_endpoint = bosdyn.client.estop.EstopEndpoint(
            client=estop_client, name='spot_ros_interface', estop_timeout=9.0)
        self.estop_endpoint.force_simple_setup()

        # Only one client at a time can operate a robot.
        self.lease_client = self.robot.ensure_client(
            bosdyn.client.lease.LeaseClient.default_service_name)
        self.lease = self.lease_client.acquire()

    # Callback functions

    def stand_cb(self, height):
        """Callback that sends stand cmd at a given height delta [m] from standard configuration"""
        # TODO: Pick a msg type that allows for body rotations while standing

        self.robot.logger.info("Commanding robot to stand...")
        cmd = RobotCommandBuilder.stand_command(body_height=height)
        self.command_client.robot_command(cmd)
        self.robot.logger.info(
            "Robot stand cmd sent. Height: {}".format(height))

    def trajectory_cb(self, pose):
        '''Callback that specifies a waypoint (Point) [m] with a final orientation [rad]'''
        # TODO: Allow for an array of waypoints to follow
        # TODO: Pose msg has many fields that do not go unused. Consider changing msg type
        # TODO: Support other reference frames (currently only body ref. frame)

        x = pose.position.x
        y = pose.position.y
        # TODO: Convert pose.orientation (quaterion) into EulerZYX (y,p,r)
        heading = 0

        cmd = RobotCommandBuilder.trajectory_command(
            goal_x=x,
            goal_y=y,
            goal_heading=heading,
            frame=trajectory_pb2.bosdyn_dot_api_dot_geometry__pb2.FRAME_BODY,
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
        self.robot.logger.info(
            "Robot velocity cmd sent: v_x=${},v_y=${},v_rot${}".format(v_x, v_y, v_rot))

    def get_robot_state(self): # TODO: Unit test the get_state method conversion from pbuf to ROS msg (test reeated fields, etc)
        ''' Returns tuple of kinematic_state, robot_state
            kinematic_state:
                timestamp
                joint_states []
                ko_tform_body
                body_twist_rt_ko
                ground_plane_rt_ko
                vo_tform_body
            robot_state: #TODO
                power_state
                battery_states[]
                comms_states[]
                system_fault_state
                estop_states[]
                behavior_fault_state
        '''
        robot_state = self.robot_state_client.get_robot_state()
        
        ### PowerState conversion
        # robot_state.power_state.timestamp #[google.protobuf.Timestamp]
        # robot_state.power_state.motor_power_state #[enum]
        # robot_state.power_state.shore_power_state #[enum]

        ### BatteryState conversion [repeated field]
        # robot_state.battery_states.timestamp #[google.protobuf.Timestamp]
        # robot_state.battery_states.identifier #[string]
        # robot_state.battery_states.charge_percentage #[double]
        # robot_state.battery_states.estimated_runtime #[google.protobuf.Duration]
        # robot_state.battery_states.current #[Double]
        # robot_state.battery_states.voltage #[Double]
        # robot_state.battery_states.temperatures #[repeated - Double]
        # robot_state.battery_states.status #[enum]

        ### CommsState conversion [repeated field]
        # robot_state.comms_states.timestamp #[google.protobuf.Timestamp]
        '''wifi_state is Repeated'''
        # robot_state.comms_states.wifi_state.current_mode #[enum] Note: wifi_state is oneof
        # robot_state.comms_states.wifi_state.essid #[string]     

        ### SystemFaultState conversion
        '''faults is Repeated'''
        # robot_state.system_fault_state.faults.name #[string]
        # robot_state.system_fault_state.faults.onset_timestamp #[google.protobuf.Timestamp]
        # robot_state.system_fault_state.faults.duration #[google.protobuf.Duration]
        # robot_state.system_fault_state.faults.code #[int32]
        # robot_state.system_fault_state.faults.uid #[uint64]
        # robot_state.system_fault_state.faults.error_message #[string]
        # robot_state.system_fault_state.faults.attributes #[repeated-string]
        # robot_state.system_fault_state.faults.severity #[enum]
        '''historical_faults is Repeated'''
        # robot_state.system_fault_state.historical_faults.name #[string]
        # robot_state.system_fault_state.historical_faults.onset_timestamp #[google.protobuf.Timestamp]
        # robot_state.system_fault_state.historical_faults.duration #[google.protobuf.Duration]
        # robot_state.system_fault_state.historical_faults.code #[int32]
        # robot_state.system_fault_state.historical_faults.uid #[uint64]
        # robot_state.system_fault_state.historical_faults.error_message #[string]
        # robot_state.system_fault_state.historical_faults.attributes #[repeated-string]
        # robot_state.system_fault_state.historical_faults.severity #[enum]

        # robot_state.system_fault_state.aggregated #[map<string,enum>]

        ### EStopState conversion [repeated field]
        # robot_state.estop_states.timestamp #[google.protobuf.Timestamp]
        # robot_state.estop_states.name #[string]
        # robot_state.estop_states.type #[enum]
        # robot_state.estop_states.state #[enum]
        # robot_state.estop_states.state_description #[string]

        ### KinematicState conversion
        ks_msg = spot_ros_msgs.msg.KinematicState()

        ks_msg.header.stamp = robot_state.kinematic_state.timestamp #[google.protobuf.Timestamp]

        '''joint_states is repeated'''
        js = sensor_msgs.msg.JointState()
        for joint_state in robot_state.kinematic_state.joint_states:
            js.name.append(joint_state.name) #[string]
            js.position.append(joint_state.position) #[DoubleValue] Note: angle in rad
            js.velocity.append(joint_state.velocity) #[DoubleValue] Note: ang vel
            #js.acc(joint_state.acceleration) #[DoubleValue] Note: ang accel. JointState doesn't have accel. Ignoring for now.
            js.effort.append(joint_state.load) #[DoubleValue] Note: Torque in N-m
        ks_msg.joint_states = js

        ks_msg.ko_tform_body.translation.x = robot_state.kinematic_state.ko_tform_body.position.x #[double]
        ks_msg.ko_tform_body.translation.y = robot_state.kinematic_state.ko_tform_body.position.y #[double]
        ks_msg.ko_tform_body.translation.z = robot_state.kinematic_state.ko_tform_body.position.z #[double]
        ks_msg.ko_tform_body.rotation.x = robot_state.kinematic_state.ko_tform_body.rotation.x #[double]
        ks_msg.ko_tform_body.rotation.y = robot_state.kinematic_state.ko_tform_body.rotation.y #[double]
        ks_msg.ko_tform_body.rotation.z = robot_state.kinematic_state.ko_tform_body.rotation.z #[double]
        ks_msg.ko_tform_body.rotation.w = robot_state.kinematic_state.ko_tform_body.rotation.w #[double]

        ks_msg.body_twist_rt_ko.linear.x = robot_state.kinematic_state.body_twist_rt_ko.linear.x #[double]
        ks_msg.body_twist_rt_ko.linear.y = robot_state.kinematic_state.body_twist_rt_ko.linear.y #[double]
        ks_msg.body_twist_rt_ko.linear.z = robot_state.kinematic_state.body_twist_rt_ko.linear.z #[double]
        ks_msg.body_twist_rt_ko.angular.x = robot_state.kinematic_state.body_twist_rt_ko.angular.x #[double]
        ks_msg.body_twist_rt_ko.angular.y = robot_state.kinematic_state.body_twist_rt_ko.angular.y #[double]
        ks_msg.body_twist_rt_ko.angular.z = robot_state.kinematic_state.body_twist_rt_ko.angular.z #[double]

        #robot_state.kinematic_state.ground_plane_rt_ko.point.x #[Vec3] #Ignoring for now
        #robot_state.kinematic_state.ground_plane_rt_ko.normal #[Vec3] #Ignoring for now

        ks_msg.vo_tform_body.translation.x = robot_state.kinematic_state.vo_tform_body.position.x #[double]
        ks_msg.vo_tform_body.translation.y = robot_state.kinematic_state.vo_tform_body.position.y #[double]
        ks_msg.vo_tform_body.translation.z = robot_state.kinematic_state.vo_tform_body.position.z #[double]
        ks_msg.vo_tform_body.rotation.x = robot_state.kinematic_state.vo_tform_body.rotation.x #[double]
        ks_msg.vo_tform_body.rotation.y = robot_state.kinematic_state.vo_tform_body.rotation.y #[double]
        ks_msg.vo_tform_body.rotation.z = robot_state.kinematic_state.vo_tform_body.rotation.z #[double]
        ks_msg.vo_tform_body.rotation.w = robot_state.kinematic_state.vo_tform_body.rotation.w #[double]

        ### BehaviourFaultState conversion
        '''faults is repeated'''
        # robot_state.behavior_fault_state.faults.behavior_fault_id #[uint32]
        # robot_state.behavior_fault_state.faults.onset_timestamp #[google.protobuf.Timestamp]
        # robot_state.behavior_fault_state.faults.cause #[enum]
        # robot_state.behavior_fault_state.faults.status #[enum]

        return ks_msg, None #TODO: Return robot_state instead of None

    def start_spot_ros_interface(self):

        # ROS Node initialization
        rospy.init_node('spot_ros_interface_py')
        rate = rospy.Rate(10)  # Update at 10hz

        # Specify topics interface will subscribe to
        # Each subscriber/topic will handle a specific command to Spot instance

        rospy.Subscriber(
            "velocity_cmd", geometry_msgs.msg.Twist, self.velocity_cb)
        rospy.Subscriber("stand_cmd", std_msgs.msg.Float32, self.stand_cb)
        rospy.Subscriber("trajectory_cmd",
                         geometry_msgs.msg.Pose, self.trajectory_cb)

        # Single image publisher will publish all images from all Spot cameras
        image_pub = rospy.Publisher(
            "image", sensor_msgs.msg.Image, queue_size=20)
        kinematic_state_pub = rospy.Publisher(
            "kinematic_state", spot_ros_msgs.msg.KinematicState, queue_size=20)
        
        # depth_image_pub = rospy.Publisher(
        #     "depth_image", sensor_msgs.msg.Image, queue_size=20) # TODO: Publish depth imgs
        # state_pub = rospy.Publisher("state", ,queue_size=10) # TODO: Publish robot state

        try:
            with bosdyn.client.lease.LeaseKeepAlive(self.lease_client), bosdyn.client.estop.EstopKeepAlive(
                    self.estop_endpoint):
                self.robot.logger.info(
                    "Powering on robot... This may take a several seconds.")
                self.robot.power_on(timeout_sec=20)
                assert self.robot.is_powered_on(), "Robot power on failed."
                self.robot.logger.info("Robot powered on.")

                while not rospy.is_shutdown():
                    ''' Publish Robot State'''
                    kinematic_state, robot_state = self.get_robot_state()
                    kinematic_state_pub.publish(kinematic_state)
                    # robot_state_pub.publish(robot_state)

                    ''' Publish Images'''
                    # Each element in image_response list is an image from each one of the sensors
                    image_list = self.image_client.get_image_from_sources(
                        self.image_source_names)

                    for img in enumerate(image_list):
                        # img[0] is enum, img[1] is image response
                        header = std_msgs.msg.Header()
                        header.stamp = rospy.Time.now()
                        # image source identifier
                        header.frame_id = self.image_source_names[img[0]]

                        i = sensor_msgs.msg.Image()
                        i.header = header

                        i.width = img[1].shot.image.cols
                        i.height = img[1].shot.image.rows
                        i.data = img[1].shot.image.data

                        image_pub.publish(i)

                    # state_pub.publish()
                    rospy.logdebug("Looping...")
                    rate.sleep()

        finally:
            # If we successfully acquired a lease, return it.
            self.lease_client.return_lease(self.lease)

if __name__ == '__main__':
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_common_arguments(parser)
    options = parser.parse_args(sys.argv[1:])
    try:
        robot = SpotInterface(options)
        robot.start_spot_ros_interface()
    except rospy.ROSInterruptException:
        pass
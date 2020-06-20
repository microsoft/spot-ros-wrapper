#!/usr/bin/env python3

import argparse
import logging
import math
import sys
import os
import subprocess
import time
import pdb # For debugging only

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

from bosdyn.client.frame_helpers import get_a_tform_b, get_vision_tform_body, get_odom_tform_body,\
    BODY_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME, VISION_FRAME_NAME, ODOM_FRAME_NAME 

# ROS specific imports
import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import spot_ros_msgs.msg
import spot_ros_srvs.srv


class SpotInterface:
    '''Callbacks for an instance of a Spot robot'''

    # 0.6 s is the standard duration for cmds in boston dynamics Spot examples
    VELOCITY_CMD_DURATION = 0.6  # [seconds]
    TRAJECTORY_CMD_TIMEOUT = 30  # [seconds] If None, go-to command will never time out
    x_goal_tolerance = 0.05 #[m]
    y_goal_tolerance = 0.05 #[m]
    angle_goal_tolerance = 0.075 #[rad]
    LOGGER = logging.getLogger()

    def __init__(self, config):
        # Ensure interface can ping Spot
        try:
            with open(os.devnull, 'wb') as devnull:
                resp = subprocess.check_call(['ping', '-c', '1', config.hostname], stdout=devnull, stderr=subprocess.STDOUT)
                if resp != 0:
                    print ("ERROR: Cannot detect a Spot with IP: {}.\n Make sure Spot is powered on and on the same network".format(config.hostname))
                    sys.exit()
        except:
            print("ERROR: Cannot detect a Spot with IP: {}.\n Make sure Spot is powered on and on the same network".format(config.hostname))
            sys.exit()

        # Set up SDK
        bosdyn.client.util.setup_logging(config.verbose)
        self.sdk = bosdyn.client.create_standard_sdk('spot_ros_interface_sdk')
        self.sdk.load_app_token(config.app_token)

        # Create instance of a robot
        try:
            self.robot = self.sdk.create_robot(config.hostname)
            self.robot.authenticate(config.username, config.password)
            self.robot.time_sync.wait_for_sync()
        except bosdyn.client.RpcError as err:
            self.LOGGER.error("Failed to communicate with robot: %s", err)

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
        try:
            self.lease = self.lease_client.acquire()
        except bosdyn.client.lease.ResourceAlreadyClaimedError as err:
            print("ERROR: Lease cannot be acquired. Ensure no other client has the lease. Shutting down.")
            print(err)
            sys.exit()
        # True for RViz visualization of Spot in 3rd person with occupancy grid
        self.third_person_view = True

    ### Callback functions ###

    def self_right_cmd_srv(self, stand):
        """ Callback that sends self-right cmd"""
        cmd = RobotCommandBuilder.selfright_command()
        ret = self.command_client.robot_command(cmd)
        rospy.loginfo("Robot self right cmd sent. {}".format(ret))

        return []

    def stand_cmd_srv(self, stand):
        """Callback that sends stand cmd at a given height delta [m] from standard configuration"""

        cmd = RobotCommandBuilder.stand_command(body_height=stand.body_pose.translation.z, footprint_R_body=self.quat_to_euler(stand.body_pose.rotation))
        ret = self.command_client.robot_command(cmd)
        rospy.loginfo("Robot stand cmd sent. {}".format(ret))

        return []

    def trajectory_cmd_srv(self, trajectory):
        '''
        Callback that specifies waypoint(s) (Point) [m] with a final orientation [rad]

        The name of the frame that trajectory is relative to.
        The trajectory must be expressed in a gravity aligned frame, so either "vision", "odom", or "flat_body".
        Any other provided se2_frame_name will be rejected and the trajectory command will not be exectuted.
        '''
        # TODO: Support other reference frames (currently only body ref. frame)

        for pose in trajectory:
            x = pose.position.x
            y = pose.position.y
            heading = self.quat_to_euler(pose.orientation)[2]
            frame = GRAV_ALIGNED_BODY_FRAME_NAME

            cmd = RobotCommandBuilder.trajectory_command(
                goal_x=x,
                goal_y=y,
                goal_heading=heading,
                frame=frame,
            )
            reached_goal = self.block_until_pose_reached(cmd, (x,y,heading))
            rospy.loginfo("Waypoint: ({},{},{}). Waypoint reached: {}".format(x,y,heading, reached_goal))

        robot_state = self.get_robot_state()[0].ko_tform_body
        final_pose = geometry_msgs.msg.Pose()
        final_pose.position = robot_state.translation
        final_pose.orientation = robot_state.rotation

        spot_ros_srvs.srv.TrajectoryResponse(final_pose)

    def velocity_cmd_srv(self, twist):
        """Callback that sends instantaneous velocity [m/s] commands to Spot"""
        
        v_x = twist.velocity.linear.x
        v_y = twist.velocity.linear.y
        v_rot = twist.velocity.angular.z

        cmd = RobotCommandBuilder.velocity_command(
            v_x=v_x,
            v_y=v_y,
            v_rot=v_rot
        )
        # Issue command to robot
        self.command_client.robot_command(
            cmd,
            end_time_secs=time.time() + self.VELOCITY_CMD_DURATION
        )
        rospy.loginfo(
            "Robot velocity cmd sent: v_x=${},v_y=${},v_rot${}".format(v_x, v_y, v_rot))
        return []
    ### Helper functions ###

    def block_until_pose_reached(self, cmd, goal):
        """Do not return until goal waypoint is reached, or TRAJECTORY_CMD_TIMEOUT is reached."""
        # TODO: Make trajectory_cmd_timeout part of the service request
        
        # Issue command to robot
        self.command_client.robot_command(
            cmd,
            end_time_secs = time.time()+self.TRAJECTORY_CMD_TIMEOUT if self.TRAJECTORY_CMD_TIMEOUT else None
        )

        start_time = time.time()
        current_time = time.time()
        while (not self.is_final_state(goal) and (current_time - start_time < self.TRAJECTORY_CMD_TIMEOUT if self.TRAJECTORY_CMD_TIMEOUT else True)):
            time.sleep(.25)
            current_time = time.time()
        return self.is_final_state(goal)

    def is_final_state(self, goal):
        """Check if the current robot state is within range of the specified position."""
        goal_x=goal[0]
        goal_y=goal[1]
        goal_heading=goal[2]
        robot_state = self.get_robot_state()[0].ko_tform_body
        robot_pose = robot_state.translation
        robot_angle = self.quat_to_euler((robot_state.rotation.x, robot_state.rotation.y,
                                          robot_state.rotation.z, robot_state.rotation.w))[2]

        x_dist = abs(goal_x - robot_pose.x)
        y_dist = abs(goal_y - robot_pose.y)
        angle = abs(goal_heading - robot_angle)
        if ((x_dist < self.x_goal_tolerance) and (y_dist < self.y_goal_tolerance) and (angle < self.angle_goal_tolerance)):
            return True
        return False

    def quat_to_euler(self, quat):
        """Convert a quaternion to xyz Euler angles."""
        q = [quat.x, quat.y, quat.z, quat.w]
        roll = math.atan2(2 * q[3] * q[0] + q[1] * q[2], 1 - 2 * q[0]**2 + 2 * q[1]**2)
        pitch = math.atan2(2 * q[1] * q[3] - 2 * q[0] * q[2], 1 - 2 * q[1]**2 - 2 * q[2]**2)
        yaw = math.atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], 1 - 2 * q[1]**2 - 2 * q[2]**2)
        return bosdyn.geometry.EulerZXY(yaw=yaw, roll=roll, pitch=pitch)

    # TODO: Unit test the get_state method conversion from pbuf to ROS msg (test repeated fields, etc)
    def get_robot_state(self):
        ''' Returns tuple of kinematic_state, robot_state
            kinematic_state:
                timestamp
                joint_states []
                ko_tform_body
                body_twist_rt_ko
                ground_plane_rt_ko
                vo_tform_body
            robot_state:
                power_state
                battery_states[]
                comms_states[]
                system_fault_state
                estop_states[]
                behavior_fault_state
        '''
        robot_state = self.robot_state_client.get_robot_state()
        rs_msg = spot_ros_msgs.msg.RobotState()
        
        ### PowerState conversion
        #[google.protobuf.Timestamp]
        rs_msg.power_state.header.stamp.secs =  robot_state.power_state.timestamp.seconds
        rs_msg.power_state.header.stamp.nsecs =  robot_state.power_state.timestamp.nanos
        rs_msg.power_state.motor_power_state = robot_state.power_state.motor_power_state #[enum]
        rs_msg.power_state.shore_power_state = robot_state.power_state.shore_power_state #[enum]
        rs_msg.power_state.locomotion_charge_percentage = robot_state.power_state.locomotion_charge_percentage.value #[google.protobuf.DoubleValue]
        rs_msg.power_state.locomotion_estimated_runtime.secs = robot_state.power_state.locomotion_estimated_runtime.seconds #[google.protobuf.Duration]

        ### BatteryState conversion [repeated field] 
        for battery_state in robot_state.battery_states:
            battery_state_msg = sensor_msgs.msg.BatteryState()

            header = std_msgs.msg.Header()
            #[google.protobuf.Timestamp]
            header.stamp.secs = battery_state.timestamp.seconds
            header.stamp.nsecs = battery_state.timestamp.nanos
            header.frame_id = battery_state.identifier #[string]

            battery_state_msg.header = header

            battery_state_msg.percentage = battery_state.charge_percentage.value/100 #[double]
            # NOTE: Using battery_state_msg.charge as the estimated runtime in sec
            battery_state_msg.charge = battery_state.estimated_runtime.seconds #[google.protobuf.Duration]
            battery_state_msg.current = battery_state.current.value #[DoubleValue]
            battery_state_msg.voltage = battery_state.voltage.value #[DoubleValue]
            # NOTE: Ignoring temperatures for now; no field in BatteryState maps directly to it
            # battery_state_msg. = battery_state.temperatures #[repeated - Double]
            battery_state_msg.power_supply_status = battery_state.status #[enum]

            rs_msg.battery_states.append(battery_state_msg)

        ### CommsState conversion [repeated field]
        for comms_state in robot_state.comms_states:
            comms_state_msg = spot_ros_msgs.msg.CommsState()

            comms_state_msg.header.stamp.secs = comms_state.timestamp.seconds #[google.protobuf.Timestamp]
            comms_state_msg.header.stamp.nsecs = comms_state.timestamp.nanos #[google.protobuf.Timestamp]
            comms_state_msg.wifi_mode = comms_state.wifi_state.current_mode #[enum] Note: wifi_state is oneof
            comms_state_msg.essid = comms_state.wifi_state.essid #[string]

            rs_msg.comms_states.append(comms_state_msg)

        ### SystemFaultState conversion
        '''faults is Repeated'''
        for fault in robot_state.system_fault_state.faults:
            system_fault_msg = spot_ros_msgs.msg.SystemFault()

            system_fault_msg.header.frame_id = fault.name #[string]
            system_fault_msg.header.stamp.secs = fault.onset_timestamp.seconds #[google.protobuf.Timestamp]
            system_fault_msg.header.stamp.nsecs = fault.onset_timestamp.nanos #[google.protobuf.Timestamp]
            system_fault_msg.duration.secs = fault.duration.seconds #[google.protobuf.Duration]
            system_fault_msg.duration.nsecs = fault.duration.nanos #[google.protobuf.Duration]
            system_fault_msg.code = fault.code #[int32]
            system_fault_msg.uid =  fault.uid #[uint64]
            system_fault_msg.error_message = fault.error_message #[string]
            system_fault_msg.attributes = fault.attributes #[repeated-string]
            system_fault_msg.severity = fault.severity #[enum]

            rs_msg.system_fault_state.faults.append(system_fault_msg)

        '''historical_faults is Repeated'''
        for historical_fault in robot_state.system_fault_state.faults:
            system_fault_msg = spot_ros_msgs.msg.SystemFault()

            system_fault_msg.header.frame_id = historical_fault.name #[string]
            system_fault_msg.header.stamp.secs = historical_fault.onset_timestamp.seconds #[google.protobuf.Timestamp]
            system_fault_msg.header.stamp.nsecs = historical_fault.onset_timestamp.nanos #[google.protobuf.Timestamp]
            system_fault_msg.duration.secs = historical_fault.duration.seconds #[google.protobuf.Duration]
            system_fault_msg.duration.nsecs = historical_fault.duration.nanos #[google.protobuf.Duration]
            system_fault_msg.code = historical_fault.code #[int32]
            system_fault_msg.uid =  historical_fault.uid #[uint64]
            system_fault_msg.error_message = historical_fault.error_message #[string]
            system_fault_msg.attributes = historical_fault.attributes #[repeated-string]
            system_fault_msg.severity = historical_fault.severity #[enum]

            rs_msg.system_fault_state.historical_faults.append(system_fault_msg)

        #[map<string,enum>]
        if robot_state.system_fault_state.aggregated:
            rs_msg.system_fault_state.aggregated.key = robot_state.system_fault_state.aggregated.key
            rs_msg.system_fault_state.aggregated.value = robot_state.system_fault_state.aggregated.value

        ### EStopState conversion [repeated field]
        for estop_state in robot_state.estop_states:
            estop_msg = spot_ros_msgs.msg.EStopState()

            estop_msg.header.stamp.secs = estop_state.timestamp.seconds #[google.protobuf.Timestamp]
            estop_msg.header.stamp.nsecs = estop_state.timestamp.nanos #[google.protobuf.Timestamp]
            estop_msg.header.frame_id = estop_state.name #[string]
            estop_msg.type = estop_state.type #[enum]
            estop_msg.state = estop_state.state #[enum]
            estop_msg.state_description = estop_state.state_description #[string]

            rs_msg.estop_states.append(estop_msg)

        ### KinematicState conversion
        ks_msg = spot_ros_msgs.msg.KinematicState()

        # [google.protobuf.Timestamp]
        ks_msg.header.stamp.secs = robot_state.kinematic_state.acquisition_timestamp.seconds
        ks_msg.header.stamp.nsecs = robot_state.kinematic_state.acquisition_timestamp.nanos

        '''joint_states is repeated'''
        js = sensor_msgs.msg.JointState()
        for joint_state in robot_state.kinematic_state.joint_states:
            # [string]
            js.name.append(joint_state.name)
            # [DoubleValue] Note: angle in rad
            js.position.append(joint_state.position.value)
            # [DoubleValue] Note: ang vel
            js.velocity.append(joint_state.velocity.value)
            #[DoubleValue] Note: ang accel. JointState doesn't have accel. Ignoring for now.
            # js.acc(joint_state.acceleration)
            # [DoubleValue] Note: Torque in N-m
            js.effort.append(joint_state.load.value)
        ks_msg.joint_states = js

        ''' vision_tform_body'''
        # SE3Pose representing transform of Spot's Body frame relative to the inertial Vision frame
        vision_tform_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # [double]
        ks_msg.vision_tform_body.translation.x = vision_tform_body.x
        # [double]
        ks_msg.vision_tform_body.translation.y = vision_tform_body.y
        # [double]
        ks_msg.vision_tform_body.translation.z = vision_tform_body.z
        # [double]
        ks_msg.vision_tform_body.rotation.x = vision_tform_body.rot.x
        # [double]
        ks_msg.vision_tform_body.rotation.y = vision_tform_body.rot.y
        # [double]
        ks_msg.vision_tform_body.rotation.z = vision_tform_body.rot.z
        # [double]
        ks_msg.vision_tform_body.rotation.w = vision_tform_body.rot.w

        ''' odom_tform_body '''
        # SE3Pose representing transform of Spot's Body frame relative to the inertial Vision frame
        odom_tform_body = get_odom_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # [double]
        ks_msg.odom_tform_body.translation.x = odom_tform_body.x
        # [double]
        ks_msg.odom_tform_body.translation.y = odom_tform_body.y
        # [double]
        ks_msg.odom_tform_body.translation.z = odom_tform_body.z
        # [double]
        ks_msg.odom_tform_body.rotation.x = odom_tform_body.rot.x
        # [double]
        ks_msg.odom_tform_body.rotation.y = odom_tform_body.rot.y
        # [double]
        ks_msg.odom_tform_body.rotation.z = odom_tform_body.rot.z
        # [double]
        ks_msg.odom_tform_body.rotation.w = odom_tform_body.rot.w

        ''' velocity_of_body_in_vision '''
        # [double]
        ks_msg.velocity_of_body_in_vision.linear.x = robot_state.kinematic_state.velocity_of_body_in_vision.linear.x
        # [double]
        ks_msg.velocity_of_body_in_vision.linear.y = robot_state.kinematic_state.velocity_of_body_in_vision.linear.y
        # [double]
        ks_msg.velocity_of_body_in_vision.linear.z = robot_state.kinematic_state.velocity_of_body_in_vision.linear.z
        # [double]
        ks_msg.velocity_of_body_in_vision.angular.x = robot_state.kinematic_state.velocity_of_body_in_vision.angular.x
        # [double]
        ks_msg.velocity_of_body_in_vision.angular.y = robot_state.kinematic_state.velocity_of_body_in_vision.angular.y
        # [double]
        ks_msg.velocity_of_body_in_vision.angular.z = robot_state.kinematic_state.velocity_of_body_in_vision.angular.z

        ''' velocity_of_body_in_odom '''
        # [double]
        ks_msg.velocity_of_body_in_odom.linear.x = robot_state.kinematic_state.velocity_of_body_in_odom.linear.x
        # [double]
        ks_msg.velocity_of_body_in_odom.linear.y = robot_state.kinematic_state.velocity_of_body_in_odom.linear.y
        # [double]
        ks_msg.velocity_of_body_in_odom.linear.z = robot_state.kinematic_state.velocity_of_body_in_odom.linear.z
        # [double]
        ks_msg.velocity_of_body_in_odom.angular.x = robot_state.kinematic_state.velocity_of_body_in_odom.angular.x
        # [double]
        ks_msg.velocity_of_body_in_odom.angular.y = robot_state.kinematic_state.velocity_of_body_in_odom.angular.y
        # [double]
        ks_msg.velocity_of_body_in_odom.angular.z = robot_state.kinematic_state.velocity_of_body_in_odom.angular.z


        ### BehaviorFaultState conversion
        '''faults is repeated'''
        for fault in robot_state.behavior_fault_state.faults:
            behaviour_fault_state_msg = spot_ros_msgs.msg.BehaviorFaultState()

            behaviour_fault_state_msg.header.frame_id = fault.behavior_fault_id #[uint32]
            behaviour_fault_state_msg.header.stamp.secs = fault.onset_timestamp.seconds #[google.protobuf.Timestamp]
            behaviour_fault_state_msg.header.stamp.nsecs = fault.onset_timestamp.nanos #[google.protobuf.Timestamp]
            behaviour_fault_state_msg.cause = fault.cause #[enum]
            behaviour_fault_state_msg.status = fault.status #[enum]

            rs_msg.behavior_fault_states.append(behaviour_fault_state_msg)

        ### FootState conversion [repeated]
        for foot_state in robot_state.foot_state:
            foot_state_msg = spot_ros_msgs.msg.FootState()

            foot_state_msg.foot_position_rt_body.x = foot_state.foot_position_rt_body.x #[double]
            foot_state_msg.foot_position_rt_body.y = foot_state.foot_position_rt_body.y #[double]
            foot_state_msg.foot_position_rt_body.z = foot_state.foot_position_rt_body.z #[double]
            foot_state_msg.contact = foot_state.contact #[enum]

            rs_msg.foot_states.append(foot_state_msg)
        

        return ks_msg, rs_msg  #kinematic_state, robot_state

    def start_spot_ros_interface(self):

        # ROS Node initialization
        rospy.init_node('spot_ros_interface_py')
        rate = rospy.Rate(60)  # Update at 60 Hz

        # Each service will handle a specific command to Spot instance
        rospy.Service("self_right_cmd", spot_ros_srvs.srv.Stand, self.self_right_cmd_srv)
        rospy.Service("stand_cmd", spot_ros_srvs.srv.Stand, self.stand_cmd_srv)
        rospy.Service("trajectory_cmd",
                      spot_ros_srvs.srv.Trajectory, self.trajectory_cmd_srv)
        rospy.Service("velocity_cmd", spot_ros_srvs.srv.Velocity, self.velocity_cmd_srv)

        # Single image publisher will publish all images from all Spot cameras
        image_pub = rospy.Publisher(
            "image_capture", spot_ros_msgs.msg.ImageCapture, queue_size=20)
        kinematic_state_pub = rospy.Publisher(
            "kinematic_state", spot_ros_msgs.msg.KinematicState, queue_size=20)
        robot_state_pub = rospy.Publisher(
            "robot_state", spot_ros_msgs.msg.RobotState, queue_size=20)

        # For RViz 3rd person POV visualization
        if self.third_person_view:
            joint_state_pub = rospy.Publisher(
                "joint_state_from_spot", sensor_msgs.msg.JointState, queue_size=20)

        # depth_image_pub = rospy.Publisher(
        #     "depth_image", sensor_msgs.msg.Image, queue_size=20) # TODO: Publish depth imgs

        try:
            with bosdyn.client.lease.LeaseKeepAlive(self.lease_client), bosdyn.client.estop.EstopKeepAlive(
                    self.estop_endpoint):
                rospy.loginfo("Acquired lease")
                rospy.loginfo("Powering on robot... This may take a several seconds.")
                self.robot.power_on(timeout_sec=20)
                assert self.robot.is_powered_on(), "Robot power on failed."
                rospy.loginfo("Robot powered on.")

                while not rospy.is_shutdown():
                    ''' Publish Robot State'''
                    # pdb.set_trace()
                    kinematic_state, robot_state = self.get_robot_state()

                    kinematic_state_pub.publish(kinematic_state)
                    robot_state_pub.publish(robot_state)
                    
                    if self.third_person_view:
                        joint_state_pub.publish(kinematic_state.joint_states)

                    ''' Publish Images'''
                    # Each element in image_response list is an image from each one of the sensors
                    image_list = self.image_client.get_image_from_sources(
                        self.image_source_names)

                    for img in image_list:
                        if not img.status == image_pb2.ImageResponse.STATUS_OK:

                            header = std_msgs.msg.Header()
                            header.stamp.secs = img.shot.sample.acquisition_time.seconds
                            header.stamp.nsecs = img.shot.sample.acquisition_time.nanos
                            header.frame_id = img.source.name

                            # Make Image component of ImageCapture
                            i = sensor_msgs.msg.Image()
                            i.header = header
                            i.width = img.shot.image.cols
                            i.height = img.shot.image.rows
                            i.data = img.shot.image.data
                            
                            # Make Transform component of ImageCapture
                            ko_tform_body = geometry_msgs.msg.Transform()
                            ko_tform_body.translation = img.ko_tform_body.position
                            ko_tform_body.rotation = img.ko_tform_body.rotation
                            
                            # Populate ImageCapture msg
                            image_capture = spot_ros_msgs.msg.ImageCapture()
                            image_capture.image = i
                            image_capture.ko_tform_body = ko_tform_body

                            image_pub.publish(image_capture)

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
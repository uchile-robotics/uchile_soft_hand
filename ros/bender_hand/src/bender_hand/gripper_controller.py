#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

# Thread
from threading import Thread
# ROS
import rospy
import actionlib
# Msgs
from std_msgs.msg import UInt16
from sensor_msgs.msg import JointState
from control_msgs.msg import (GripperCommand, GripperCommandAction,
    GripperCommandFeedback, GripperCommandResult)
# Dynamic reconfigure
from dynamic_reconfigure.server import Server as DynamicReconfServer
from bender_gripper.cfg import GripperParamsConfig

from dynamixel_driver.dynamixel_const import DXL_ALARM_SHUTDOWN, DXL_OVERLOAD_ERROR

from bender_hand.hand_interface import HandInterface
from control_util.pid import PID

class GripperActionController():
    def __init__(self, controller_namespace, controllers):
        self.update_rate = 1000
        self.controller_namespace = controller_namespace
        self.joint_names = [ctrl.joint_name for ctrl in controllers]
        # Get joint to controller dict
        self.joint_to_controller = {}
        for ctrl in controllers:
            self.joint_to_controller[ctrl.joint_name] = ctrl
        # Get port to joints dicts
        self.port_to_joints = {}
        for ctrl in controllers:
            if ctrl.port_namespace not in self.port_to_joints:
                self.port_to_joints[ctrl.port_namespace] = []
            self.port_to_joints[ctrl.port_namespace].append(ctrl.joint_name)
        # Get get port to io (DynamixelIO)
        self.port_to_io = {}
        for ctrl in controllers:
            if ctrl.port_namespace in self.port_to_io:
                continue
            self.port_to_io[ctrl.port_namespace] = ctrl.dxl_io
        self.joint_states = dict(zip(self.joint_names,
            [ctrl.joint_state for ctrl in controllers]))
        self.num_joints = len(self.joint_names)
        self.joint_to_idx = dict(zip(self.joint_names, range(self.num_joints)))
        # Base msgs
        # Feedback and result base msg
        self.feedback = GripperCommandFeedback()
        self.result = GripperCommandResult()
        # Joint state base msg
        self.joint_state = JointState()
        self.joint_state.name = self.joint_names
        self.joint_state.position = [0.0]*self.num_joints
        self.joint_state.velocity = [0.0]*self.num_joints
        self.joint_state.effort = [0.0]*self.num_joints
        # Current max effort
        self.current_goal = 0.0
        self.current_effort = 1.0
        # Soft sensor
        self.left_side_pressure = UInt16()
        self.right_side_pressure = UInt16()

    def initialize(self):
        # Get controller parameters
        ns = self.controller_namespace + '/gripper_action_node' # Namespace
        # Parameters
        # Topic name for publish joint_states
        self.joint_states_topic = rospy.get_param(ns + '/joint_states_topic',
            '/joint_states')
        # Goal position tolerance in rads
        self.goal_tolerance = rospy.get_param(ns + '/goal_tolerance', 0.01)
        # Max allowed effort for actions [0 - 1023]
        self.max_effort = int(abs(rospy.get_param(ns + '/max_effort', 900)))
        # Effort tolerance [0 - 1023]
        self.effort_tolerance = abs(rospy.get_param(
            ns + '/effort_tolerance', 100))
        # Stall timeout in seg
        self.stall_timeout = rospy.get_param(ns + '/stall_timeout', 1.0)
        #print 'timeout %d' % self.stall_timeout
        # Stalled velocity in rad/s
        self.stalled_velocity_threshold = rospy.get_param(
            ns + '/stalled_velocity_threshold', 0.1)
        # State update rate for joint state s
        self.state_update_rate = rospy.get_param(
            ns + '/state_update_rate', 10)
        # Dynamic reconfigure server, load dinamic parameters
        self.reconfig_server = DynamicReconfServer(GripperParamsConfig,
            self.update_params)
        # Check motor parameters
        if not self.set_motor_config():
            return False

        # Soft sensors
        self.soft_sensor_id = rospy.get_param(
            ns + 'soft_sensor/id', 36)
        self.soft_sensor_io = self.find_soft_sensor(self.soft_sensor_id)
        if not self.soft_sensor_io:
            return False

        return True

    def start(self):
        self.running = True
        self.last_movement_time = rospy.Time.now()
        # Soft sensor interface
        self.soft_sensor = HandInterface(self.soft_sensor_io, self.soft_sensor_id)
        self.left_side_pressure_pub = rospy.Publisher(self.controller_namespace + '/left_side_pressure', UInt16, queue_size=50)
        self.right_side_pressure_pub = rospy.Publisher(self.controller_namespace + '/right_side_pressure', UInt16, queue_size=50)
        self.left_pid = PID(kp=0.5)
        self.right_pid = PID(kp=0.5)
        # Joint state publisher
        self.joint_states_pub = rospy.Publisher(self.joint_states_topic,
            JointState, queue_size=20)
        # Action feedback publisher
        self.state_pub = rospy.Publisher(self.controller_namespace + '/state',
            GripperCommandFeedback, queue_size=20)
        self.action_server = actionlib.SimpleActionServer(
            self.controller_namespace + '/gripper_action', GripperCommandAction,
            execute_cb=self.process_action, auto_start=False)
        self.action_server.start()
        Thread(target=self.update_state).start()

    def find_soft_sensor(self, id):
        for port, io in self.port_to_io.iteritems():
            for attempt in range(3):
                if io.ping(id):
                    rospy.loginfo("Tactile sensor found in \"{}\" with id: {}".format(port, id))
                    return io
        rospy.logerr("Tactile sensor not found, using id: {}".format(port, id))
        return None

      
    def update_params(self, config, level):
        # Update velocity
        self.velocity = config.gripper_vel
        rospy.loginfo('Gripper velocity setting at {:.2f}'.format(
            config.gripper_vel))
        return config

    def stop(self):
        self.running = False

    def hold_position(self, effort):
        rospy.loginfo('Holding current position with effort \
            {:.2f}'.format(effort))
        # Get current position
        pos = list()
        for joint in self.joint_names:
            pos.append(self.joint_states[joint].current_pos)
        # Send command with effort + tolerance for avoid torque limit problem
        self.pos_torque_command(pos, effort + self.effort_tolerance)

    def set_motor_config(self):
        # Check motor config
        for joint, controller in self.joint_to_controller.iteritems():
            motor_id = controller.motor_id
            # Read alarm shutdown
            try:
                current_config = controller.dxl_io.read(motor_id, DXL_ALARM_SHUTDOWN, 1)[-2]
            except:
                return False
            rospy.logdebug("Current motor config: {0:b}".format(current_config))
            # Check config
            if not current_config & DXL_OVERLOAD_ERROR:
                rospy.loginfo("Updating current motor config")
                updated_config = current_config | DXL_ALARM_SHUTDOWN
                try:
                    controller.dxl_io.write(motor_id, DXL_ALARM_SHUTDOWN, [updated_config])
                except:
                    return False
        return True

    def pos_torque_command(self, position, effort):
        # Multipacket
        multi_packet_torque = dict()
        multi_packet_position = dict()
        raw_effort = int(1024 * effort)
        # Set torque limit
        for port, joints in self.port_to_joints.items():
            vals_torque = list()
            vals_position = list()
            i = 0
            for joint in joints:
                # Get motor ID
                motor_id = self.joint_to_controller[joint].motor_id
                # Get the raw position (encoder ticks)
                pos = self.joint_to_controller[joint].pos_rad_to_raw(position)
                spd = self.joint_to_controller[joint].spd_rad_to_raw(self.velocity)
                # Create effort command
                vals_torque.append((motor_id, raw_effort))
                # Create position command
                vals_position.append((motor_id, pos, spd))
                i += 1
            # Add command to multipacket
            multi_packet_torque[port] = vals_torque
            multi_packet_position[port] = vals_position

        # Send torque command
        for port, vals in multi_packet_torque.items():
            self.port_to_io[port].set_multi_torque_limit(vals)
        # Send position command
        for port, vals in multi_packet_position.items():
            self.port_to_io[port].set_multi_position_and_speed(vals)


    def process_action(self, goal):
        command = goal.command
        # @TODO Check limits of position using URDF info
        # Check max effort
        rospy.loginfo('Received new action with position={:.2f} effort={:.2f}'.format(
            command.position, command.max_effort))
        # Effort saturation
        effort = min(max(abs(command.max_effort), 0.1), 1.0)
        # Send Dynamixel command
        self.current_goal = command.position
        self.current_effort = effort
        self.pos_torque_command(self.current_goal, command.max_effort)
        # Wait @TODO
        rospy.sleep(0.1)
        self.last_movement_time = rospy.Time.now()
        # Check for success action
        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            # Check position
            reach_goal_tolerance = True
            mean_position = 0 # Mean position
            mean_effort = 0 # Mean effort
            for joint, joint_state in self.joint_states.items():
                mean_position += joint_state.current_pos
                mean_effort += abs(joint_state.load)
                #print 'goal tolerance %f' % abs(joint_state.current_pos - command.position)
                # Check postion tolerance
                if abs(joint_state.current_pos - command.position) > self.goal_tolerance:
                    reach_goal_tolerance = False
            mean_position /= self.num_joints
            mean_effort /= self.num_joints
            # Send for goal tolerance
            if reach_goal_tolerance:
                self.result.position = mean_position
                self.result.effort = mean_effort
                self.result.stalled = False
                self.result.reached_goal = True
                # Result msg
                msg = 'Execution successfully, goal position reached with \
                position={:.2f} effort={:.2f}'.format(mean_position, mean_effort)
                rospy.loginfo(msg)
                self.action_server.set_succeeded(result=self.result, text=msg)
                break
            else:
                # Check for stall velocity
                reach_stall_velocity = True
                for joint, joint_state in self.joint_states.items():
                    if (abs(joint_state.velocity) > self.stalled_velocity_threshold):
                        reach_stall_velocity = False

                if not reach_stall_velocity:
                    self.last_movement_time = rospy.Time.now()
                else:
                    # Gripper stalled
                    # Check for torque limit
                    reach_stall_effort = False
                    for joint in self.joint_names:
                        if (abs(self.joint_states[joint].load) > effort - self.effort_tolerance):
                            reach_stall_effort = True
                    if reach_stall_effort:
                        self.result.position = mean_position
                        self.result.effort = mean_effort
                        self.result.stalled = True
                        self.result.reached_goal = False
                        # Result msg
                        msg = 'Execution stalled, position={:.2f} \
                            effort={:.2f}'.format(mean_position, mean_effort)
                        rospy.logwarn(msg)
                        self.action_server.set_succeeded(result=self.result, text=msg)
                        break
                    # Check timeout
                    elif ((rospy.Time.now()-self.last_movement_time).to_sec() > self.stall_timeout):
                        self.result.position = mean_position
                        self.result.effort = mean_effort
                        self.result.stalled = True
                        self.result.reached_goal = False
                        # Result msg
                        msg = 'Execution timeout, position={:.2f} \
                            effort={:.2f}'.format(mean_position, mean_effort)
                        rospy.logwarn(msg)
                        self.action_server.set_aborted(result=self.result, text=msg)
                        break
            #print 'timeout %f' % (rospy.Time.now() - self.last_movement_time).to_sec()
            rate.sleep()


    # State and feedback publishers in a thread
    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():
            # Joint state
            self.joint_state.header.stamp = rospy.Time.now()
            i = 0
            for joint_name in self.joint_names:
                # Joint state msg
                self.joint_state.position[i] = self.joint_states[joint_name].current_pos
                self.joint_state.velocity[i] = self.joint_states[joint_name].velocity
                self.joint_state.effort[i] = self.joint_states[joint_name].load
                # Gripper feedback
                self.feedback.position += self.joint_states[joint_name].current_pos
                self.feedback.effort += self.joint_states[joint_name].load
                i += 1

            # Mean position and effort
            self.feedback.position /= self.num_joints
            self.feedback.effort /= self.num_joints
            # Call publishers
            self.state_pub.publish(self.feedback)
            self.joint_states_pub.publish(self.joint_state)

            # Soft sensors
            self.left_side_pressure.data = self.hand_interface.read_tactil_sensor(1)
            self.right_side_pressure.data = self.hand_interface.read_tactil_sensor(2)

            # Publish soft sensor data
            self.left_side_pressure_pub.publish(self.left_side_pressure)
            self.right_side_pressure_pub.publish(self.right_side_pressure)

            rate.sleep()

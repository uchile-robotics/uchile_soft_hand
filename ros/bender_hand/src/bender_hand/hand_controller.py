#!/usr/bin/python

__author__ = 'gdiaz'

import rospy
import random

from threading import Thread

from std_msgs.msg import UInt16, String

# Use HW interface
from uchile_soft_hand.hand_interface import HandInterface

class HandController(object):

    DEV_ID = 36
    def __init__(self, dxl_io, controller_namespace, port_namespace):
        """
        Provides a high level interface over ROS for reading tactil-sensor data from Bender soft hand.
        It use methods provided by HandInterface class (non ROS hardware interface). See Documentation.
        """
        # Only argument stuff
        self.running = False
        self.dxl_io = dxl_io

        self.controller_namespace = controller_namespace
        self.port_namespace = port_namespace

        self.state_update_rate = rospy.get_param(self.controller_namespace + '/rate', 2)
        rospy.logwarn("Using rate: {}".format(self.state_update_rate))
        self.device_id = rospy.get_param(self.controller_namespace + '/id', 1)
        rospy.logwarn("Using id: {}".format(self.device_id))
        self.hand_interface = HandInterface(dxl_io, dev_id=self.device_id)
        self.left_side_pressure = UInt16()
        self.right_side_pressure = UInt16()
        
    def initialize(self):
        self.left_side_pressure.data = 0
        return True

    def start(self):
        # Create subs, services, publishers, threads
        self.running = True
        # subscribers
        self.command_sub = rospy.Subscriber(self.controller_namespace + '/soft_hand_cmd', String, self.process_command)

        # publishers
        self.left_side_pressure_pub = rospy.Publisher(self.controller_namespace + '/left_side_pressure', UInt16, queue_size=50)
        self.right_side_pressure_pub = rospy.Publisher(self.controller_namespace + '/right_side_pressure', UInt16, queue_size=50)
        Thread(target=self.update_state).start()

    def stop(self):
        self.running = False
        self.command_sub.unregister()
        self.left_side_pressure_pub.unregister()

    def process_command(self, msg):
        rospy.logwarn("Not implemented yet:")

    def update_state(self):
        rate = rospy.Rate(self.state_update_rate)
        while self.running and not rospy.is_shutdown():

            #update sensors data
            self.left_side_pressure.data = self.hand_interface.read_tactil_sensor(1)
            self.right_side_pressure.data = self.hand_interface.read_tactil_sensor(2)

            #publish data
            self.left_side_pressure_pub.publish(self.left_side_pressure)
            self.right_side_pressure_pub.publish(self.right_side_pressure)
            rate.sleep()
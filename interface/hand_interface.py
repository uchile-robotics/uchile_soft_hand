#!/usr/bin/python

import sys
import time
import rospy
from dynamixel_driver.dynamixel_io import DynamixelIO

# NON ROS HARDWARE INTERFACE

"""The DynamixelInterface class provides low-level methods to control arduino devices
   using Dynamixel protocol, provided by DynamixelIO class.
IMPORTANT: Modify only if you are sure of hardware specifications. See Documentation"""

# MMap position for commands (mem Addrs)

TACTIL1_ADDR = 6
TACTIL2_ADDR = 7

# Sensor commands (not addr)
CMD1 = 1
CMD2 = 2

class HandInterface(object):
    def __init__(self, dxl_io, dev_id = 1):
        self.dxl = dxl_io
        self.id = dev_id
        self.state = [0]

    def write_addr(self, addr, value):
        result = []
        try:
            result = self.dxl.write(self.id, addr, [value])
        except Exception as e:
            rospy.logwarn('Exception thrown while writing addres %d, value %d' % (addr, value))
        return result

    def ping(self):
        result = []
        try:
            result = self.dxl.ping(self.id)
        except Exception as e:
            rospy.logwarn('Exception thrown while pinging device %d - %s' % (self.id, e))
        return result

    def get_state(self, state_variable):
        result = []
        try:
            result = self.dxl.read(self.id, state_variable, 1)
        except Exception as e:
            rospy.logwarn('Exception thrown while writing addres %d' % (state_variable))
            # return e
            self.state = result[5]
        return self.state

    def change_id(self, new_id):
        if new_id<32:
            print 'Not Allow ID:%d. Must be greater than 31' % (new_id)
            return
        self.write_addr(3, new_id)

    def read_tactil_sensor(self, id):
        if id == 1:
            return self.get_state(TACTIL1_ADDR)
        elif id == 2:
            return self.get_state(TACTIL2_ADDR)
        else:
            print 'Not Allow ID:%d. Must be 1 or 2' % (id)

if __name__ == '__main__':
    DEV_ID = 36
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 200000)
    hand = HandInterface(dxl, dev_id = DEV_ID)
    print hand.ping()

    while True:
        t1 = hand.read_tactil_sensor(1)
        t2 = hand.read_tactil_sensor(2)
        print 'Tactil sensor 1:%d' %(t1)
        print 'Tactil sensor 2:%d' %(t2)
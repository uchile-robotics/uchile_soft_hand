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

TACTIL1_ADDR_LB = 6
TACTIL1_ADDR_HB = 7
TACTIL2_ADDR_LB = 8
TACTIL2_ADDR_HB = 9

# Sensor commands (not addr)
CMD1 = 1
CMD2 = 2

class HandInterface(object):
    def __init__(self, dxl_io, dev_id = 1):
        self.dxl = dxl_io
        self.id = dev_id
        self.state = 0

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
            return 0
        return result[5]

    def change_id(self, new_id):
        if new_id<32:
            print 'Not Allow ID:%d. Must be greater than 31' % (new_id)
            return
        self.write_addr(3, new_id)

    def read_tactil_sensor(self, id):
        value_lb = 0
        value_hb = 0
        if id == 1:
            value_lb = self.get_state(TACTIL1_ADDR_LB)
            value_hb = self.get_state(TACTIL1_ADDR_HB)
        elif id == 2:
            value_lb = self.get_state(TACTIL2_ADDR_LB)
            value_hb = self.get_state(TACTIL2_ADDR_HB)
        else:
            print 'Not Allow ID:%d. Must be 1 or 2' % (id)
        return value_lb | (value_hb<<8)

if __name__ == '__main__':
    DEV_ID = 36
    dxl = DynamixelIO('/dev/ttyUSB0', baudrate = 1000000)
    hand = HandInterface(dxl, dev_id = DEV_ID)
    # print dxl.ping(DEV_ID)
    # hand.change_id(36)

    while True:
        t1 = hand.read_tactil_sensor(1)
        t2 = hand.read_tactil_sensor(2)
        print 'Tactil sensor L:%d \t\t\t Tactil sensor R:%d' %(t1, t2)
        time.sleep(0.1)
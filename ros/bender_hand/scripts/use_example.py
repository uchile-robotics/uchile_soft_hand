#!/usr/bin/env python

####################################
##   This file shows the basic    ##
## usage of the node 'hand.py'.   ##
####################################

# R O S
import roslib
roslib.load_manifest('bender_hand')
import rospy

# P y t h o n 
import sys
import math
import time

# Messages
from std_msgs.msg import Uint16, String

rospy.init_node("test_hand")

# TO DO
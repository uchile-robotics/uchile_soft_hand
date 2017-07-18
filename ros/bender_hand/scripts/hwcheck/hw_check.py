#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import sys
from uchile_util.syscheck import SystemCheck, SystemCheckTask, FileCheckTask
from bender_fieldbus.check import DynamixelCheck

def hand_check():
    hand = SystemCheck("hand")
    hand.add_child(FileCheckTask('/dev/bender/r_port'))
    hand.add_child(DynamixelCheck('/dev/bender/r_port', 200000, [30,31,35]))
    return hand.check()

if __name__ == "__main__":
    if hand_check():
        sys.exit(0)
    sys.exit(1)
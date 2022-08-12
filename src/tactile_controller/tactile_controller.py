#!/usr/bin/env python

from __future__ import print_function
from select import select
import sys
import time
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import rospy
import RPi.GPIO as GPIO
# from msilib.schema import Control

import threading

import roslib
roslib.load_manifest('tactile_controller')


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


MUX_TABLE = {
#   CH EN S0 S1 S2 S3
    0: (0, 0, 0, 0),
    1: (1, 0, 0, 0),
    2: (0, 1, 0, 0),
    3: (1, 1, 0, 0),
    4: (0, 0, 1, 0),
    5: (1, 0, 1, 0),
    6: (0, 1, 1, 0),
    7: (1, 1, 1, 0),
    8: (0, 0, 0, 1),
    9: (1, 0, 0, 1),
    10: (0, 1, 0, 1),
    11: (1, 1, 0, 1),
    12: (0, 0, 1, 1),
    13: (1, 0, 1, 1),
    14: (0, 1, 1, 1),
    15: (1, 1, 1, 1),
}

RL0_GPIO = 21
RL1_GPIO = 20
RL2_GPIO = 16
RL3_GPIO = 26
RL4_GPIO = 19
RL5_GPIO = 13
RL6_GPIO = 6
RL7_GPIO = 12
RL8_GPIO = 5
RL9_GPIO = 0
RL10_GPIO = 1
RL11_GPIO = 7
RL12_GPIO = 8
RL13_GPIO = 11
RL14_GPIO = 9
RL15_GPIO = 25

RL_PINS = [RL0_GPIO, RL1_GPIO, RL2_GPIO, RL3_GPIO, RL4_GPIO, RL5_GPIO, RL6_GPIO, RL7_GPIO, RL8_GPIO, RL9_GPIO, RL10_GPIO, RL11_GPIO, RL12_GPIO, RL13_GPIO, RL14_GPIO, RL15_GPIO]

class Kontroller(object):
    def __init__(self):
        rospy.init_node('tactile_controller')
        self.rate = rospy.get_param("~rate", 12)
        self.thres = rospy.get_param("~threshould", 10)
        self.grid = OccupancyGrid()
        self.sub_grid = rospy.Subscriber("tk_map", OccupancyGrid, self.recive_map)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RL_PINS, GPIO.IN)
        print(RL_PINS)
        print("GPIO setup complete")
        #self.write()

    def recive_map(self, grid):
        self.grid = grid
        self.write()

    def write(self):
        i = 0
        for i in range(4):
            for j in range(4):
                idx = i*4 + j
                item = self.grid.data[idx]
                if item >= self.thres:
                    md = GPIO.OUT
                else:
                    md = GPIO.IN
                self.set_pin(j, i, md)

    def set_pin(self, i, j, state):
        idx = i*4 + j
        if idx >= 16 or idx <0:
            print("out of range pin idx")
            return
        GPIO.setup(RL_PINS[idx], state)

    def stop(self):
        print("\nShutdown gracefully")
        GPIO.setup(RL_PINS, GPIO.IN)
        GPIO.cleanup()
        print("GPIO uninitialize")
        
    def run(self):
        pass

if __name__ == "__main__":


    controller = Kontroller()

    rospy.on_shutdown(controller.stop)

    rospy.spin()


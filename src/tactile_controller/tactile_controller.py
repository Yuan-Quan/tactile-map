#!/usr/bin/env python

from __future__ import print_function
from select import select
import sys
import time
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
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

class ControllerThread(threading.Thread):
    def __init__(self, rate):
        super(ControllerThread, self).__init__()
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RL_PINS, GPIO.OUT)
        print("GPIO setup complete")

        GPIO.output(RL_PINS, 1)


        self.start()

    def set_on(self, idx):
        if idx < 0 or idx >= 16:
            print("invalid idx")
            return


    def update(self, data):
        pass

    def stop(self):
        GPIO.output(RL_PINS, 1)
        GPIO.cleanup()
        print("GPIO uninitialize")
        self.done = True
        self.update(None)
        self.join()
        exit()
        
    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            print("run!!!")
            self.condition.release()

            # write to gpio
            for i in range(16):
                self.set_on(i);
                time.sleep(0.01)

        # write stop message when thread exits.


if __name__ == "__main__":

    rospy.init_node('tactile_controller')

    repeat = rospy.get_param("~repeat_rate", 0.0)

    ctrl_thread = ControllerThread(repeat)

    try:
        while not rospy.is_shutdown():
            pass


    except Exception as e:
        print(e)
    finally:
        print("Shutdown gracefully")
        ctrl_thread.stop()
        exit()

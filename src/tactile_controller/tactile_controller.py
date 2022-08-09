#!/usr/bin/env python

from __future__ import print_function
from select import select
import sys
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
#   CH S0 S1 S2 S3
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

EN_GPIO = 7 # we use this pin, 
            # cuz when a Pi is first powered up, the first eight GPIOs have pull-ups enabled 

S0_GPIO = 11
S1_GPIO = 13
S2_GPIO = 15
S3_GPIO = 16

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

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(EN_GPIO, GPIO.OUT)
        GPIO.setup(S0_GPIO, GPIO.OUT)
        GPIO.setup(S1_GPIO, GPIO.OUT)
        GPIO.setup(S2_GPIO, GPIO.OUT)
        GPIO.setup(S3_GPIO, GPIO.OUT)
        print("GPIO setup complete")

        GPIO.output(S0_GPIO, 0)
        GPIO.output(S1_GPIO, 0)
        GPIO.output(S2_GPIO, 0)
        GPIO.output(S3_GPIO, 0)
        GPIO.output(EN_GPIO, 0)

        self.start()

    def update(self, data):
        pass

    def stop(self):
        GPIO.output(S0_GPIO, 0)
        GPIO.output(S1_GPIO, 0)
        GPIO.output(S2_GPIO, 0)
        GPIO.output(S3_GPIO, 0)
        GPIO.output(EN_GPIO, 1)
        GPIO.cleanup()
        print("GPIO uninitialize")
        self.done = True
        self.update(None)
        self.join()
        
    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            print("run!!!")
            self.condition.release()

            # write to gpio

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

#!/usr/bin/env python

from __future__ import print_function
from select import select
import sys
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
import rospy
from msilib.schema import Control

import threading

import roslib
roslib.load_manifest('teleop_twist_keyboard')


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}


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

        self.start()

    def update(self, data):
        pass

    def stop(self):
        self.done = True
        self.update(None)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            self.condition.release()

            # write to gpio

        # write stop message when thread exits.


if __name__ == "__main__":

    rospy.init_node('tactile_controller')

    repeat = rospy.get_param("~repeat_rate", 0.0)

    ctrl_thread = ControllerThread(repeat)

    try:
        pass
    except Exception as e:
        print(e)

    finally:
        ctrl_thread.stop()

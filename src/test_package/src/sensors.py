import os
import sys
import rospy
import numpy as np
from abc import ABC, abstractmethod

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *


# Basic class for sensor
class Sensor(object):
    def __init__(self):
        self.data = None

        self.x = np.zeros((9, 9))  # [x, y, z, vx, vy, vz, r, p ,y]
        self.P = 0.1 * np.eye(9)
        self.H = np.zeros((9, 9)).astype(int)

        self.observers = []

    def register_observer(self, observer_callback):
        self.observers.append(observer_callback)

    def notify_observers(self):
        """Call all observers function when this function is called"""
        for observer in self.observers:
            observer()


class RealSenseD455_RGB(Sensor):
    def __init__(self):
        super().__init__()

        self.data = None

        self.subscription = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.update
        )

    def update(self, msg: Image):
        """This callback function input sensor_msgs.msg.Image and update 2d RGB image"""
        self.data = msg

        self.notify_observers()


class RealSenseD455_D(Sensor):
    def __init__(self):
        super().__init__()

        self.data = None

        self.subscription = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.update
        )

    def update(self, msg: Image):
        """This callback function input sensor_msgs.msg.Image and update 2d RGB image"""
        self.data = msg

        self.notify_observers()

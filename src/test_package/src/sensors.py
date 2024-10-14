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
from custom_msgs.msg import DifferentialEncoder


# Basic class for sensor
class Sensor(object):
    def __init__(self):
        self.data = None

        self.x = np.zeros((9, 9))  # [x, y, z, vx, vy, vz, r, p ,y]
        self.P = np.zeros((9, 9))
        self.H = np.zeros((9, 9)).astype(int)
        self.k = np.zeros((9, 9))

        self.observers = []

    @abstractmethod
    def update(self, msg):
        """This function is called when sensor data is updated"""
        pass

    def register_observer(self, observer_callback):
        self.observers.append(observer_callback)

    def notify_observers(self):
        """Call all observers function when this function is called"""
        for observer in self.observers:
            observer()


class RealSenseD455_RGB(Sensor):
    def __init__(self, topic: str):
        super().__init__()

        self.data = None

        self.subscription = rospy.Subscriber(topic, Image, self.update)

    def update(self, msg: Image):
        """This callback function input sensor_msgs.msg.Image and update 2d RGB image"""
        self.data = msg

        self.notify_observers()


class RealSenseD455_D(Sensor):
    def __init__(self, topic: str):
        super().__init__()

        self.data = None

        self.subscription = rospy.Subscriber(topic, Image, self.update)

    def update(self, msg: Image):
        """This callback function input sensor_msgs.msg.Image and update 2d RGB image"""
        self.data = msg

        self.notify_observers()


class Encoder(Sensor):
    def __init__(self, topic: str):
        super().__init__()

        self.data = None

        self.subscription = rospy.Subscriber(topic, DifferentialEncoder, self.update)

    def update(self, msg: DifferentialEncoder):
        """This callback function input custom_msgs.msg.DifferentialEncoder and update encoder data"""
        self.data = msg

        self.notify_observers()


class VIO(Sensor):
    def __init__(self, topic: str):
        super().__init__()

        self.data = None

        self.subscription = rospy.Subscriber(topic, Odometry, self.update)

    def update(self, msg: Odometry):
        """This callback function input nav_msgs.msg.Odometry and update VIO data"""
        self.data = msg

        self.notify_observers()

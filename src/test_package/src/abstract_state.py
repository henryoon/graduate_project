#!/usr/bin/python3

import os
import sys
import rospy
from abc import ABC, abstractmethod

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *


class State(ABC):
    """State of End-Effector"""

    def __init__(self, topic: str):
        self._observers = []

    @abstractmethod
    def odom_callback(self, msg: Odometry):
        """This callback function input nav_msgs.msg.Odometry and update robot's state"""
        if not isinstance(msg, Odometry):
            rospy.logerr("Received message is not of type Odometry")
            return

        self.notify_observers()

    @abstractmethod
    def register_observer(self, observer_callback: function):
        self._observers.append(observer_callback)

    @abstractmethod
    def notify_observers(self):
        """Call all observers function when this function is called"""
        for observer in self._observers:
            observer()


class TargetObject(ABC):
    """{
        pose:
            position:
                x: float64
                y: float64
                z: float64
            orientation:
                x: float64
                y: float64
                z: float64
                w: float64
        scale:
            x: float64
            y: float64
            z: float64
        id: uint8
    }"""

    # TODO: Replace this abstract class to ros message
    def __init__(self, **kargs):
        self.pose = kargs.get("pose", Pose())
        self.scale = kargs.get("scale", Vector3())
        self.id = kargs.get("id", 0)

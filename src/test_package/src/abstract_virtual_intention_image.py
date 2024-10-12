#!/usr/bin/python3

import os
import sys
import rospy
import numpy as np
from abc import ABC, abstractmethod

# Messages
from sensor_msgs.msg import Image
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *

# Custrom
from abstract_state import State, TargetObject
from abstract_real_intention_image import FState, FOV, RealIntention


class VFOV(FOV):
    def __init__(self, state: FState):
        super().__init__(state)

        self.state.register_observer(self.image_callback)

    @abstractmethod
    def image_callback(self):
        """Calculate projecting image via state"""
        pass


class VirtualIntention(RealIntention):
    def __init__(self, state: FState, target_objects: list):
        self.state = state
        self.target_objects = target_objects

        self.fov = FOV(self.state)


def main():
    state = FState(topic="/odometry")
    target_objects = []

    real_intention = VirtualIntention(state, target_objects)

    while not rospy.is_shutdown():
        real_intention.run()
        rospy.spin()

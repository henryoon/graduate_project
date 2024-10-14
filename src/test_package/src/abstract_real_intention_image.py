#!/usr/bin/python3

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

# Custrom
from abstract_state import State, TargetObject
from sensors import RealSenseD455_D, RealSenseD455_RGB


class FState(State):
    def __init__(self, topic: str):
        super().__init__(topic)

        self.linear_acceleration = Vector3()

        self.register_observer(self.calculate_covariance)
        self.register_observer(self.calculate_linear_acceleration)

    def calculate_covariance(self) -> np.array:
        """This function calculate covariance matrix from state"""
        pass

    def calculate_linear_acceleration(self) -> Vector3:
        """This function calculate tangent vector from state"""
        pass


class FOV(ABC):
    def __init__(
        self, state: FState, camera: RealSenseD455_RGB, depth_camera: RealSenseD455_D
    ):
        self.state = state
        self.camera = camera
        self.depth_camera = depth_camera

    @abstractmethod
    def project_image(self, state, image):
        """This function transform covariance area on image
        via state.covariance and state.tangent_vector"""
        pass

    @abstractmethod
    def check_overlay(self, target_object: TargetObject) -> float:
        """This function check overlayed area ratio between target object and FOV"""
        pass


class RealIntention(ABC):
    def __init__(self, state: FState, target_objects: list):
        self.state = state
        self.target_objects = target_objects

        self.fov = FOV(self.state)

    @abstractmethod
    def check_best_target_object(self) -> TargetObject:
        """This function check best target object to approach"""
        if len(self.target_objects) == 0:
            rospy.logwarn("No target object available to check.")
            return None

        best_target = max(
            self.target_objects, key=lambda obj: self.fov.check_overlay(obj)
        )
        return best_target

    @abstractmethod
    def run(self):
        """Publish best target object to approach"""
        pass


def main():
    state = FState(topic="/odometry")
    target_objects = []

    real_intention = RealIntention(state, target_objects)

    while not rospy.is_shutdown():
        real_intention.run()
        rospy.spin()

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


class FState(State):
    def __init__(self, topic: str):
        super().__init__(topic)

        self.covariance = self.calculate_covariance()
        self.tangent_vector = self.calculate_tangent_vector()

        self.register_observer(self.calculate_covariance)
        self.register_observer(self.calculate_tangent_vector)

    def calculate_covariance(self) -> np.array:
        """This function calculate covariance matrix from state"""
        pass

    def calculate_tangent_vector(self) -> Vector3:
        """This function calculate tangent vector from state"""
        pass


class FOV(ABC):
    def __init__(self, state: FState):
        self.state = state
        self.image = Image()

    @abstractmethod
    def image_callback(self, msg: Image):
        """This callback function input sensor_msgs.msg.Image and update 2d image"""
        if not isinstance(msg, Image):
            rospy.logerr("Received message is not of type Image")
            return

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
        pass

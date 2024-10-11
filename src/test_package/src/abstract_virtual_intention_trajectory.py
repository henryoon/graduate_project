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

# Custrom
from abstract_state import State, TargetObject


class Operator(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def view_callback(self, msg: Pose):
        """This callback function input geometry_msgs.msg.Pose and update operator's view pose"""
        pass


class Trajectory(ABC):
    def __init__(self, target_object: TargetObject, state: State, operator: Operator):
        self.target_object = target_object
        self.state = state
        self.operator = operator
        self.path = self.create_trajectory(self.state, self.target_object)

        self.similarity = 0.0

        self.state.register_observer(self.update_trajectory_similarity)

    def create_trajectory(self, state: State, target_object) -> Path:
        """Create trajectory via state and target_object"""
        pass

    @abstractmethod
    def update_trajectory_similarity(self):
        """Calculate similarity via state, path, and operator data"""
        pass


class VirtualIntention(ABC):
    def __init__(self, state: State, target_objects: list, operator: Operator):
        self.state = state
        self.operator = operator
        self.target_objects = target_objects
        self.trajectories = self.create_trajectories()

    @abstractmethod
    def create_trajectories(self) -> list:
        return [
            Trajectory(target_object, self.state, self.operator)
            for target_object in self.target_objects
        ]

    @abstractmethod
    def set_target_objects(self, target_objects: list):
        self.target_objects = target_objects

    @abstractmethod
    def reset(self, new_target_objects: list):
        """This callback function will be called whenever a task is finished. Regenerate new path to target object"""
        self.trajectories = self.create_trajectories()

    @abstractmethod
    def check_best_trajectory(self) -> Trajectory:
        """Find the trajectory instance that has the highest similarity among self.trajectories."""
        if len(self.trajectories) == 0:
            rospy.logwarn("No trajectories available to check.")
            return None

        best_trajectory = max(self.trajectories, key=lambda traj: traj.similarity)
        return best_trajectory

    @abstractmethod
    def run(self):
        """Main loop"""
        pass


def main():
    rospy.init_node("virtual_intention_node")

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        r.sleep()

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


class State(ABC):
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


class Trajectory(ABC):
    def __init__(self, target_object: TargetObject, state: State):
        self.target_object = target_object
        self.state = state
        self.path = self.create_trajectory(self.state, self.target_object)

        self.similarity = 0.0

        self.state.register_observer(self.update_trajectory_similarity)

    def create_trajectory(self, state: State, target_object) -> Path:
        """Create trajectory via state and target_object"""
        pass

    @abstractmethod
    def update_trajectory_similarity(self):
        """Calculate similarity via state and path"""
        pass


class VirtualIntention(ABC):
    def __init__(self, state: State, target_objects: list):
        self.state = state
        self.target_objects = target_objects
        self.trajectories = self.create_trajectories()

    @abstractmethod
    def create_trajectories(self) -> list:
        return [
            Trajectory(target_object, self.state)
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

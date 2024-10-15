#!/usr/bin/python3

import os
import sys
import roslib.exceptions
import roslib.packages
import rospy
import roslib
import numpy as np
from abc import ABC, abstractmethod

# Moveit!
import moveit_commander

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from custom_msgs.msg import *

# Custom
try:
    sys.path.append(roslib.packages.get_pkg_dir("test_package") + "/src")
    from abstract_state import State as AbstractState
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


class EndEffectorState(AbstractState):
    def __init__(self, topic):
        super().__init__(topic)

    def odom_callback(self, msg: Odometry):
        """This callback function input nav_msgs.msg.Odometry and update robot's state"""
        if not isinstance(msg, Odometry):
            rospy.logerr("Received message is not of type Odometry")
            return

        self.notify_observers()

    def register_observer(self, observer_callback):
        self._observers.append(observer_callback)

    def notify_observers(self):
        """Call all observers function when this function is called"""
        for observer in self._observers:
            observer()


def main():
    rospy.init_node("end_effector_state_node", anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("ARM")  # 플래닝 그룹 이름

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        ee_pose = move_group.get_current_pose().pose
        print(ee_pose)
        r.sleep()


if __name__ == "__main__":
    main()

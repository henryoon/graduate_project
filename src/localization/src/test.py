#!/usr/bin/python3

import os
import sys
import roslib.exceptions
import roslib.packages
import rospy
import roslib
import numpy as np
import cv2
from abc import ABC, abstractmethod

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *


class JointTest:
    def __init__(self):
        self.angle = 0.0

        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    def create_message(self):
        msg = JointState()

        msg.header = Header(stamp=rospy.Time.now())

        msg.name = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        msg.position = [
            0.0,  # shoulder_pan_joint
            self.angle,  # shoulder_lift_joint
            0.0,  # elbow_joint
            0.0,  # wrist_1_joint
            0.0,  # wrist_2_joint
            0.0,  # wrist_3_joint
        ]

        msg.velocity = [
            0.0,  # shoulder_pan_joint
            1.0,  # shoulder_lift_joint
            0.0,  # elbow_joint
            0.0,  # wrist_1_joint
            0.0,  # wrist_2_joint
            0.0,  # wrist_3_joint]
        ]
        msg.effort = []

        self.angle += 0.1

        return msg

    def run(self):
        self.pub.publish(self.create_message())


def main():
    rospy.init_node("test", anonymous=True)

    t = JointTest()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        t.run()

        r.sleep()


if __name__ == "__main__":
    main()

#!/usr/bin/python3

# Basic modules
import os
import sys
import rospy
import roslib
import numpy as np

# TF modules
import tf
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_multiply,
)

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from md.msg import md_robot_msg1


class WheelOdometry:
    def __init__(self):
        # TODO: Add subscriber topic name
        # ROS
        self.md_subscriber = rospy.Subscriber("", md_robot_msg1, self.md_callback)
        self.wheel_odom_publisher = rospy.Publisher(
            "/wheel_odometry/twist", Twist, queue_size=1
        )

        # Data
        self.data = Twist()

        # Parameters
        self.reduction_ratio = rospy.get_param(
            "~reduction_ratio", 30
        )  # Reduction ratio of the motord
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.05)
        self.wheel_base = rospy.get_param("~wheel_base", 0.3)

    def md_callback(self, msg: md_robot_msg1):
        motor1_rps = msg.motor1_rpm / (self.reduction_ratio * 60.0)
        motor2_rps = msg.motor2_rpm / (self.reduction_ratio * 60.0)

        motor1_mps = motor1_rps * 2 * np.pi * self.wheel_radius
        motor2_mps = motor2_rps * 2 * np.pi * self.wheel_radius

        v = (motor1_mps + motor2_mps) / 2
        w = (motor2_mps - motor1_mps) / self.wheel_base

        self.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

        # Publish
        self.publish(self.twist)

        return self.twist

    def publish(self, twist: Twist):
        self.wheel_odom_publisher.publish(twist)


def main():
    rospy.init_node("wheel_odometry_node")  # TODO: Add node name

    wheel_odometry = WheelOdometry()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

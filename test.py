#!/usr/bin/python3

import os
import sys
import roslib.exceptions
import roslib.packages
import rospy
import roslib
import numpy as np
from abc import ABC, abstractmethod


# Messages
import tf
from tf.transformations import (
    quaternion_multiply,
    quaternion_from_euler,
    euler_from_quaternion,
)
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from custom_msgs.msg import *


rospy.init_node("test_node")

tf_pub = tf.TransformBroadcaster()

d90 = np.pi / 2.0

quat = quaternion_from_euler(d90, -d90, 0)

print(quat)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    tf_pub.sendTransform(
        translation=(0, 0, 0),
        rotation=quat,
        time=rospy.Time.now(),
        parent="wrist_3_link",
        child="camera",
    )
    r.sleep()

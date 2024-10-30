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


def main():
    rospy.init_node("test_node")  # TODO: Add node name

    # rospy.spin()

    pub = rospy.Publisher(
        "test", PointStamped, queue_size=10
    )  # TODO: Add publisher topic

    msg = PointStamped()

    msg.header.frame_id = "tool0"
    msg.header.stamp = rospy.Time.now()

    msg.point.x = -1.0  # 왼쪽 1
    msg.point.y = -3.0  # 위로 3
    msg.point.z = 5.0  # 앞으로 5

    msg.header.stamp = rospy.Time.now()

    r = rospy.Rate(1)  # TODO: Add rate
    while not rospy.is_shutdown():

        pub.publish(msg)
        r.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as ros_ex:
        rospy.logfatal("ROS Interrupted.")
        rospy.logfatal(ros_ex)
    except Exception as ex:
        rospy.logfatal("Exception occurred.")
        rospy.logfatal(ex)
    finally:
        rospy.loginfo("Shutting down.")
        rospy.signal_shutdown("Shutting down.")
        sys.exit(0)

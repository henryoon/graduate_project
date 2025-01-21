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
from custom_msgs.msg import *


class WaveMaker:
    def __init__(self):
        self.sub = rospy.Subscriber("/wave0", WaveMsg, self.wave_callback)
        self.distance = 0.0
        self.signal = 0.0

        self.pub = rospy.Publisher("/wave_marker", Marker, queue_size=10)

    def wave_callback(self, msg: WaveMsg):
        self.distance = msg.distance
        self.signal = msg.signal

        self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "box"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "wave"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0.1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.0
        marker.scale.y = 0.0
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0 if self.signal < 20.0 else 0.0
        marker.color.b = 1.0 if self.signal < 20.0 else 0.0

        marker.text = f"{int(self.distance)}mm"

        self.pub.publish(marker)


def main():
    rospy.init_node("wave_maker")  # TODO: Add node name

    wave_maker = WaveMaker()

    rospy.spin()

    # r = rospy.Rate()  # TODO: Add rate
    # while not rospy.is_shutdown():
    #     r.sleep()


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

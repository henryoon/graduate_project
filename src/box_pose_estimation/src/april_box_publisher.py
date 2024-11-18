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

# Custom modules
import apriltag

try:
    sys.path.append(roslib.packages.get_pkg_dir("localization") + "/src")
    from april_tag_localization import AprilTagDetector
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


class AprilBoxPublisher:
    def __init__(self):
        self.april_detector = AprilTagDetector(camera_frame="EEF_camera_link", tag_size=0.04)
        self.april_box_pub = rospy.Publisher("/april_box", BoxObjectMultiArray, queue_size=1)

    def publish(self):
        box_object_multi_array = BoxObjectMultiArray()

        # print(self.april_detector.tags.tags)

        # return

        for tag in self.april_detector.tags.tags:
            box_object = BoxObject()
            box_object.header.stamp = rospy.Time.now()
            box_object.id = tag.id
            box_object.pose = tag.pose
            box_object_multi_array.boxes.append(box_object)

        self.april_box_pub.publish(box_object_multi_array)


def main():
    rospy.init_node("april_box_publisher_node")  # TODO: Add node name

    # rospy.spin()

    april_publisher = AprilBoxPublisher()
    

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():
        april_publisher.publish()
        r.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as ros_ex:
        rospy.logfatal('ROS Interrupted.')
        rospy.logfatal(ros_ex)
    except Exception as ex:
        rospy.logfatal('Exception occurred.')
        rospy.logfatal(ex)
    finally:
        rospy.loginfo('Shutting down.')
        rospy.signal_shutdown('Shutting down.')
        sys.exit(0)
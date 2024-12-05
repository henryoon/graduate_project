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


class TestFramePublisher:
    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

    def publish_fake_localization(self):
        self.tf_broadcaster.sendTransform(
            translation=(-0.0, 0, 0),
            rotation=(0, 0, 0, 1),
            time=rospy.Time.now(),
            child="base_link_virtual",
            parent="map"
        )


    def get_tf_between_base_eef(self):
        self.publish_fake_localization()

        if self.tf_listener.canTransform("base_link", "VGC", rospy.Time(0)):
            transform_data = self.tf_listener.lookupTransform("base_link", "VGC", rospy.Time(0))
            
            trans = transform_data[0]
            rot = transform_data[1]

            self.tf_broadcaster.sendTransform(
                translation=trans,
                rotation=rot,
                time=rospy.Time.now(),
                child="VGC_virtual",
                parent="base_link_virtual"
            )
        
        else:
            rospy.logwarn("Cannot get transform between base_link and VCG.")

    



def main():
    rospy.init_node("test_frame_publisher")  # TODO: Add node name

    # rospy.spin()
    tf_pub = TestFramePublisher()

    

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():
        tf_pub.get_tf_between_base_eef()
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
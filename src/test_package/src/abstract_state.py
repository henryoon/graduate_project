#!/usr/bin/python3

import os
import sys
from abc import ABC, abstractmethod
import rospy
import tf

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *


class Localization(ABC):
    def __init__(self, topic: str, frame_id: str):
        super().__init__()

        # Basic information
        self.topic = topic
        self.frame_id = frame_id

        # TF
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()


class State(Localization):
    def __init__(self, topic: str, frame_id: str):
        super().__init__(topic, frame_id)

        self.data = None
        self.transformed_pose = None

        if self.topic is not None:
            self.subscriber = rospy.Subscriber(
                self.topic, Odometry, self.odom_callback, queue_size=1
            )

    def odom_callback(self, msg: Odometry):
        if not isinstance(msg, Odometry):
            rospy.logerr("Invalid message type")

        self.data = msg
        self.get_target_frame_pose()

    def get_target_frame_pose(self):
        """Get position and orientation of specified frame_id"""
        # if self.data.header.frame_id == self.frame_id:
        #     self.transformed_pose = self.data.pose
        #     return self.transformed_pose

        if self.tf_listener.canTransform("map", self.frame_id, time=rospy.Time()):
            origin_pose = PoseStamped()
            origin_pose.header = Header(frame_id=self.frame_id, stamp=rospy.Time())
            origin_pose.pose.orientation.w = 1.0  # Base Pose

            transfromed_pose = self.tf_listener.transformPose("map", origin_pose)

            self.transformed_pose = transfromed_pose

            return self.transformed_pose
        else:
            rospy.logerr("Cannot get transform from map to {}".format(self.frame_id))
            return None

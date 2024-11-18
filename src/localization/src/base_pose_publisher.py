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


# Custom
try:
    sys.path.append(roslib.packages.get_pkg_dir("test_package") + "/src")
    from abstract_state import State
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


class BasePosePublisher():
    def __init__(self):
        self.state = State(topic="None", frame_id="base_link")

        self.pose_publisher = rospy.Publisher("/base_pose", PoseStamped, queue_size=1)

        self.i = 0.0

    def get_pose(self):
        base_frame_pose = self.state.get_target_frame_pose()
        return base_frame_pose

    def publish_pose(self):
        pose = self.get_pose()
        if pose is None:
            return

        # Publish the pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rospy.Time.now()
        
        position = pose.pose.position
        orientation = pose.pose.orientation

        # position.y *= -1
        position.z = 0.725
        position.z = 0.0

        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(quat)

        new_quat = quaternion_from_euler(0, -yaw - (np.pi / 2.0), 0.0)

        pose_msg.pose.position = position
        pose_msg.pose.orientation = Quaternion(x=new_quat[0], y=new_quat[1], z=new_quat[2], w=new_quat[3])

        self.pose_publisher.publish(pose_msg)

def main():
    rospy.init_node("base_pose_node")  # TODO: Add node name

    base_pose_publisher = BasePosePublisher()

    r = rospy.Rate(30)  # TODO: Add rate
    while not rospy.is_shutdown():
        
        base_pose_publisher.publish_pose()
        
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
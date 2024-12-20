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

try:
    sys.path.append(roslib.packages.get_pkg_dir("test_package") + "/src")
    from abstract_state import State
    from custom_filter import LowPassFilter
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


class TwistPublisher(object):
    def __init__(self):
        self.eef_state = State(topic=None, frame_id="wrist_3_link")
        self.vx_lpf = LowPassFilter(cutoff_freq=0.05, ts=1.0)
        self.vy_lpf = LowPassFilter(cutoff_freq=0.05, ts=1.0)
        self.vz_lpf = LowPassFilter(cutoff_freq=0.05, ts=1.0)
        
        self.last_eef_pose = None
        self.last_time = rospy.Time.now()
        
        self.twist_publisher = rospy.Publisher("/calculated_twist", Twist, queue_size=10)
        
        
    def update(self):
        current_pose = self.eef_state.get_target_frame_pose()
        current_time = rospy.Time.now()
        
        current_pose: PoseStamped
        self.last_eef_pose: PoseStamped
        
        if current_pose is None or self.last_eef_pose is None:
            rospy.logwarn("Pose Data is None")
            self.last_eef_pose = current_pose
            self.last_time = current_time
            return Twist()
        
        dt = (current_time - self.last_time).to_sec()
        
        dx = current_pose.pose.position.x - self.last_eef_pose.pose.position.x
        dy = current_pose.pose.position.y - self.last_eef_pose.pose.position.y
        dz = current_pose.pose.position.z - self.last_eef_pose.pose.position.z
        
        vx = (dx / dt)
        # self.vx_lpf.filter
        vy = (dy / dt)
        # self.vy_lpf.filter
        vz = (dz / dt)
        # self.vz_lpf.filter
        
        twist = Twist()
        twist.linear = Vector3(x=vx, y=vy, z=vz)
        
        self.last_eef_pose = current_pose
        self.last_time = current_time
        
        print(f"vx: {round(vx, 5)}, vy: {round(vy, 5)}, vz: {round(vz, 5)}")
        
        return twist
    
    def publish(self):
        twist = self.update()
        
        self.twist_publisher.publish(twist)
        


def main():
    rospy.init_node("eef_twist_publisher_by_dt_node")  # TODO: Add node name

    twist_publisher = TwistPublisher()

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():
        twist_publisher.publish()
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
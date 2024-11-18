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

# Custom
from sklearn.cluster import KMeans
from scipy.stats import multivariate_normal
from enum import Enum

try:
    sys.path.append(roslib.packages.get_pkg_dir("test_package") + "/src")
    from abstract_state import State
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


class BoxSubscriber:
    def __init__(self, topic: str):
        self.__sub = rospy.Subscriber(topic, BoxObjectMultiArray, self.callback)
        self.data = BoxObjectMultiArray()

    def callback(self, msg: BoxObjectMultiArray):
        self.data = msg


class EEF_Twist_Subscriber:
    def __init__(self, topic: str):
        self.__sub = rospy.Subscriber(topic, Twist, self.callback)
        self.data = Twist()

    def callback(self, msg: Twist):
        self.data = msg


class Circle:
    def __init__(self, x: float, y: float, r: float):
        self.x = x
        self.y = y
        self.r = r

    def is_valid(self, point: np.array):
        dist = np.linalg.norm(point - np.array([self.x, self.y]))
        return dist < self.r


class Plane:
    class Axis(Enum):
        X = 0
        Y = 1
        Z = 2

    def __init__(self, point: Point, axis: Axis):
        self.data = np.array([point.x, point.y, point.z])

        self.axis = axis

    def get_depth(self):
        return self.data[self.axis.value]


class Box(BoxObject):
    def __init__(self, box: BoxObject):
        self.header = box.header
        self.id = box.id
        self.pose = box.pose
        self.pdf = 0.0

    def get_pdf(self, rv, plane: Plane):
        pose = np.array([0.0, 0.0])
        if plane.axis == Plane.Axis.X:
            pose = np.array([self.pose.position.y, self.pose.position.z])

        elif plane.axis == Plane.Axis.Y:
            pose = np.array([self.pose.position.x, self.pose.position.z])

        elif plane.axis == Plane.Axis.Z:
            pose = np.array([self.pose.position.x, self.pose.position.y])

        pdf = rv.pdf(pose)
        self.pdf = pdf

        return pdf


class RealIntentionGaussian:
    def __init__(self):
        # Define EEF and Boxes
        self.eef_pose_state = State(topic=None, frame_id="VGC")
        self.eef_twist_state = EEF_Twist_Subscriber("/eef_twist")
        self.box_objects_sub = BoxSubscriber("/box_objects")

        # Variables
        self.data = np.array([])
        self.plane = Plane(point=Point(x=0.0, y=0.0, z=0.0), axis=Plane.Axis.X)

        L = 1.5
        self.circle = Circle(
            x=0.0, y=0.0, r=np.sqrt(L**2 - self.plane.get_depth() ** 2)
        )

        # Result
        self.mean = np.array([0.0, 0.0])
        self.cov = np.eye(2)
        self.boxes = []

        # ROS
        self.best_box_pub = rospy.Publisher("/best_box", BoxObject, queue_size=10)

    def get_projection(self):
        # Update EEF Pose
        eef_pose = self.eef_pose_state.get_target_frame_pose()  # PoseStamped
        eef_twist = self.eef_twist_state.data  # Twist

        # Calculate Projection
        v = 0.0  # Velocity
        if self.plane.axis == Plane.Axis.X:
            v = eef_pose.pose.position.x
        elif self.plane.axis == Plane.Axis.Y:
            v = eef_pose.pose.position.y
        elif self.plane.axis == Plane.Axis.Z:
            v = eef_pose.pose.position.z

        if not v < 0.0:
            return np.array([None, None, None])

        dt = self.plane.get_depth() / v

        current_pose = np.array(
            [
                eef_pose.pose.position.x,
                eef_pose.pose.position.y,
                eef_pose.pose.position.z,
            ]
        )
        current_pose = np.delete(current_pose, self.plane.axis.value)

        current_twist = np.array(
            [eef_twist.linear.x, eef_twist.linear.y, eef_twist.linear.z]
        )
        current_twist = np.delete(current_twist, self.plane.axis.value)

        # 2D Projection
        return current_pose + (current_twist * dt)

    def calculate_gaussian(self):
        # KMeans Clustering
        kmeans = KMeans(n_clusters=2, random_state=0)
        kmeans.fit(self.data)

        labels = kmeans.labels_
        centers = kmeans.cluster_centers_

        mean = np.array([0.0, 0.0])
        cov = np.eye(2)

        # If data is suitable for clustering
        if (
            self.data[labels == 0].shape[0] > 30
            and self.data[labels == 1].shape[0] > 30
        ):
            mean0 = centers[0]
            mean1 = centers[1]

            dist0 = np.sqrt(
                (self.circle.x - mean0[0]) ** 2 + (self.circle.y - mean0[1]) ** 2
            )
            dist1 = np.sqrt(
                (self.circle.x - mean1[0]) ** 2 + (self.circle.y - mean1[1]) ** 2
            )

            if dist0 > dist1:
                mean = mean0
                cov = np.cov(self.data[labels == 0], rowvar=False)
            else:
                mean = mean1
                cov = np.cov(self.data[labels == 1], rowvar=False)

        else:
            mean = np.mean(self.data, axis=0)
            cov = np.cov(self.data, rowvar=False)

        self.mean = mean
        self.cov = cov

        return mean, cov

    def update(self):
        new_projection = self.get_projection()

        # Append new projection
        if new_projection[0] is not None:
            if self.circle.is_valid(new_projection):
                self.data = np.append(self.data, new_projection)

                # Calculate Gaussian
                mean, cov = self.calculate_gaussian()
                rv = multivariate_normal(mean=mean, cov=cov)

                boxes = []
                # Update Box PDF
                for box in self.box_objects_sub.data.boxes:
                    box = Box(box)
                    pdf = box.get_pdf(rv, self.plane)
                    boxes.append(box)

                self.boxes = boxes

    def publish_best_box(self):
        box = max(self.boxes, key=lambda box: box.pdf)

        msg = BoxObject()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        msg.id = box.id
        msg.pose = box.pose

        self.best_box_pub.publish(msg)

    def get_pdfs(self):
        return [box.pdf for box in self.boxes]


def main():
    rospy.init_node("real_intention_gaussian_node")  # TODO: Add node name

    intention = RealIntentionGaussian()

    test_pub = rospy.Publisher("/test", Float32MultiArray, queue_size=10)

    r = rospy.Rate(30)  # TODO: Add rate
    while not rospy.is_shutdown():
        intention.update()
        test_pub.publish(Float32MultiArray(data=intention.get_pdfs()))
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

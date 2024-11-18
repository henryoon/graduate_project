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

        self.box_pub = rospy.Publisher("/april_box/visual", MarkerArray, queue_size=10)

    def callback(self, msg: BoxObjectMultiArray):
        new_data = self.data

        for new_box in msg.boxes:
            flag = True
            for box in new_data.boxes:
                if box.id == new_box.id:
                    new_data.boxes.remove(box)
                    new_data.boxes.append(new_box)
                    flag = False
                    break

            if flag:
                new_data.boxes.append(new_box)

        self.data = new_data

    def publish_box(self):
        marker_array = MarkerArray()

        for box in self.data.boxes:
            marker = Marker()
            marker.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
            marker.id = box.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = box.pose.position
            marker.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
            marker.scale = Vector3(x=0.01, y=0.25, z=0.15)
            marker.color = ColorRGBA(r=255 / 255, g=195 / 255, b=0, a=0.3)

            marker_array.markers.append(marker)

        self.box_pub.publish(marker_array)


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

        self.circle_pub = rospy.Publisher("/circle", Marker, queue_size=10)

    def is_valid(self, point: np.array):
        dist = np.linalg.norm(point - np.array([self.x, self.y]))
        return dist < self.r

    def publish_circle(self, x):
        msg = Marker()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        msg.id = 999
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD

        msg.pose.position.x = x
        msg.pose.position.y = self.x
        msg.pose.position.z = self.y

        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.1
        msg.scale.y = 2.0 * self.r
        msg.scale.z = 2.0 * self.r

        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 1.0

        msg.color.a = 0.3

        self.circle_pub.publish(msg)


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

    def get_pdf(self, mean, cov, plane: Plane):
        pose = np.array([0.0, 0.0])
        if plane.axis == Plane.Axis.X:
            pose = np.array([self.pose.position.y, self.pose.position.z])

        elif plane.axis == Plane.Axis.Y:
            pose = np.array([self.pose.position.x, self.pose.position.z])

        elif plane.axis == Plane.Axis.Z:
            pose = np.array([self.pose.position.x, self.pose.position.y])

        pdf = multivariate_normal.pdf(pose, mean, cov)
        self.pdf = pdf

        return pdf
    
    def get_marker(self):
        msg = Marker()

        msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        msg.id = self.id

        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.ADD

        msg.pose.position = self.pose.position
        msg.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        msg.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        msg.scale = Vector3(x=0.0, y=0.0, z=0.03)

        msg.text = str(round(self.pdf, 3))

        return msg


class RealIntentionGaussian:
    def __init__(self):
        # Define EEF and Boxes
        self.eef_pose_state = State(topic=None, frame_id="VGC")
        self.eef_twist_state = EEF_Twist_Subscriber("/calculated_twist")
        self.box_objects_sub = BoxSubscriber("/april_box")


        # Variables
        self.data = np.array([])
        self.plane = Plane(point=Point(x=0.8947, y=-0.1972, z=0.2043), axis=Plane.Axis.X)

        R = 1.2
        self.circle = Circle(
            x=0.0, y=0.0, r=np.sqrt(R**2 - self.plane.get_depth() ** 2)
        )

        # Result
        self.mean = np.array([0.0, 0.0])
        self.cov = np.eye(2)
        self.boxes = []

        # ROS
        self.best_box_pub = rospy.Publisher("/best_box", BoxObject, queue_size=10)
        self.cov_pub = rospy.Publisher("/covariance", PoseWithCovarianceStamped, queue_size=10)
        self.pdf_pub = rospy.Publisher("/pdf", MarkerArray, queue_size=10)
        self.right_controller_sub = rospy.Subscriber("/controller/right/joy", Joy, self.right_controller_callback)

    def reset(self):
        self.data = np.array([])
        self.boxes = []
        self.mean = np.array([0.0, 0.0])
        self.cov = np.eye(2)

    def right_controller_callback(self, msg: Joy):
        if len(msg.buttons) != 4:
            rospy.logwarn("Invalid Joy Message.")
            return
        
        if msg.buttons[2] == 1:
            self.reset()

    def publish_covariance(self, mean, cov):
        msg = PoseWithCovarianceStamped()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        msg.pose.pose.position.x = self.plane.data[0]
        msg.pose.pose.position.y = mean[0]
        msg.pose.pose.position.z = mean[1]

        msg.pose.pose.orientation.w = 1.0

        msg_cov = np.eye(6) * 0.0
        msg_cov[1][1] = cov[0][0]
        msg_cov[1][2] = cov[0][1]
        msg_cov[2][1] = cov[1][0]
        msg_cov[2][2] = cov[1][1]

        msg.pose.covariance = msg_cov.flatten().tolist()

        self.cov_pub.publish(msg)


    def get_projection(self):
        # Update EEF Pose
        eef_pose = self.eef_pose_state.get_target_frame_pose()  # PoseStamped
        eef_twist = self.eef_twist_state.data  # Twist

        if eef_pose is None:
            rospy.logwarn("EEF Pose is None.")
            return np.array([None, None])

        # Calculate Projection
        
        v = 0.0  # Velocity
        if self.plane.axis == Plane.Axis.X:
            v = eef_twist.linear.x
            L = eef_pose.pose.position.x
        elif self.plane.axis == Plane.Axis.Y:
            v = eef_twist.linear.y
            L = eef_pose.pose.position.y
        elif self.plane.axis == Plane.Axis.Z:
            v = eef_twist.linear.z
            L = eef_pose.pose.position.z

        if not (0.01 < v and v < 0.5):
            return np.array([None, None])

        dt = (self.plane.get_depth() - L) / v

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
        if self.data.shape[0] < 3:
            rospy.logwarn("Data is not enough.")
            return np.array([0.0, 0.0]), np.eye(2)
        
        mean = np.array([0.0, 0.0])
        cov = np.eye(2)
        
        # kmeans = KMeans(n_clusters=2, random_state=0, n_init=10)
        # kmeans.fit(self.data)

        # labels = kmeans.labels_
        # centers = kmeans.cluster_centers_


        # # If data is suitable for clustering
        # if (
        #     self.data[labels == 0].shape[0] > 30
        #     and self.data[labels == 1].shape[0] > 30
        # ):
        #     rospy.loginfo("Data is suitable for clustering.")
        #     mean0 = centers[0]
        #     mean1 = centers[1]

        #     dist0 = np.sqrt(
        #         (self.circle.x - mean0[0]) ** 2 + (self.circle.y - mean0[1]) ** 2
        #     )
        #     dist1 = np.sqrt(
        #         (self.circle.x - mean1[0]) ** 2 + (self.circle.y - mean1[1]) ** 2
        #     )

        #     if dist0 > dist1:
        #         mean = mean0
        #         cov = np.cov(self.data[labels == 0], rowvar=False)
        #     else:
        #         mean = mean1
        #         cov = np.cov(self.data[labels == 1], rowvar=False)

        # else:
        #     mean = np.mean(self.data, axis=0)
        #     cov = np.cov(self.data, rowvar=False)


        # Remove here when using KMeans
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
                # self.data = np.append(self.data, [new_projection], axis=0)
                self.data = np.append(self.data, [new_projection], axis=0) if self.data.size else np.array([new_projection])

                # Calculate Gaussian
                mean, cov = self.calculate_gaussian()
                # rv = multivariate_normal(mean=mean, cov=cov)

                boxes = []
                # Update Box PDF
                for box in self.box_objects_sub.data.boxes:
                    box = Box(box)
                    pdf = box.get_pdf(mean, cov, self.plane)
                    boxes.append(box)

                self.boxes = boxes

    def publish_best_box(self):
        if len(self.boxes) == 0:
            return

        box = max(self.boxes, key=lambda box: box.pdf)

        msg = BoxObject()
        msg.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        msg.id = box.id
        msg.pose = box.pose

        self.best_box_pub.publish(msg)

    def publish_pdf_marker(self, boxes):
        msg = MarkerArray()

        for box in boxes:
            msg.markers.append(box.get_marker())

        self.pdf_pub.publish(msg)


    def get_ids(self):
        return [box.id for box in self.boxes]

    def get_pdfs(self):
        return [box.pdf for box in self.boxes]


def main():
    rospy.init_node("real_intention_gaussian_node")  # TODO: Add node name

    intention = RealIntentionGaussian()

    test_pub = rospy.Publisher("/test", Float32MultiArray, queue_size=10)
    test_pose_pub = rospy.Publisher("/test_pose", PoseStamped, queue_size=10)

    r = rospy.Rate(10)  # TODO: Add rate
    for _ in range(10):
        intention.circle.publish_circle(intention.plane.get_depth())
        r.sleep()

    while not rospy.is_shutdown():
        intention.update()

        pdf = intention.get_pdfs()
        ids = intention.get_ids()

        print(ids)
        print(pdf)
        print("")

        test_pub.publish(Float32MultiArray(data=pdf)) # Publish PDFs
        intention.publish_best_box()
        intention.publish_covariance(intention.mean, intention.cov)
        intention.publish_pdf_marker(intention.boxes)

        intention.box_objects_sub.publish_box()

        r.sleep()


if __name__ == "__main__":
    try:
        main()
        pass
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

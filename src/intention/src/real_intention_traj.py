#!/usr/bin/python3

import os
import sys
import roslib.exceptions
import roslib.packages
import rospy
import roslib
import numpy as np
import cv2
from cv_bridge import CvBridge
from abc import ABC, abstractmethod

from tf.transformations import euler_from_quaternion, quaternion_from_euler
import moveit_commander
from scipy.interpolate import CubicSpline


# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from custom_msgs.msg import *


# Custom
try:
    sys.path.append(roslib.packages.get_pkg_dir("test_package") + "/src")
    from abstract_state import State as AbstractState
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


class EndEffectorState(AbstractState):
    def __init__(self, topic):
        super().__init__(topic)

        self.odom_sub = rospy.Subscriber(topic, Odometry, self.odom_callback)

    def odom_callback(self, msg):
        if not isinstance(msg, Odometry):
            rospy.logerr("Received message is not of type Odometry")
            return

        self.data = msg

        self.notify_observers()

    def register_observer(self, observer_callback):
        return super().register_observer(observer_callback)

    def notify_observers(self):
        return super().notify_observers()


class BoxesState:
    def __init__(self):
        self.data = BoxObjectMultiArray()

        self.boxes_sub = rospy.Subscriber(
            "/segmetation/boxes", BoxObjectMultiArray, self.boxes_callback
        )

    def boxes_callback(self, msg):
        if not isinstance(msg, BoxObjectMultiArray):
            rospy.logerr("Received message is not of type BoxObjectMultiArray")
            return

        rospy.loginfo("Successfully subscribe BoxObjectMultiArray")

        self.data = msg

        self.boxes_sub.unregister()


class ReadIntentionTraj:
    def __init__(self):
        self.boxes = BoxesState()
        self.end_effector = EndEffectorState("/odometry/end_effector")

        # Moveit
        self.move_group = moveit_commander.MoveGroupCommander(
            "ENDEFFECTOR"
        )  # 플래닝 그룹 이름
        self.path_pub = rospy.Publisher("/real_intention/path", Path, queue_size=1)
        self.path_marker_pub = rospy.Publisher(
            "/real_intention_trajectory", MarkerArray, queue_size=1
        )

    def traj_to_path(self):
        paths = self.calculate_traj()
        marker_paths = parse_marker(paths)

        if len(paths) > 0:
            self.path_pub.publish(paths[0])
            self.path_marker_pub.publish(marker_paths)

    def calculate_traj(self):
        paths = []

        if self.end_effector.data is None:
            return []

        for box in self.boxes.data.boxes:
            path = Path()
            path.header.frame_id = "map"
            path.header.stamp = rospy.Time.now()

            x = [
                self.end_effector.data.pose.pose.position.x,
                self.end_effector.data.pose.pose.position.x,
                box.pose.position.x,
            ]
            y = [
                self.end_effector.data.pose.pose.position.y,
                box.pose.position.y,
                box.pose.position.y,
            ]
            z = [
                self.end_effector.data.pose.pose.position.z,
                box.pose.position.z,
                box.pose.position.z,
            ]

            origin = euler_from_quaternion(
                [
                    self.end_effector.data.pose.pose.orientation.x,
                    self.end_effector.data.pose.pose.orientation.y,
                    self.end_effector.data.pose.pose.orientation.z,
                    self.end_effector.data.pose.pose.orientation.w,
                ]
            )
            rpy = [
                origin,
                [0.0, np.pi / 2.0, 0.0],
                [0.0, np.pi / 2.0, 0.0],
            ]

            roll = [r for r, _, _ in rpy]
            pitch = [p for _, p, _ in rpy]
            yaw = [y for _, _, y in rpy]

            # 각 축과 각도에 대해 Cubic Spline 보간기를 만듭니다.
            t = np.linspace(0, 1, len(x))  # 시간 축 (균일한 간격)
            spline_x = CubicSpline(t, x)
            spline_y = CubicSpline(t, y)
            spline_z = CubicSpline(t, z)
            spline_roll = CubicSpline(t, roll)
            spline_pitch = CubicSpline(t, pitch)
            spline_yaw = CubicSpline(t, yaw)

            # 매끄러운 경로를 생성합니다.
            t_new = np.linspace(0, 1, 10 * (len(x) - 1))

            for ti in t_new:
                xi = spline_x(ti)
                yi = spline_y(ti)
                zi = spline_z(ti)
                ori = spline_roll(ti)
                opi = spline_pitch(ti)
                oyi = spline_yaw(ti)

                quat = quaternion_from_euler(ori, opi, oyi)

                p = PoseStamped()
                p.header = Header(frame_id="map", stamp=rospy.Time.now())
                p.pose.position = Point(x=xi, y=yi, z=zi)
                p.pose.orientation = Quaternion(
                    x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                )

                path.poses.append(p)

            paths.append(path)
            # break

        return paths


def parse_marker(paths):
    marker_array = MarkerArray()

    for i, path in enumerate(paths):
        marker = Marker()
        marker.header = path.header
        marker.ns = "trajectory"
        marker.id = i
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005  # Line width
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for pose in path.poses:
            marker.points.append(pose.pose.position)

        marker_array.markers.append(marker)

    return marker_array


def main():
    rospy.init_node("read_intention_traj_node", anonymous=True)

    real_intention = ReadIntentionTraj()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():

        real_intention.traj_to_path()

        r.sleep()


if __name__ == "__main__":
    main()

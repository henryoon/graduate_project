#!/usr/bin/python3

import os
import sys
import roslib.exceptions
import roslib.packages
import rospy
import roslib
import numpy as np
from abc import ABC, abstractmethod

from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_inverse,
    quaternion_multiply,
)
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
    from abstract_state import State
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)


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


class Intention(ABC):
    def __init__(self, state_topic: str, target_object_topic: str):
        self.boxes = BoxesState()
        self.end_effector = State(state_topic, "wrist_3_link")

        self.target_object_pub = rospy.Publisher(
            target_object_topic, BoxObject, queue_size=1
        )

    @abstractmethod
    def calculate_confidence(self, objects):
        return 0.0

    @abstractmethod
    def recognize_target_object(self):
        target_object = max(
            self.boxes.data.boxes, key=lambda box: self.calculate_confidence(box)
        )
        return target_object


class Trajectory(object):
    def __init__(self, state: State, target_object: BoxObject):
        self.state = state
        self.target_object = target_object
        self.target_idx = 0

        self.path = self.create_path_from_state_and_target_object(
            self.state, self.target_object
        )
        self.marker_path = self.parse_marker(self.path)

    def reset(self):
        self.__init__(self.state, self.target_object)

    def calculate_confidence(self):
        self.target_idx = self.calculate_target_idx(last_target_idx=self.target_idx)

        target_err = self.calculate_target_err(target_idx=self.target_idx)

        return 1.0 / target_err

    def calculate_distance_err(self, pose: PoseStamped):
        dist = np.sqrt(
            (pose.pose.position.x - self.state.transformed_pose.pose.position.x) ** 2
            + (pose.pose.position.y - self.state.transformed_pose.pose.position.y) ** 2
            + (pose.pose.position.z - self.state.transformed_pose.pose.position.z) ** 2
        )

        return dist

    def calculate_orientation_err(self, pose: PoseStamped):
        trajectory_data = self.state.trajectory_data
        target_pose_data = pose.pose.orientation
        if trajectory_data is None:
            return 0.0

        trajectory_quat = [
            trajectory_data.x,
            trajectory_data.y,
            trajectory_data.z,
            trajectory_data.w,
        ]

        target_pose_quat = [
            target_pose_data.x,
            target_pose_data.y,
            target_pose_data.z,
            target_pose_data.w,
        ]

        dquat = quaternion_multiply(
            quaternion_inverse(trajectory_quat), target_pose_quat
        )

        r, p, y = euler_from_quaternion(dquat)

        r = self.normalize_angle(r)
        p = self.normalize_angle(p)
        y = self.normalize_angle(y)

        ori_err = abs(r) + abs(p) + abs(y)

        return ori_err

    def calculate_target_idx(self, last_target_idx: int):
        dists = []
        for i, pose in enumerate(self.path.poses):
            dist = self.calculate_distance_err(pose)
            dists.append(dist if i >= last_target_idx else float("inf"))

        return np.argmin(dists)

    def calculate_target_err(self, target_idx: int):
        target_pose = self.path.poses[target_idx]

        k_distance = 1.0
        k_orientation = 0.0

        distance_err = self.calculate_distance_err(target_pose)
        ori_err = self.calculate_orientation_err(target_pose)

        # TODO: Calculate the error
        return distance_err * k_distance + ori_err * k_orientation + 0.001  # test

    def create_path_from_state_and_target_object(
        self, state: State, target_object: BoxObject
    ):
        header = Header(frame_id="map", stamp=rospy.Time.now())
        path = Path(header=header)  # initialize Path message

        end_effector_pose = state.transformed_pose

        # If end effector pose is not available, return empty path
        if end_effector_pose is None:
            return path

        # X, Y, Z are [end_effector_pose, target_object_pose on YZ plane, target_object_pose]
        x = [
            end_effector_pose.pose.position.x,
            end_effector_pose.pose.position.x,
            target_object.pose.position.x,
        ]
        y = [
            end_effector_pose.pose.position.y,
            target_object.pose.position.y,
            target_object.pose.position.y,
        ]
        z = [
            end_effector_pose.pose.position.z,
            target_object.pose.position.z,
            target_object.pose.position.z,
        ]

        # Get the orientation of the end effector
        origin = euler_from_quaternion(
            [
                end_effector_pose.pose.orientation.x,
                end_effector_pose.pose.orientation.y,
                end_effector_pose.pose.orientation.z,
                end_effector_pose.pose.orientation.w,
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
            p.header = header
            p.pose.position = Point(x=xi, y=yi, z=zi)
            p.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

            path.poses.append(p)

        return path

    def parse_marker(self, path: Path):
        marker = Marker()
        marker.header = path.header
        marker.ns = "trajectory"
        marker.id = self.target_object.id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005  # Line width
        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for pose in path.poses:
            marker.points.append(pose.pose.position)

        return marker

    def change_makrer_color(self, r, g, b):
        self.marker_path.color.r = r
        self.marker_path.color.g = g
        self.marker_path.color.b = b

        return self.marker_path

    def normalize_angle(self, angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle


class RealIntentionTraj(Intention):
    def __init__(
        self,
        state_topic: str,
        target_object_topic: str = "/target_object/real_intention_traj",
    ):
        super().__init__(state_topic, target_object_topic)

        # Local variables
        self.trajs = []

        # Path publisher
        self.path_pub = rospy.Publisher("/path/target_object/traj", Path, queue_size=1)
        self.path_marker_pub = rospy.Publisher(
            "/paths/box_objects/traj", MarkerArray, queue_size=1
        )

        # Action listener
        self.service = rospy.Subscriber(
            "/real_intention_traj/generate_path", Empty, self.action_callback
        )

    # Update the path and markers : TEST
    def action_callback(self, msg):
        self.trajs = self.calculate_trajs()

        marker_array = MarkerArray()
        for traj in self.trajs:
            marker_array.markers.append(traj.marker_path)

        self.path_marker_pub.publish(marker_array)

    def calculate_trajs(self):
        trajs = []
        for box in self.boxes.data.boxes:
            traj = Trajectory(self.end_effector, box)
            trajs.append(traj)

        return trajs

    # Abstract methods
    def calculate_confidence(self, traj: Trajectory):
        return traj.calculate_confidence()

    # Abstract methods
    def recognize_target_object(self):
        if len(self.trajs) == 0:
            return None

        target_traj = max(self.trajs, key=lambda traj: self.calculate_confidence(traj))
        return target_traj

    def run(self):
        # self.end_effector.get_target_frame_pose()

        target_traj = self.recognize_target_object()

        if target_traj is None:
            rospy.logwarn("No target traj is recognized")
            return

        print("Target object id: ", target_traj.target_object.id)
        self.path_pub.publish(target_traj.path)


def main():
    rospy.init_node("read_intention_traj_node", anonymous=True)

    real_intention = RealIntentionTraj(
        state_topic="/odometry/test",
        target_object_topic="/target_object/real_intention_traj",
    )

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        real_intention.end_effector.get_target_frame_pose()  # TEST

        real_intention.run()

        r.sleep()


if __name__ == "__main__":
    main()

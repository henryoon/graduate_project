#!/usr/bin/python3

# Basic modules
import os
import sys
import rospy
import roslib
import numpy as np
import math as m

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


class EEF_State(object):
    def __init__(self, *args, **kwargs):
        self.tf_listener = tf.TransformListener()

        self.base_frame = kwargs.get("base_frame", "base_link")
        self.tool_frame = kwargs.get("tool_frame", "tool0")
        self.camera_frame = kwargs.get("camera_frame", "camera_link")
        self.box_topic = kwargs.get("box_topic", "/tf_C2O")

        self.box = None

        self.box_subscirber = rospy.Subscriber(
            self.box_topic, PoseArray, self.box_callback
        )

    def box_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            return None

        if self.tf_listener.canTransform(
            self.base_frame, self.camera_frame, rospy.Time(0)
        ):
            box_pose = PoseStamped()
            box_pose.header = Header(frame_id=self.camera_frame, stamp=rospy.Time(0))
            box_pose.pose = msg.poses[0]

            transformed_box_pose = self.tf_listener.transformPose(
                self.base_frame, box_pose
            )

            self.box = transformed_box_pose.pose

        else:
            rospy.logwarn(
                f"Cannot transform from {self.camera_frame} to {self.base_frame}."
            )

    def get_tf_origin(self):
        source_frame = self.tool_frame
        target_frame = self.base_frame

        base_pose = PoseStamped()

        base_pose.header = Header(frame_id=source_frame, stamp=rospy.Time(0))
        base_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        base_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        try:
            if self.tf_listener.canTransform(target_frame, source_frame, rospy.Time(0)):
                transformed_pose = self.tf_listener.transformPose(
                    target_frame, base_pose
                )

                return transformed_pose.pose

            else:
                rospy.logwarn(
                    f"Cannot transform from {source_frame} to {target_frame}."
                )

        except Exception as ex:
            rospy.logerr(ex)

        return None


class DataLogger:
    def __init__(self, log_file: str, log_path: str, *args, **kwargs):
        self.log_file = log_file
        self.log_path = log_path

        self.base_frame = kwargs.get("base_frame", "base_link")
        self.tool_frame = kwargs.get("tool_frame", "tool0")
        self.camera_frame = kwargs.get("camera_frame", "camera_link")
        self.box_topic = kwargs.get("box_topic", "/tf_C2O")

        self.hz = kwargs.get("hz", 10)

        self.last_pose = None
        self.last_heading = None
        self.last_time = None

        self.eef = EEF_State(
            base_frame=self.base_frame,
            tool_frame=self.tool_frame,
            camera_frame=self.camera_frame,
            box_topic=self.box_topic,
        )

    def calculate_eef_pose(self):
        return self.eef.get_tf_origin()

    def calculate_linear_velocity(self, last_pose: Pose, current_pose: Pose, dt: float):
        dx = current_pose.position.x - last_pose.position.x
        dy = current_pose.position.y - last_pose.position.y
        dz = current_pose.position.z - last_pose.position.z

        linear_velocity = Vector3(x=dx / dt, y=dy / dt, z=dz / dt)

        return linear_velocity

    def calculate_angular_velocity(
        self, last_pose: Pose, current_pose: Pose, dt: float
    ):
        last_orientation = [
            last_pose.orientation.x,
            last_pose.orientation.y,
            last_pose.orientation.z,
            last_pose.orientation.w,
        ]
        current_orientation = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w,
        ]

        d_orientation = quaternion_multiply(
            quaternion_inverse(last_orientation), current_orientation
        )

        roll, pitch, yaw = euler_from_quaternion(d_orientation)

        angular_velocity = Vector3(x=roll / dt, y=pitch / dt, z=yaw / dt)

        return angular_velocity

    def calculate_heading_vector(self, last_pose: Pose, current_pose: Pose):
        dx = current_pose.position.x - last_pose.position.x
        dy = current_pose.position.y - last_pose.position.y
        dz = current_pose.position.z - last_pose.position.z

        v = np.array([dx, dy, dz])
        v1 = np.array([1.0, 0.0, 0.0])  # 기준 벡터
        norm = np.linalg.norm([dx, dy, dz])

        if norm < 1e-8:  # 벡터의 크기가 0이면 회전이 의미가 없음
            return self.last_heading

        v2 = v / norm  # 주어진 벡터 정규화

        # 회전각 계산 (내적 이용)
        dot_product = np.dot(v1, v2)
        theta = np.arccos(np.clip(dot_product, -1.0, 1.0))  # 수치 안정성을 위한 clip

        # 회전축 계산 (외적 이용)
        u = np.cross(v1, v2)
        if np.linalg.norm(u) < 1e-8:  # 외적이 0이면 벡터가 평행한 경우
            u = np.array([0, 0, 0])  # 회전축이 의미가 없음
        else:
            u = u / np.linalg.norm(u)  # 회전축 정규화

        # 쿼터니언 계산
        qx = u[0] * np.sin(theta / 2)
        qy = u[1] * np.sin(theta / 2)
        qz = u[2] * np.sin(theta / 2)
        qw = np.cos(theta / 2)

        position = current_pose.position
        orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        pose = Pose(position=position, orientation=orientation)

        self.last_heading = pose

        return pose

    def calculate_eef_state(self):
        if self.last_pose is None:
            self.last_pose = self.calculate_eef_pose()
            self.last_time = rospy.Time.now()
            return None, None, None

        current_pose = self.calculate_eef_pose()
        current_time = rospy.Time.now()

        if current_pose is None:
            return None, None, None

        dt = (current_time - self.last_time).to_sec()
        linear_velocity = self.calculate_linear_velocity(
            self.last_pose, current_pose, dt
        )
        angular_velocity = self.calculate_angular_velocity(
            self.last_pose, current_pose, dt
        )
        heading_vector = self.calculate_heading_vector(self.last_pose, current_pose)

        self.last_pose = current_pose
        self.last_time = current_time

        twist = Twist(linear=linear_velocity, angular=angular_velocity)

        return current_pose, twist, heading_vector

    def run(self):
        r = rospy.Rate(self.hz)

        f = open(self.log_path + "/" + self.log_file, "w")

        f.write(
            "Time, Pose.x, Pose.y, Pose.z, Pose.qx, Pose.qy, Pose.qz, Pose.qw, Twist.linear.x, Twist.linear.y, Twist.linear.z, Twist.angular.x, Twist.angular.y, Twist.angular.z, Heading.qx, Heading.qy, Heading.qz, Heading.qw, Box.x, Box.y, Box.z, Box.qx, Box.qy, Box.qz, Box.qw\n".replace(
                " ", ""
            )
        )

        while not rospy.is_shutdown():
            eef_pose, eef_twist, heading = self.calculate_eef_state()
            box = self.eef.box

            if eef_pose is None or eef_twist is None or heading is None:
                warn_txt = "Cannot calculate: "
                warn_txt += "EEF Pose\t" if eef_pose is None else ""
                warn_txt += "EEF Twist\t" if eef_twist is None else ""
                warn_txt += "Heading\t" if heading is None else ""
                warn_txt += "Box\t" if box is None else ""

                rospy.logwarn(warn_txt)

                r.sleep()

                continue

            txt = ""

            # Time
            txt += f"{rospy.Time.now().to_sec()},"
            # Pose
            txt += f"{eef_pose.position.x}, {eef_pose.position.y}, {eef_pose.position.z}, {eef_pose.orientation.x}, {eef_pose.orientation.y}, {eef_pose.orientation.z}, {eef_pose.orientation.w},"
            # Twist
            txt += f"{eef_twist.linear.x}, {eef_twist.linear.y}, {eef_twist.linear.z}, {eef_twist.angular.x}, {eef_twist.angular.y}, {eef_twist.angular.z},"
            # Heading
            txt += f"{heading.orientation.x}, {heading.orientation.y}, {heading.orientation.z}, {heading.orientation.w},"
            # Box
            if box is not None:
                txt += f"{box.position.x}, {box.position.y}, {box.position.z}, {box.orientation.x}, {box.orientation.y}, {box.orientation.z}, {box.orientation.w},"
            else:
                txt += "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0".replace(" ", "")
            txt += "\n"

            f.write(txt.replace(" ", ""))

            rospy.loginfo("Logging Success! : {}".format(rospy.Time.now().to_sec()))

            r.sleep()


def main():
    rospy.init_node("data_logging_node")  # TODO: Add node name

    # rospy.spin()

    logger = DataLogger(
        log_file="data_log.csv",
        log_path="/home/catkin_ws/src",
        base_frame="base_link",
        tool_frame="camera",
        camera_frame="camera",
        box_topic="/tf_C2O",
        hz=10,
    )

    logger.run()


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

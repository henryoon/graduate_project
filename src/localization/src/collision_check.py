#!/usr/bin/python3

# Basic modules
import os
import sys
import rospy
import roslib
import numpy as np
import time

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

# MoveIt
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.srv import (
    GetStateValidity,
    GetStateValidityRequest,
    GetStateValidityResponse,
)


class EEFState(State):
    def __init__(self, topic: str, frame_id: str):
        super().__init__(topic, frame_id)

        self.last_pose = None
        self.velocity = Twist()
        self.last_time = rospy.Time.now()

    def odom_callback(self, msg: Odometry):
        if not isinstance(msg, Odometry):
            rospy.logerr("Invalid message type")

        self.data = msg

        self.get_target_frame_pose()

        self.linear_velocity = self.get_velocity(current_pose=self.transformed_pose)

    def get_velocity(self, current_pose: PoseStamped):
        last_pose = self.last_pose

        if current_pose is None or last_pose is None:
            rospy.logwarn("Invalid pose")
            self.last_pose = current_pose
            return Twist()

        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if dt == 0:
            rospy.logwarn("Time difference is zero")
            return Twist()

        # Calculate linear velocity
        dx = current_pose.pose.position.x - last_pose.pose.position.x
        dy = current_pose.pose.position.y - last_pose.pose.position.y
        dz = current_pose.pose.position.z - last_pose.pose.position.z

        linear_velocity = Vector3(x=dx / dt, y=dy / dt, z=dz / dt)

        # Calculate angular velocity
        current_quat = [
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w,
        ]
        last_quat = [
            last_pose.pose.orientation.x,
            last_pose.pose.orientation.y,
            last_pose.pose.orientation.z,
            last_pose.pose.orientation.w,
        ]

        # Calculate quaternion difference
        dquat = quaternion_multiply(quaternion_inverse(last_quat), current_quat)

        # Calculate euler angles
        dx, dy, dz = euler_from_quaternion(dquat)

        angular_velocity = Vector3(x=dx / dt, y=dy / dt, z=dz / dt)

        velocity = Twist(linear=linear_velocity, angular=angular_velocity)

        self.velocity = velocity
        self.last_time = current_time
        self.last_pose = current_pose

        return velocity


class CollisionCheck:
    def __init__(self):
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")

        self.right_controller_twist = Twist()
        self.right_controller_joy = Joy()
        self.right_controller_joy.buttons = [0, 0, 0, 0]

        self.scene.clear()

        # Add Collision Objects
        self.create_box(
            Pose(
                position=Point(x=0.0, y=0.7, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            (2.1, 1.0, 2.1),
        )

        # GetStateValidity 서비스 클라이언트 설정
        rospy.wait_for_service("/check_state_validity")
        self.state_validity_service = rospy.ServiceProxy(
            "/check_state_validity", GetStateValidity
        )

        # 충돌 여부를 판단하는 퍼블리셔 설정
        self.collision_pub = rospy.Publisher("/is_collision", Bool, queue_size=1)

    def joint_states_callback(self, msg: JointState):
        self.joint_states = msg

    def control_msg_callback(self, msg: Float32MultiArray):
        self.control_msg = msg.data

    def create_box(self, pose: Pose, size: tuple):
        # 사용자 정의 오브젝트 생성 & 추가
        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_link"  # 기준 좌표계 설정
        object_pose.pose = pose

        self.scene.add_box("box", object_pose, size=size)

    def request_state_validity(self):
        is_valid = False

        try:
            current_request = GetStateValidityRequest()
            current_request.group_name = "manipulator"
            current_request.robot_state = self.robot.get_current_state()

            current_valid = self.state_validity_service(current_request)

            is_valid = current_valid.valid

        except rospy.ServiceException as service_ex:
            rospy.logerr("Service call failed.")
            rospy.logerr(service_ex)
        except Exception as ex:
            rospy.logerr("Exception occurred.")
            rospy.logerr(ex)

        return is_valid


def main():
    rospy.init_node("collision_check_node")  # TODO: Add node name

    collision_check = CollisionCheck()

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():

        is_valid = collision_check.request_state_validity()
        # rospy.loginfo(f"State Validity: {is_valid}")

        collision_check.collision_pub.publish(is_valid)

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

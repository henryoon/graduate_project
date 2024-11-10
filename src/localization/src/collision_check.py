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
from moveit_msgs.msg import (
    PlanningScene,
    RobotState,
    CollisionObject,
    AttachedCollisionObject,
)
from moveit_msgs.srv import (
    GetStateValidity,
    GetStateValidityRequest,
    GetStateValidityResponse,
)


class EEFState(State):
    def __init__(self, topic: str, frame_id: str):
        super().__init__(topic, frame_id)

        self.last_pose = None
        self.linear_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.last_time = rospy.Time.now()

    def odom_callback(self, msg: Odometry):
        if not isinstance(msg, Odometry):
            rospy.logerr("Invalid message type")

        self.data = msg

        self.get_target_frame_pose()

        self.linear_velocity = self.get_linear_velocity(
            current_pose=self.transformed_pose
        )

    def get_linear_velocity(self, current_pose: PoseStamped):
        last_pose = self.last_pose

        if current_pose is None or last_pose is None:
            rospy.logwarn("Invalid pose")
            self.last_pose = current_pose
            return Vector3(x=0.0, y=0.0, z=0.0)

        # Calculate linear velocity
        dx = current_pose.pose.position.x - last_pose.pose.position.x
        dy = current_pose.pose.position.y - last_pose.pose.position.y
        dz = current_pose.pose.position.z - last_pose.pose.position.z

        # Calculate time difference
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if dt == 0:
            rospy.logwarn("Time difference is zero")
            return Vector3(x=0.0, y=0.0, z=0.0)

        self.last_time = current_time
        self.last_pose = current_pose

        # Calculate linear velocity
        return Vector3(x=dx / dt, y=dy / dt, z=dz / dt)


class CollisionCheck:
    def __init__(self):
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("ENDEFFECTOR")

        self.scene.clear()

        # End Effector State
        # self.end_effector_state = EEFState("/odom/fake", "camera")

        # Add Collision Objects
        self.create_box(
            Pose(
                position=Point(x=-0.5, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            (0.1, 1.0, 1.0),
        )

        # GetStateValidity 서비스 클라이언트 설정
        rospy.wait_for_service("/check_state_validity")
        self.state_validity_service = rospy.ServiceProxy(
            "/check_state_validity", GetStateValidity
        )

    def create_box(self, pose: Pose, size: tuple):
        # 사용자 정의 오브젝트 생성 & 추가
        object_pose = PoseStamped()
        object_pose.header.frame_id = "base_link"  # 기준 좌표계 설정
        object_pose.pose = pose

        self.scene.add_box("box", object_pose, size=size)

    def predict_joint_state(self, delta_time: float):
        current_joint_state = self.robot.get_current_state()

        # Get current joint positions and velocities
        current_joint_positions = np.array(current_joint_state.joint_state.position)
        current_joint_velocities = current_joint_state.joint_state.velocity

        # TODO: Change current_joint_velocities to real values

        # If joint positions and velocities have different lengths, set velocities to 0
        if len(current_joint_positions) != len(current_joint_velocities):
            current_joint_velocities = [0.0] * len(current_joint_positions)

        # Convert to numpy array
        current_joint_positions = np.array(current_joint_positions)
        current_joint_velocities = np.array(current_joint_velocities)

        # Predict joint positions
        predicted_joint_positions = current_joint_positions + (
            current_joint_velocities * delta_time
        )

        # Update joint state
        predicted_joint_state = current_joint_state
        predicted_joint_state.joint_state.position = list(predicted_joint_positions)

        return predicted_joint_state

    def request_state_validity(self):
        is_valid = False

        try:
            request = GetStateValidityRequest()
            request.group_name = "ENDEFFECTOR"
            # request.robot_state = self.robot.get_current_state()
            request.robot_state = self.predict_joint_state(1.0)

            response = self.state_validity_service(request)

            is_valid = response.valid
        except rospy.ServiceException as service_ex:
            rospy.logerr("Service call failed.")
            rospy.logerr(service_ex)
        except Exception as ex:
            rospy.logerr("Exception occurred.")
            rospy.logerr(ex)
        finally:
            return is_valid


def main():
    rospy.init_node("collision_check_node")  # TODO: Add node name

    collision_check = CollisionCheck()

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():

        is_valid = collision_check.request_state_validity()
        rospy.loginfo(f"State Validity: {is_valid}")

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

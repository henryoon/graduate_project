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
import cv2
import cv_bridge
import apriltag
import json


class AprilTagDetector:
    class AprilTag:
        def __init__(self, id: int, pose: Pose):
            self.id = id
            self.pose = pose

        def to_json(self):
            return {
                "id": self.id,
                "pose": {
                    "position": {
                        "x": self.pose.position.x,
                        "y": self.pose.position.y,
                        "z": self.pose.position.z,
                    },
                    "orientation": {
                        "x": self.pose.orientation.x,
                        "y": self.pose.orientation.y,
                        "z": self.pose.orientation.z,
                        "w": self.pose.orientation.w,
                    },
                },
            }

    class AprilTagArray:
        def __init__(self, header: Header, tags: list):
            self.header = header
            self.tags = tags

        def append(self, tag):
            self.tags.append(tag)

        def to_json(self):
            return {"tags": [tag.to_json() for tag in self.tags]}

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.tag_detector = apriltag.Detector(
            options=apriltag.DetectorOptions(families="tag36h11")
        )

        self.tf_listener = tf.TransformListener()

        tag_size = 0.04
        self.object_points = np.array(
            [
                [-tag_size / 2.0, -tag_size / 2.0, 0.0],
                [tag_size / 2.0, -tag_size / 2.0, 0.0],
                [tag_size / 2.0, tag_size / 2.0, 0.0],
                [-tag_size / 2.0, tag_size / 2.0, 0.0],
            ]
        )

        self.image_array = None
        self.color_image_subscriber = rospy.Subscriber(
            "/EEF_camera/color/image_raw", Image, self.color_image_callback
        )

        self.camera_info = None
        self.camera_info_subscriber = rospy.Subscriber(
            "/EEF_camera/color/camera_info", CameraInfo, self.camera_info_callback
        )

        self.tags = self.AprilTagArray(header=Header(frame_id="base_link"), tags=[])

    def color_image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            self.image_array = cv_image_gray

            self.tags = self.detect_april_tags(cv_image_gray)

        except cv_bridge.CvBridgeError as cv_bridge_ex:
            rospy.logerr(cv_bridge_ex)
        except Exception as ex:
            rospy.logerr(ex)

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg

    def detect_april_tags(self, np_image: np.array):
        tags = self.tag_detector.detect(np_image)

        if self.camera_info is None:
            rospy.logwarn("Camera info is not available.")
            return self.AprilTagArray(header=Header(frame_id="base_link"), tags=[])

        camera_matrix = np.array(self.camera_info.K).reshape(3, 3)
        dist_coeffs = np.array(self.camera_info.D)

        tag_array = self.AprilTagArray(
            header=Header(frame_id="camera_link", stamp=rospy.Time.now()), tags=[]
        )

        for tag in tags:
            image_points = np.array(tag.corners)

            success, rotation_vector, translation_vector = cv2.solvePnP(
                self.object_points, image_points, camera_matrix, dist_coeffs
            )

            if success:
                translation_vector = translation_vector.flatten()
                rotation_vector = rotation_vector.flatten()

                pose_stamped = PoseStamped()

                pose_stamped.header = Header(
                    frame_id="EEF_camera_link", stamp=rospy.Time(0)
                )

                pose = Pose()

                pose.position.x = translation_vector[0]
                pose.position.y = translation_vector[1]
                pose.position.z = translation_vector[2]

                quaternion = quaternion_from_euler(
                    rotation_vector[0], rotation_vector[1], rotation_vector[2]
                )

                pose.orientation.x = quaternion[0]
                pose.orientation.y = quaternion[1]
                pose.orientation.z = quaternion[2]
                pose.orientation.w = quaternion[3]

                pose_stamped.pose = pose

                if self.tf_listener.canTransform(
                    "base_link", "EEF_camera_link", rospy.Time(0)
                ):
                    transformed_pose = self.tf_listener.transformPose(
                        "base_link", pose_stamped
                    )
                    tag_array.append(
                        self.AprilTag(id=tag.tag_id, pose=transformed_pose.pose)
                    )
                else:
                    rospy.logwarn(
                        "Cannot lookup transform between base_link and EEF_camera_link."
                    )

            else:
                rospy.logwarn("Failed to solve PnP.")

        return tag_array


class AprilTagData:
    def __init__(self, file_path: str):
        self.file_path = file_path
        self.data = self.load_data()

    def load_data(self):
        f = open(self.file_path, "r")
        return json.load(f)

    def get_pose(self, tag_id: int):
        tags_data = self.data["tags"]

        for tag in tags_data:
            if int(tag["id"]) == tag_id:
                position = tag["pose"]["position"]
                orientation = tag["pose"]["orientation"]

                pose = PoseStamped()
                pose.header = Header(frame_id="map", stamp=rospy.Time.now())

                pose.pose.position.x = position["x"]
                pose.pose.position.y = position["y"]
                pose.pose.position.z = position["z"]

                pose.pose.orientation.x = orientation["x"]
                pose.pose.orientation.y = orientation["y"]
                pose.pose.orientation.z = orientation["z"]
                pose.pose.orientation.w = orientation["w"]

                return pose

        return None


class AprilTagLocalization:
    def __init__(self):
        self.april_tag_detector = AprilTagDetector()
        self.april_tag_data = AprilTagData(
            file_path="/home/irol/project_hj/src/localization/resources/001.json"
        )

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

    def run(self):
        local_tags = self.april_tag_detector.tags

        print(local_tags.to_json())

        for tag in local_tags.tags:
            local_pose = PoseStamped()
            local_pose.header = Header(frame_id="base_link", stamp=rospy.Time.now())
            local_pose.pose = tag.pose

            # Before
            translation, rotation = self.calculate_tf(
                global_pose=self.april_tag_data.get_pose(tag_id=tag.id),
                local_pose=local_pose,
            )

            # Before
            # translation, rotation = self.calculate_tf(
            #     global_pose=local_pose,
            #     local_pose=self.april_tag_data.get_pose(tag_id=tag.id),
            # )

            # print(translation, rotation)

            self.tf_broadcaster.sendTransform(
                time=rospy.Time.now(),
                parent="map",
                child="base_link",
                translation=translation,
                rotation=rotation,
            )

            return None

    def calculate_tf(self, local_pose: PoseStamped, global_pose: PoseStamped):
        local_frame = local_pose.header.frame_id
        global_frame = global_pose.header.frame_id

        (local_roll, local_pitch, local_yaw) = euler_from_quaternion(
            [
                local_pose.pose.orientation.x,
                local_pose.pose.orientation.y,
                local_pose.pose.orientation.z,
                local_pose.pose.orientation.w,
            ]
        )

        (global_roll, global_pitch, global_yaw) = euler_from_quaternion(
            [
                global_pose.pose.orientation.x,
                global_pose.pose.orientation.y,
                global_pose.pose.orientation.z,
                global_pose.pose.orientation.w,
            ]
        )

        droll = global_roll - local_roll
        dpitch = global_pitch - local_pitch
        dyaw = global_yaw - local_yaw

        print(droll, dpitch, dyaw)

        # Calculate translation in 3D space
        trans_x = global_pose.pose.position.x - (
            (local_pose.pose.position.x * np.cos(dyaw) * np.cos(dpitch))
            - (local_pose.pose.position.y * np.sin(dyaw) * np.cos(dpitch))
            - (local_pose.pose.position.z * np.sin(dpitch))
        )

        trans_y = global_pose.pose.position.y - (
            (local_pose.pose.position.x * np.sin(dyaw) * np.cos(droll))
            + (local_pose.pose.position.y * np.cos(dyaw) * np.cos(droll))
            - (local_pose.pose.position.z * np.sin(droll))
        )

        trans_z = global_pose.pose.position.z - local_pose.pose.position.z

        translation = [trans_x, trans_y, trans_z]

        rotation = quaternion_from_euler(0.0, 0.0, dyaw)

        return translation, rotation


def main():
    rospy.init_node("april_tag_localization_node")  # TODO: Add node name

    april_tag_localization = AprilTagLocalization()

    r = rospy.Rate(20)  # TODO: Add rate
    while not rospy.is_shutdown():
        april_tag_localization.run()
        r.sleep()


if __name__ == "__main__":
    main()
    try:
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

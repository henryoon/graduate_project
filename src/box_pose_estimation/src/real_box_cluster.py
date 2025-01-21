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


class AverageFilter:
    def __init__(self):
        self.i = 0
        self.average = 0.0

    def filter(self, data):
        # 샘플 수 +1 (+1 the number of sample)
        self.i += 1

        # 평균 필터의 alpha 값 (alpha of average filter)
        alpha = (self.i - 1) / (self.i + 0.0)

        # 평균 필터의 재귀식 (recursive expression of average filter)
        average = alpha * self.average + (1 - alpha) * data

        # 평균 필터의 이전 상태값 업데이트 (update previous state value of average filter)
        self.average = average

        return average


class DetectedBox:
    def __init__(self, position: Point, id=None):
        self.point = position
        self.id = id

    def set_id(self, id):
        self.id = id

    def abs_distance(self):
        return np.abs(np.linalg.norm([self.point.x, self.point.y, self.point.z]))

    def __eq__(self, other):
        p1 = np.array([self.point.x, self.point.y, self.point.z])
        p2 = np.array([other.point.x, other.point.y, other.point.z])

        dist = np.abs(np.linalg.norm(p1 - p2))

        return dist < 0.1

    def __add__(self, other):
        p1 = np.array([self.point.x, self.point.y, self.point.z])
        p2 = np.array([other.point.x, other.point.y, other.point.z])

        mean = np.mean([p1, p2], axis=0)

        return DetectedBox(Point(mean[0], mean[1], mean[2]), self.id)

    def __str__(self):
        return f"Box at ({self.point.x}, {self.point.y}, {self.point.z})"


class RealBoxCluster:
    def __init__(self):
        self.sub = rospy.Subscriber("/box_pose_array", PoseArray, self.boxes_callback)
        self.box_object_multi_array_pub = rospy.Publisher(
            "/real_box/eb", BoxObjectMultiArray, queue_size=1
        )

        self.data = []

    def boxes_callback(self, msg: PoseArray):
        existing_boxes = self.data
        new_boxes = msg.poses

        for i, new_box in enumerate(new_boxes):
            new_box: Pose
            new_box = DetectedBox(new_box.position, i)

            if new_box.abs_distance() > 5.0:
                continue

            check_duplicate = [box == new_box for box in existing_boxes]
            if not any(check_duplicate):
                # new_box.set_id(i)
                if 0.8 < new_box.point.x and new_box.point.x < 1.0:
                    existing_boxes.append(new_box)

            else:
                if check_duplicate.count(True) > 1:
                    rospy.logwarn("Multiple boxes detected.")

                else:
                    duplicated_box = existing_boxes[check_duplicate.index(True)]
                    updated_box = new_box + duplicated_box

                    existing_boxes.remove(duplicated_box)
                    existing_boxes.append(updated_box)

        self.data = existing_boxes

        self.parse_data()

    def parse_data(self):
        box_object_multi_array = BoxObjectMultiArray()

        for i, box in enumerate(self.data):
            box: DetectedBox

            box_object = BoxObject()
            box_object.header.stamp = rospy.Time.now()
            box_object.id = box.id
            box_object.pose.position = box.point
            box_object.scale = Vector3(0.1, 0.1, 0.1)

            box_object_multi_array.boxes.append(box_object)

        self.box_object_multi_array_pub.publish(box_object_multi_array)


def main():
    rospy.init_node("fuck")  # TODO: Add node name

    real_box_cluster = RealBoxCluster()

    rospy.spin()

    # r = rospy.Rate()  # TODO: Add rate
    # while not rospy.is_shutdown():
    #     r.sleep()


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

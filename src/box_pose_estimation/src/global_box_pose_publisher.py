#!/usr/bin/python3

import os
import sys
import rospy
from abc import ABC, abstractmethod

# Messages
from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from visualization_msgs.msg import *
from custom_msgs.msg import *
import json


class GlobalBoxPosePublisher:
    def __init__(self):
        # Load virtual boxes data from json file
        self.boxes = self.load_boxes_from_json(
            "/workspace/src/box_pose_estimation/resource/1205_virtual_boxes_data.json"
        )

        self.marker_array = self.parse_box_to_marker_array()

        self.data_pub = rospy.Publisher(
            "/virtual_box", BoxObjectMultiArray, queue_size=1
        )
        self.pub = rospy.Publisher("/boxes/virtual", MarkerArray, queue_size=1)

    def load_boxes_from_json(self, json_file_path):
        boxes = BoxObjectMultiArray()

        with open(json_file_path, "r") as file:
            data = json.load(file)
            for box_data in data["boxes"]:
                box = BoxObject()
                box.pose.position.x = box_data["pose"]["position"]["x"]
                box.pose.position.y = box_data["pose"]["position"]["y"]
                box.pose.position.z = box_data["pose"]["position"]["z"]
                box.pose.orientation.x = box_data["pose"]["orientation"]["x"]
                box.pose.orientation.y = box_data["pose"]["orientation"]["y"]
                box.pose.orientation.z = box_data["pose"]["orientation"]["z"]
                box.pose.orientation.w = box_data["pose"]["orientation"]["w"]
                box.scale.x = box_data["scale"]["x"]
                box.scale.y = box_data["scale"]["y"]
                box.scale.z = box_data["scale"]["z"]
                box.id = box_data["id"]
                boxes.boxes.append(box)

        return boxes

    def parse_box_to_marker_array(self):
        marker_array = MarkerArray()
        for i, box in enumerate(self.boxes.boxes):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "box"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = box.pose
            marker.scale = box.scale
            248, 196, 113
            marker.color.r = 248.0 / 255.0
            marker.color.g = 196.0 / 255.0
            marker.color.b = 113.0 / 255.0
            marker.color.a = 0.5

            marker_array.markers.append(marker)

        return marker_array

    def publish_boxes_data(self):
        self.data_pub.publish(self.boxes)

    def publish_boxes(self):
        self.pub.publish(self.marker_array)


def main():
    rospy.init_node("global_box_pose_publisher_node", anonymous=True)
    global_box_pose_publisher = GlobalBoxPosePublisher()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        global_box_pose_publisher.publish_boxes()
        global_box_pose_publisher.publish_boxes_data()
        rate.sleep()


if __name__ == "__main__":
    main()

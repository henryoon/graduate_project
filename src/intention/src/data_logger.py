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
from custom_msgs.msg import *


class EEF_State(object):
    def __init__(self, *args, **kwargs):
        self.tf_listener = tf.TransformListener()

        self.base_frame = kwargs.get("base_frame", "base_link")
        self.tool_frame = kwargs.get("tool_frame", "tool0")
        self.camera_frame = kwargs.get("camera_frame", "camera_link")
        self.box_topic = kwargs.get("box_topic", "/tf_C2O")
        self.twist_topic = kwargs.get("twist_topic", "/twist")

        self.box = None
        self.twist = Twist()

        self.box_subscirber = rospy.Subscriber(
            self.box_topic, PoseArray, self.box_callback
        )
        self.twist_subscriber = rospy.Subscriber(
            self.twist_topic, Twist, self.twist_callback
        )

    def twist_callback(self, msg: Twist):
        self.twist = msg

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
        self.twist_topic = kwargs.get("twist_topic", "/twist")

        self.hz = kwargs.get("hz", 10)

        self.boxes_sub = rospy.Subscriber("/april_box", BoxObjectMultiArray, self.boxes_callback)
        self.boxes = []

        self.joystick_sub = rospy.Subscriber(
            "/controller/right/joy", Joy, self.joystick_callback
        )
        self.is_pressed_A = 0

        self.last_pose = None
        self.last_heading = None
        self.last_time = None

        self.eef = EEF_State(
            base_frame=self.base_frame,
            tool_frame=self.tool_frame,
            camera_frame=self.camera_frame,
            box_topic=self.box_topic,
            twist_topic=self.twist_topic,
        )

    def boxes_callback(self, msg: BoxObjectMultiArray):
        self.boxes = msg.boxes
        # return
    
        # if len(self.boxes) == 0:
        #     self.boxes = msg.boxes

        # new_data = self.boxes

        # for new_box in msg.boxes:
        #     # new_box => new box
        #     for box in new_data:
        #         # box => current box
        #         if box.id == new_box.id:
        #             new_data.remove(box)
        #             new_data.append(new_box)
        #             break

        # self.data = new_data

    def log_boxes(self):
        text = ""

        for box in self.boxes:
            text += f"-1,{box.id}, {box.pose.position.x}, {box.pose.position.y}, {box.pose.position.z},".replace(" ", "")

        text = text[:-1]

        return text


    def joystick_callback(self, msg: Joy):
        A = msg.buttons[0]
        self.is_pressed_A = int(A)

        if self.is_pressed_A == 1:
            self.joystick_sub.unregister()


    def run(self):
        r = rospy.Rate(self.hz)

        f = open(self.log_path + "/" + self.log_file + ".csv", "w")
        f2 = open(self.log_path + "/" + self.log_file + "_box.csv", "w")

        f.write(
            "Time, Pose.x, Pose.y, Pose.z, Twist.linear.x, Twist.linear.y, Twist.linear.z, Twist.angular.x, Twist.angular.y, Twist.angular.z\n".replace(
                " ", ""
            )
        )


        while not rospy.is_shutdown():
            eef_pose = self.eef.get_tf_origin()
            eef_twist = self.eef.twist
            box = self.eef.box

            if eef_pose is None or eef_twist is None:
                warn_txt = "Cannot calculate: "
                warn_txt += "EEF Pose\t" if eef_pose is None else ""
                warn_txt += "EEF Twist\t" if eef_twist is None else ""

                rospy.logwarn(warn_txt)

                r.sleep()

                continue

            if self.is_pressed_A == 1:
                rospy.loginfo("Logging Stopped.")
                f.close()
                f2.close()
                break

            current_time = rospy.Time.now().to_sec()

            txt = ""
            # Time
            txt += f"{current_time},"
            # Pose
            txt += f"{eef_pose.position.x}, {eef_pose.position.y}, {eef_pose.position.z},"
            # Twist
            txt += f"{eef_twist.linear.x}, {eef_twist.linear.y}, {eef_twist.linear.z}, {eef_twist.angular.x}, {eef_twist.angular.y}, {eef_twist.angular.z}"
            txt += "\n"

            f.write(txt.replace(" ", ""))

            box_text = ""
            # Time
            box_text += f"{current_time},"
            # Data
            box_text += self.log_boxes()
            box_text += "\n"

            f2.write(box_text.replace(" ", ""))

            rospy.loginfo("Logging Success! : {}, {}".format(rospy.Time.now().to_sec(), len(self.boxes)))

            r.sleep()


def main():
    rospy.init_node("data_logging_node")  # TODO: Add node name

    # rospy.spin()

    logger = DataLogger(
        log_file="241121_09",
        log_path="/home/irol/project_hj/src/intention/resource",
        base_frame="base_link",
        tool_frame="VGC",
        camera_frame="EEF_camera_link",
        box_topic="/tf_C2O",
        twist_topic="/calculated_twist",
        hz=10,
    )


    file_list = os.listdir("/home/irol/project_hj/src/intention/resource")

    for file in file_list:
        if file == logger.log_file + ".csv":
            rospy.loginfo("File already exists.")
            return None

    logger.run()


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

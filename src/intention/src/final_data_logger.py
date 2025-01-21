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


try:
    sys.path.append(roslib.packages.get_pkg_dir("test_package") + "/src")
    from abstract_state import State
except roslib.exceptions.ROSLibException:
    rospy.logfatal("Cannot find package test_package")
    sys.exit(1)

# === Real Level Data ===
# === 0 Dimensional Data ===
# Time
# Distance
# Wave Sensor Data
# Each Boxes Data => Log to another file

# === 1 Dimensional Data ===
# Mean
# Covariance

# === 2 Dimensional Data ===
# Each Boxes PDF Data

# @@@@@@@@@@@@@@@@@@@@@@@@@@@

# === Virtual Level Data ===
# === 0 Dimensional Data ===
# Time
# Distance => Calculated Box - EEF Distance
# Wave Sensor Data => None
# Each Boxes Data => Json file

# === 1 Dimensional Data ===
# Mean
# Covariance

# === 2 Dimensional Data ===
# Each Boxes PDF Data


class BoxLogger:
    def __init__(self, f):
        self.f = f

        self.sub = rospy.Subscriber("/real_box/eb", BoxObjectMultiArray, self.callback)

    def callback(self, msg: BoxObjectMultiArray):
        boxes = msg.boxes
        boxes: list

        txt = ""

        for box in boxes:
            box_id = box.id
            box_pose = box.pose
            box_pose: Pose

            box_position = box_pose.position
            box_position: Point

            txt += ","
            txt += f"{box_id},{box_position.x},{box_position.y},{box_position.z}"

        self.f.write(txt + "\n")


class FinalDataLogger:
    def __init__(self, f, *args, **kwargs):
        self.f = f

        self.f.write(
            "time,distance,wave_sensor.distance,wave_sensor.signal,intersection.mean.x,intersection.mean.y,intersection.cov.xx,intersection.cov.xy,intersection.cov.yx,intersection.cov.yy,pdf.id0,pdf.val0,pdf.id1,pdf.val1,pdf.id2,pdf.val2,pdf.id3,pdf.val3\n"
        )

        self.task_idx = 0

        distance_topic = kwargs.get("distance_topic", "/distance/real")
        wave_sensor_topic = kwargs.get("wave_sensor_topic", "/wave0")
        intersection_topic = kwargs.get("intersection_topic", "/intersection/real")
        pdf_topic = kwargs.get("pdf_topic", "/pdf/real")

        self.distance_sub = rospy.Subscriber(
            distance_topic, Float32, self.distance_callback
        )
        if wave_sensor_topic is not None:
            self.wave_sensor_sub = rospy.Subscriber(
                wave_sensor_topic, WaveMsg, self.wave_sensor_callback
            )
        self.intersection_sub = rospy.Subscriber(
            intersection_topic, PoseWithCovarianceStamped, self.intersection_callback
        )
        self.pdf_sub = rospy.Subscriber(
            pdf_topic, BoxObjectWithPDFMultiArray, self.pdf_callback
        )
        self.controller_sub = rospy.Subscriber(
            "/controller/right/joy", Joy, self.controller_callback
        )

        self.state = State(topic=None, frame_id="VGC")

        self.controller_input = [False, False, False, False]

        self.distance_log = None  # m
        self.wave_sensor_log = None  # mm, sigal
        self.intersection_log = None  # m, m, cov, cov, cov, cov
        self.pdf_log = None  # pdf, pdf, pdf, pdf

    # === Callbacks ===
    def distance_callback(self, msg: Float32):
        # distance_data = msg.data

        pose = self.state.get_target_frame_pose2(target_frame="box")
        pose: PoseStamped

        distance = -pose.pose.position.x

        self.distance_log = f"{distance}"

    def wave_sensor_callback(self, msg: WaveMsg):
        distance = msg.distance
        signal = msg.signal
        self.wave_sensor_log = f"{distance},{signal}"

    def intersection_callback(self, msg: PoseWithCovarianceStamped):
        intersection_data = msg.pose
        point_data = intersection_data.pose.position
        cov_data = (
            np.reshape(np.array(intersection_data.covariance), newshape=(6, 6))[
                1:3, 1:3
            ]
            .flatten()
            .tolist()
        )
        cov_data: list

        self.intersection_log = f"{point_data.y},{point_data.z},{cov_data[0]},{cov_data[1]},{cov_data[2]},{cov_data[3]}"

    def pdf_callback(self, msg: BoxObjectWithPDFMultiArray):
        pdf_data = msg.boxes
        pdf_data: list

        pdf_text = ""
        for pdf in pdf_data:
            pdf_text += f"{pdf.id},{pdf.pdf},"

        self.pdf_log = pdf_text[:-1] + "\n"

    def controller_callback(self, msg: Joy):
        # [A, B, 중지, 검지]
        if len(msg.buttons) != 4:
            rospy.logwarn("Invalid Joy Message")
            return

        A = msg.buttons[0]
        B = msg.buttons[1]
        RESET = msg.buttons[2]
        CONTROL = msg.buttons[3]

        self.controller_input = [bool(A), bool(B), bool(RESET), bool(CONTROL)]

    def log(self, *args, **kwargs):
        # === Level 0 Data ===

        if self.distance_log is None:
            return

        if self.wave_sensor_log is None:
            return

        if self.intersection_log is None:
            return

        if self.pdf_log is None:
            return

        time_data = rospy.Time.now().to_sec()
        distance_data = self.distance_log
        wave_sensor_data = self.wave_sensor_log

        # === Level 1 Data ===
        intersection_data = self.intersection_log

        # === Level 2 Data ===
        pdf_data = self.pdf_log

        # === Write Data ===
        data = f"{time_data},{distance_data},{wave_sensor_data},{intersection_data},{pdf_data}"

        print(data)

        self.f.write(data)


def main():
    rospy.init_node("final_data_logger")  # TODO: Add node name

    # rospy.spin()

    idx = "21"

    real_data_stream = open(
        f"/home/irol/project_hj/src/intention/resource/final_data/real_data{idx}.csv",
        "w",
    )
    real_data_logger = FinalDataLogger(
        f=real_data_stream,
        distance_topic="/distance/real",
        wave_sensor_topic="/wave0",
        intersection_topic="/intersection/real",
        pdf_topic="/pdf/real",
    )

    virtual_data_stream = open(
        f"/home/irol/project_hj/src/intention/resource/final_data/virtual_data{idx}.csv",
        "w",
    )
    virtual_data_logger = FinalDataLogger(
        f=virtual_data_stream,
        distance_topic="/distance/virtual",
        wave_sensor_topic=None,
        intersection_topic="/intersection/virtual",
        pdf_topic="/pdf/virtual",
    )

    box_data_stream = open(
        f"/home/irol/project_hj/src/intention/resource/final_data/real_box_data{idx}.csv",
        "w",
    )
    box_data_logger = BoxLogger(f=box_data_stream)

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():
        if real_data_logger.controller_input[0]:
            real_data_stream.close()
            virtual_data_stream.close()
            box_data_stream.close()
            break

        real_data_logger.log()
        virtual_data_logger.log()
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

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
from custom_msgs.msg import MotorControl

# Custom
from abc import ABC, abstractmethod
import enum
import threading
import serial


class Packet(ABC):
    def __init__(self, ID: int):

        self.HEADER1 = 255
        self.HEADER2 = 254

        self.ID = ID

        self.request_packet = self.request_packet()
        self.feedback_packet = self.feedback_packet()

    @abstractmethod
    def create_request_packet(self):
        pass

    @abstractmethod
    def create_feedback_packet(self):
        pass


# 위치, 속도제어 (송신)
class PositionRPMControlPacket(Packet):
    class Direction(enum.Enum):
        CCW = 0
        CW = 1

    def __init__(self, id: int):
        super().__init__(id)

    def create_request_packet(self, direction: int, position: float, rpm: float):
        """Direction: 0: CCW, 1: CW\nPosition: DEG\nRPM: RPM"""
        position_data = int(position / 0.01)
        position_packet = position_data.to_bytes(2, byteorder="big")

        rpm_data = int(rpm / 0.1)
        rpm_packet = rpm_data.to_bytes(2, byteorder="big")

        packet = bytearray(11)

        packet[0] = self.HEADER1
        packet[1] = self.HEADER2
        packet[2] = self.ID
        packet[3] = len(packet) - 4  # DATASIZE. CHECKSUM ~ END
        packet[4] = 0  # Temp Checksum
        packet[5] = 1  # MODE : POSITION RPM CONTROL
        packet[6] = direction
        packet[7] = position_packet[0]
        packet[8] = position_packet[1]
        packet[9] = rpm_packet[0]
        packet[10] = rpm_packet[1]

        packet[4] = ~(sum(packet) - self.HEADER1 - self.HEADER2) & 0xFF  # Checksum

        return packet

    def create_feedback_packet(self):
        return None


class MotorControlNode(object):
    def __init__(self, port: str, baudrate: int, motor_id: int):
        self.ser = serial.Serial(port, baudrate, timeout=1)

        self.motor_id = motor_id
        self.position_rpm_control_packet = PositionRPMControlPacket(self.motor_id)

        self.control_packet = None

        self.cmd_sub = rospy.Subscriber(
            "motor1/cmd_vel", MotorControl, self.cmd_vel_callback
        )

    def cmd_vel_callback(self, msg: MotorControl):
        direction = (
            PositionRPMControlPacket.Direction.CW.value
            if msg.rpm.z > 0
            else PositionRPMControlPacket.Direction.CCW.value
        )
        position = msg.position.z
        rpm = msg.rpm.z

        self.control_packet = self.position_rpm_control_packet.create_request_packet(
            direction=direction, position=position, rpm=rpm
        )

    def send_serial(self):
        if self.control_packet is None:
            rospy.logwarn("Control packet is None.")
            return None

        self.ser.write(self.control_packet)


def main():
    rospy.init_node("motor_control_node")  # TODO: Add node name

    motor_control_node = MotorControlNode(
        port="/dev/ttyUSB0", baudrate=9600, motor_id=0
    )

    r = rospy.Rate(30)  # TODO: Add rate
    while not rospy.is_shutdown():
        motor_control_node.send_serial()
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

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
    class Direction(enum.Enum):
        CCW = 0
        CW = 1

        @classmethod
        def from_value(cls, value):
            for member in cls:
                if member.value == value:
                    return member.name
            rospy.logwarn(f"No matching Direction for value: {value}")
            return "UNKNOWN"

    def __init__(self, ID: int):
        self.HEADER1 = 255
        self.HEADER2 = 254
        self.ID = ID

    @abstractmethod
    def create_request_packet(self):
        pass

    def create_feedback_packet(self):
        packet = bytearray(6)

        packet[0] = self.HEADER1
        packet[1] = self.HEADER2
        packet[2] = self.ID
        packet[3] = 2  # DataSize
        packet[4] = 0  # Temp Checksum
        packet[5] = 161  # 위치 피드백

        return packet

    def interpret_feedback_packet(self, packet):
        if packet[0] != self.HEADER1 or packet[1] != self.HEADER2:
            rospy.logwarn("Header is not correct.")
            return None
        if int(packet[5]) != 209:
            rospy.logwarn("Mode is not correct. Response: {}".format(packet[5]))
            return None

        id = int(packet[2])
        direction = int(packet[6])
        position = int.from_bytes(packet[7:9], byteorder="big") * 0.01  # DEG
        rpm = int.from_bytes(packet[9:11], byteorder="big") * 0.1  # RPM
        current = int(packet[11]) * 0.1  # A

        return {
            "id": id,
            "direction": direction,
            "position": position,
            "rpm": rpm,
            "current": current,
        }


# 위치, 속도제어 (송신)
class PositionRPMControlPacket(Packet):
    def __init__(self, id: int):
        super().__init__(id)

        self.request_packet = self.create_request_packet(
            direction=self.Direction.CW.value, position=0, rpm=0
        )

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

        self.request_packet = packet

        return packet


class MotorControlNode(object):
    def __init__(self, port: str, baudrate: int, motor_id: int, cmd_vel_topic: str):
        self.motor_id = motor_id
        self.ser = serial.Serial(port, baudrate, timeout=1)

        self.packet = PositionRPMControlPacket(id=motor_id)

        self.motor_cmd_sub = rospy.Subscriber(
            cmd_vel_topic, MotorControl, self.cmd_vel_callback
        )

    def cmd_vel_callback(self, msg: MotorControl):
        direction = (
            PositionRPMControlPacket.Direction.CW.value
            if msg.rpm.z > 0
            else PositionRPMControlPacket.Direction.CCW.value
        )
        position = msg.position.z
        rpm = msg.rpm.z

        control_packet = self.packet.create_request_packet(
            direction=direction, position=position, rpm=rpm
        )

    def send_packet(self):
        if not self.ser.is_open:
            rospy.logwarn("Serial is not open.")
            return

        self.ser.write(self.packet.request_packet)


def main():
    rospy.init_node("motor_control_node")  # TODO: Add node name

    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baudrate", 9600)
    motor_id = rospy.get_param("~motor_id", 0)

    motor_control_node = MotorControlNode(
        port=port,
        baudrate=baudrate,
        motor_id=motor_id,
        cmd_vel_topic="/motor1/cmd_vel",
    )

    r = rospy.Rate(10)  # TODO: Add rate
    while not rospy.is_shutdown():
        motor_control_node.send_packet()
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

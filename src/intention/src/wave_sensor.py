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

# Serial
import serial
import time


class Wave:
    def __init__(self):
        # Data Publisher
        self.pub = rospy.Publisher("/wave0", WaveMsg, queue_size=10)

        # Data Structure
        self.TOF_length = 16
        self.TOF_header = (87, 0, 255)
        self.TOF_system_time = 0
        self.TOF_distance = 0
        self.TOF_status = 0
        self.TOF_signal = 0
        self.TOF_check = 0

        self.sef = serial.Serial("/dev/ttyUSB0", 921600)
        self.ser.flushInput()

        self.run()

    def verifyCheckSum(self, data, len):
        TOF_check = 0
        for k in range(0, len - 1):
            TOF_check += data[k]
        TOF_check = TOF_check % 256

        if TOF_check == data[len - 1]:
            print("TOF data is ok!")
            return 1
        else:
            print("TOF data is error!")
            return 0

    def run(self):
        while True:
            TOF_data = ()
            time.sleep(0.05)
            if self.ser.inWaiting() >= 32:
                for i in range(0, 16):
                    TOF_data = TOF_data + (ord(self.ser.read(1)), ord(self.ser.read(1)))

                # Initialize variables
                data = WaveMsg()
                data.header = Header(frame_id="", stamp=rospy.Time.now())

                for j in range(0, 16):
                    if (
                        TOF_data[j] == self.TOF_header[0]
                        and TOF_data[j + 1] == self.TOF_header[1]
                        and TOF_data[j + 2] == self.TOF_header[2]
                    ) and (
                        self.verifyCheckSum(
                            TOF_data[j : self.TOF_length], self.TOF_length
                        )
                    ):
                        if ((TOF_data[j + 12]) | (TOF_data[j + 13] << 8)) == 0:
                            print("Out of range!")
                        else:
                            # print("TOF id is: " + str(TOF_data[j + 3]))
                            data.id = TOF_data[j + 3]

                            TOF_system_time = (
                                TOF_data[j + 4]
                                | TOF_data[j + 5] << 8
                                | TOF_data[j + 6] << 16
                                | TOF_data[j + 7] << 24
                            )
                            # print("TOF system time is: " + str(TOF_system_time) + "ms")

                            TOF_distance = (
                                (TOF_data[j + 8])
                                | (TOF_data[j + 9] << 8)
                                | (TOF_data[j + 10] << 16)
                            )
                            # print("TOF distance is: " + str(TOF_distance) + "mm")
                            data.distance = TOF_distance

                            TOF_status = TOF_data[j + 11]
                            # print("TOF status is: " + str(TOF_status))
                            data.status = TOF_status
                            TOF_signal = TOF_data[j + 12] | TOF_data[j + 13] << 8
                            # print("TOF signal is: " + str(TOF_signal))
                            data.signal = TOF_signal

                            self.pub.publish(data)

                        break


def main():
    rospy.init_node("wave_sensor_node")  # TODO: Add node name

    wave_sensor = Wave()

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

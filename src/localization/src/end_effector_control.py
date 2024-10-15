#!/usr/bin/python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose


def main():
    # ROS 노드 초기화
    rospy.init_node("move_robot_to_pose", anonymous=True)

    # MoveIt! 초기화
    moveit_commander.roscpp_initialize(sys.argv)

    # Move Group Commander 초기화 (플래닝 그룹 설정)
    move_group = moveit_commander.MoveGroupCommander("ARM")

    move_group.set_max_velocity_scaling_factor(0.3)  # 100% 속도
    move_group.set_max_acceleration_scaling_factor(0.3)  # 100% 가속도

    # 타겟 초기화
    move_group.clear_pose_targets()

    pose = [0.23, 0.81]

    waypoints = []
    for i in range(4):
        pose[0], pose[1] = pose[1], pose[0]
        p = Pose()
        p.position.x = pose[0]
        p.position.y = pose[1]
        p.orientation.x = -0.5
        p.orientation.y = 0.5
        p.orientation.z = -0.5
        p.orientation.w = 0.5

        r = move_group.plan(p)
        print(type(r))
        move_group.go(wait=True)
        print(i)

    # 목표 Pose 설정 해제
    move_group.clear_pose_targets()

    print("로봇이 목표 위치로 이동했습니다.")

    rospy.spin()


if __name__ == "__main__":
    main()

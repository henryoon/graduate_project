using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using System;
using System.Collections.Generic;

public class BasePoseSubscriber : MonoBehaviour
{
    // ROSConnector를 통한 ROS 연결 인스턴스
    private ROSConnection ros;
    private PoseStampedMsg msg;

    // 송수신할 ROS 토픽 이름
    public string topicName;

    public ArticulationBody baseObject;

    public PoseStampedMsg GetMsg()
    {
        return msg;
    }

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PoseStampedMsg>(topicName, ReceiveMessage);

        // 수신할 메시지 인스턴스 생성
        msg = new PoseStampedMsg();
    }

    void Update()
    {
        // 수신한 메시지가 있을 경우
        if (msg != null)
        {
            // Vector3 position = RosToUnityTransform.ConvertPosition(new Vector3((float)msg.pose.position.x, (float)msg.pose.position.y, (float)msg.pose.position.z));
            // Quaternion rotation = RosToUnityTransform.ConvertRotation(new Quaternion((float)msg.pose.orientation.x, (float)msg.pose.orientation.y, (float)msg.pose.orientation.z, (float)msg.pose.orientation.w));

            Vector3 position = new Vector3((float)msg.pose.position.x, (float)msg.pose.position.z, (float)msg.pose.position.y);
            Quaternion rotation = new Quaternion((float)msg.pose.orientation.x, (float)msg.pose.orientation.y, (float)msg.pose.orientation.z, (float)msg.pose.orientation.w);

            rotation = rotation * Quaternion.Euler(0, 0, 0);

            baseObject.TeleportRoot(position, rotation);
        }
    }

    // ROS로부터 메시지 수신
    private void ReceiveMessage(PoseStampedMsg message)
    {
        // 수신한 메시지를 저장
        msg = message;
    }
}
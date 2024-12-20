using UnityEngine;
using UnityEngine.XR;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using System;
using System.Collections.Generic;

public class UpdateManipulator : MonoBehaviour
{
    // ROSConnector를 통한 ROS 연결 인스턴스
    private ROSConnection ros;
    private JointStateMsg msg;

    // 송수신할 ROS 토픽 이름
    public string topicName;
    
    [SerializeField]
    private List<ArticulationBody> joints;

    public JointStateMsg GetMsg()
    {
        return msg;
    }

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>(topicName, ReceiveMessage);

        // 수신할 메시지 인스턴스 생성
        msg = new JointStateMsg();
    }

    void Update()
    {
        for (int i=0; i < msg.name.Length; i++)
        {
            // 메시지로부터 조인트 이름, 위치, 속도, 힘, 각속도, 토크, 토크 한계값, 각속도 한계값, 토크 한계값을 읽어옴
            string name = msg.name[i];
            float position = (float)msg.position[i];

            ArticulationBody articulationBody = joints[i];

            ArticulationDrive xDrive = new ArticulationDrive();

            xDrive.driveType = ArticulationDriveType.Target;
            xDrive.target = position* Mathf.Rad2Deg * getGain(i);
            xDrive.forceLimit = 9999f; // msg.effort[i];

            // 설정된 드라이브를 ArticulationBody에 적용
            articulationBody.xDrive = xDrive;
        }
    }

    public float getGain(int index)
    {
        if (index == 0 || index == 3){
            return -1.0f;
        }

        return 1.0f;
    }

    // ROS로부터 메시지 수신
    private void ReceiveMessage(JointStateMsg message)
    {
        // 수신한 메시지를 저장
        msg = message;
    }
}
using UnityEngine;
using UnityEngine.XR;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using System;

public class BoxRenderer : MonoBehaviour
{
    // ROSConnector를 통한 ROS 연결 인스턴스
    private ROSConnection ros;
    private MarkerArrayMsg markerArrayMsg;

    // 송수신할 ROS 토픽 이름
    public string topicName;
    // 박스 머티리얼
    public Material boxMaterial;
    // 박스 부모 오브젝트
    public GameObject boxes;


    public MarkerArrayMsg GetMarkerArrayMsg()
    {
        return markerArrayMsg;
    }

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<MarkerArrayMsg>(topicName, ReceiveMessage);

        // 수신할 메시지 인스턴스 생성
        markerArrayMsg = new MarkerArrayMsg();
    }

    void Update()
    {
    }

    // ROS로부터 메시지 수신
    private void ReceiveMessage(MarkerArrayMsg message)
    {
        // 수신한 메시지를 저장
        markerArrayMsg = message;

        // 기존 박스 삭제
        foreach(Transform child in boxes.transform)
        {
            Destroy(child.gameObject);
        }

        for(int i = 0; i < markerArrayMsg.markers.Length; i++)
        {
            // 마커 메시지로부터 박스 생성, 위치, 회전, 크기 설정
            MarkerMsg marker = markerArrayMsg.markers[i];
            Vector3 position = RosToUnityTransform.ConvertPosition(new Vector3((float)marker.pose.position.x, (float)marker.pose.position.y, (float)marker.pose.position.z));
            Quaternion rotation = RosToUnityTransform.ConvertRotation(new Quaternion((float)marker.pose.orientation.x, (float)marker.pose.orientation.y, (float)marker.pose.orientation.z, (float)marker.pose.orientation.w));
            Vector3 scale = RosToUnityTransform.ConvertScale(new Vector3((float)marker.scale.x * 0.9f, (float)marker.scale.y * 0.9f, (float)marker.scale.z * 0.9f));

            CreateBox(position, rotation, scale, marker.id);
        }
    }

    GameObject CreateBox(Vector3 position, Quaternion rotation, Vector3 scale, int name)
    {
        // 박스 생성
        GameObject box = GameObject.CreatePrimitive(PrimitiveType.Cube);

        box.transform.parent = boxes.transform;
        box.transform.position = position;
        box.transform.rotation = rotation;
        box.transform.localScale = scale;
        box.name = name.ToString();

        // 박스 머티리얼 설정
        box.GetComponent<MeshRenderer>().material = boxMaterial;
        return box;
    }
}

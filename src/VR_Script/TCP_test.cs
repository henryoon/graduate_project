using UnityEngine;
using RosMessageTypes.Std;  // std_msgs/String 타입을 사용할 수 있도록 임포트
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;

public class StringMessageExample : MonoBehaviour
{
    // ROSConnector를 통한 ROS 연결 인스턴스
    private ROSConnection ros;
    private GameObject[] boxes;

    // 송수신할 ROS 토픽 이름
    public string topicName = "/boxes/virtual";

    // 송신 주기를 조절할 변수
    public float publishRate = 1.0f;
    private float timeElapsed;

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<MarkerArrayMsg>(topicName, ReceiveMessage);
    }

    void Update()
    {
        // 주기적으로 메시지 송신
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishRate)
        {
            timeElapsed = 0;
            SendMessageToROS("Hello from Unity!");
        }
    }

    // ROS로 String 메시지 송신
    private void SendMessageToROS(string messageContent)
    {
        StringMsg message = new StringMsg(messageContent);
        ros.Publish(topicName, message);
        Debug.Log("Sent message: " + messageContent);
    }

    // ROS로부터 메시지 수신
    private void ReceiveMessage(MarkerArrayMsg message)
    {
        if (message.markers.Length == 0)
        {
            Debug.Log("Received message: Empty");
            return;
        }

        if (boxes == null)
        {
            boxes = new GameObject[message.markers.Length];
            for (int i = 0; i < message.markers.Length; i++)
            {
                boxes[i] = GameObject.CreatePrimitive(PrimitiveType.Cube);

                boxes[i].transform.position = new Vector3(
                    (float)-message.markers[i].pose.position.y,
                    (float)message.markers[i].pose.position.z,
                    (float)message.markers[i].pose.position.x
                );

                boxes[i].transform.rotation = new Quaternion(
                    (float)message.markers[i].pose.orientation.x,
                    (float)message.markers[i].pose.orientation.y,
                    (float)message.markers[i].pose.orientation.z,
                    (float)message.markers[i].pose.orientation.w
                );

                boxes[i].transform.localScale = new Vector3(
                    (float)message.markers[i].scale.y,
                    (float)message.markers[i].scale.z,
                    (float)message.markers[i].scale.x
                );
            }
        }
    }
}

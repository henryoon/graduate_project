using UnityEngine;
using UnityEngine.XR;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector;
using System;

public class PointCloudSubscriber : MonoBehaviour
{
    // ROSConnector를 통한 ROS 연결 인스턴스
    private ROSConnection ros;
    private PointCloud2Msg PointCloud2Msg;

    // 송수신할 ROS 토픽 이름
    public string topicName;


    private bool rgb = false;
    private bool intensity = false;

    private PointFieldMsg[] fields;
    public int size;
    public int length;
    private byte[] byteArray;
    public int width;
    public int height;
    public int row_step;
    public int point_step;
    private bool isMessageReceived = false;

    private Vector3[] pcl;
    private Color[] pcl_color;
    public Color color;


    public PointCloud2Msg GetPointCloud2Msg()
    {
        return PointCloud2Msg;
    }

    public Vector3[] GetPCL()
    {
        return pcl;
    }

    public Color[] GetPCLColor()
    {
        return pcl_color;
    }

    void Start()
    {
        // ROS 연결 설정
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<PointCloud2Msg>(topicName, ReceiveMessage);

        // 수신할 메시지 인스턴스 생성
        PointCloud2Msg = new PointCloud2Msg();
    }

    void Update()
    {
        if (isMessageReceived)
        {
            renderPointCloud();
            isMessageReceived = false;
        }
    }

    // ROS로부터 메시지 수신
    private void ReceiveMessage(PointCloud2Msg message)
    {
        Debug.Log("Receive PointCloud2 Message");
        // 수신한 메시지를 저장
        PointCloud2Msg = message;

        length = PointCloud2Msg.data.Length;
        fields = PointCloud2Msg.fields;

        if (fields.Length > 3)
        {
            if (fields[3].name == "rgb")
            {
                rgb = true;
                intensity = false;
            }
            else if (fields[3].name == "intensity")
            {
                rgb = false;
                intensity = true;
            }
        }

        byteArray = new byte[length];
        byteArray = PointCloud2Msg.data;

        width = (int)PointCloud2Msg.width;
        height = (int)PointCloud2Msg.height;
        row_step = (int)PointCloud2Msg.row_step;
        point_step = (int)PointCloud2Msg.point_step;
        
        size = length / point_step;

        isMessageReceived = true;
    }

    void renderPointCloud()
    {
        pcl = new Vector3[size];
        pcl_color = new Color[size];

        int x_posi;
        int y_posi;
        int z_posi;

        int rgb_posi;
        int intensity_posi;

        float x;
        float y;
        float z;

        for (int i = 0; i < size; i++)
        {
            x_posi = i * point_step;
            y_posi = i * point_step + 4;
            z_posi = i * point_step + 8;

            x = BitConverter.ToSingle(byteArray, x_posi);
            y = BitConverter.ToSingle(byteArray, y_posi);
            z = BitConverter.ToSingle(byteArray, z_posi);

            pcl[i] = new Vector3(-y, z, x);

            if (rgb){
                rgb_posi = i * point_step + 16;
                uint rgb_value = BitConverter.ToUInt32(byteArray, rgb_posi);

                // 각 색상(R, G, B) 추출
                byte r = (byte)((rgb_value >> 16) & 0xFF);  // 상위 8비트 (Red)
                byte g = (byte)((rgb_value >> 8) & 0xFF);   // 중간 8비트 (Green)
                byte b = (byte)(rgb_value & 0xFF);          // 하위 8비트 (Blue)

                // RGB 값을 float 타입으로 변환 (0.0f ~ 1.0f 범위로 변환)

                float r_float = r / 255.0f;
                float g_float = g / 255.0f;
                float b_float = b / 255.0f;
            
                pcl_color[i] = new Color(r_float, g_float, b_float);

                if (i == 0)
                {
                    color = pcl_color[i];
                }
            }
            else if (intensity)
            {
                intensity_posi = i * point_step + 12;
                float intensity = BitConverter.ToSingle(byteArray, intensity_posi);

                // TODO - Intensity 값을 이용하여 색상을 설정

                pcl_color[i] = new Color(0.0f, 0.0f, 0.0f);
            }
        }
    }
}

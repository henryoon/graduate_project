using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using System;
using System.Collections.Generic;

public class SpineTransformPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private PoseMsg msg;
    
    public string topicName;
    public float publishRate = 1.0f;
    public Transform spineRootTarget;

    private float timeElapsed;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);

        msg = new PoseMsg();
    }

    void Update()
    {
        PublishMessage();
    }

    private PoseMsg CreatePoseMsg(Transform target)
    {
        Vector3 transformed_position = RosToUnityTransform.ReverseConvertPosition(target.position);
        Quaternion transformed_rotation = RosToUnityTransform.ReverseConvertRotation(target.rotation);

        PoseMsg poseMsg = new PoseMsg();

        poseMsg.position = new PointMsg();
        poseMsg.position.x = transformed_position.x;
        poseMsg.position.y = transformed_position.y;
        poseMsg.position.z = transformed_position.z;
        
        poseMsg.orientation = new QuaternionMsg();
        poseMsg.orientation.x = transformed_rotation.x;
        poseMsg.orientation.y = transformed_rotation.y;
        poseMsg.orientation.z = transformed_rotation.z;
        poseMsg.orientation.w = transformed_rotation.w;

        return poseMsg;
    }


    private void PublishMessage(){
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishRate)
        {
            msg = CreatePoseMsg(spineRootTarget);
            timeElapsed = 0.0f;
            ros.Publish(topicName, msg);
        }
    }
}
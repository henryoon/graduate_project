using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using System;
using System.Collections.Generic;

public class ControllerStatePublisher : MonoBehaviour
{
    private ROSConnection ros;
    private PoseMsg head_controller_pose_msg;
    private PoseMsg left_controller_pose_msg;
    private TwistMsg left_controller_twist_msg;
    private PoseMsg right_controller_pose_msg;
    private TwistMsg right_controller_twist_msg;
    
    public string head_controller_pose_topic;
    public ControllerState head_controller_state;
    public string left_controller_pose_topic;
    public string left_controller_twist_topic;
    public ControllerState left_controller_state;
    public string right_controller_pose_topic;
    public string right_controller_twist_topic;
    public ControllerState right_controller_state;
    
    public float publishRate = 1.0f;
    private float timeElapsed;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(head_controller_pose_topic);
        ros.RegisterPublisher<PoseMsg>(left_controller_pose_topic);
        ros.RegisterPublisher<PoseMsg>(right_controller_pose_topic);
        ros.RegisterPublisher<TwistMsg>(left_controller_twist_topic);
        ros.RegisterPublisher<TwistMsg>(right_controller_twist_topic);

        head_controller_pose_msg = new PoseMsg();
        left_controller_pose_msg = new PoseMsg();
        right_controller_pose_msg = new PoseMsg();
        left_controller_twist_msg = new TwistMsg();
        right_controller_twist_msg = new TwistMsg();
    }

    void Update()
    {
        PublishMessage();
    }

    private PoseMsg CreatePoseMsg(ControllerState controller_state){
        PoseMsg pose_msg = new PoseMsg();

        pose_msg.position = new PointMsg();

        Vector3 transformed_position = RosToUnityTransform.ReverseConvertPosition(controller_state.position);

        pose_msg.position.x = transformed_position.x;
        pose_msg.position.y = transformed_position.y;
        pose_msg.position.z = transformed_position.z;

        pose_msg.orientation = new QuaternionMsg();

        Quaternion transformed_rotation = RosToUnityTransform.ReverseConvertRotation(controller_state.rotation);

        pose_msg.orientation.x = transformed_rotation.x;
        pose_msg.orientation.y = transformed_rotation.y;
        pose_msg.orientation.z = transformed_rotation.z;
        pose_msg.orientation.w = transformed_rotation.w;

        return pose_msg;
    }

    private TwistMsg CreateTwistMsg(ControllerState controller_state){
        TwistMsg twist_msg = new TwistMsg();

        twist_msg.linear = new Vector3Msg();
        twist_msg.angular = new Vector3Msg();

        Vector3 transformed_linear = RosToUnityTransform.ReverseConvertPosition(controller_state.linear_velocity);
        Vector3 transformed_angular = RosToUnityTransform.ReverseConvertPosition(controller_state.angular_velocity);

        twist_msg.linear.x = transformed_linear.x;
        twist_msg.linear.y = transformed_linear.y;
        twist_msg.linear.z = transformed_linear.z;

        twist_msg.angular.x = transformed_angular.x;
        twist_msg.angular.y = transformed_angular.y;
        twist_msg.angular.z = transformed_angular.z;

        return twist_msg;
    }

    private void PublishMessage(){
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishRate)
        {
            head_controller_pose_msg = CreatePoseMsg(head_controller_state);
            left_controller_pose_msg = CreatePoseMsg(left_controller_state);
            right_controller_pose_msg = CreatePoseMsg(right_controller_state);

            left_controller_twist_msg = CreateTwistMsg(left_controller_state);
            right_controller_twist_msg = CreateTwistMsg(right_controller_state);

            timeElapsed = 0.0f;

            ros.Publish(head_controller_pose_topic, head_controller_pose_msg);
            ros.Publish(left_controller_pose_topic, left_controller_pose_msg);
            ros.Publish(right_controller_pose_topic, right_controller_pose_msg);

            ros.Publish(left_controller_twist_topic, left_controller_twist_msg);
            ros.Publish(right_controller_twist_topic, right_controller_twist_msg);
        }
    }
}
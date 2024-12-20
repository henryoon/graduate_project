using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Visualization;
using Unity.Robotics.ROSTCPConnector;
using System;
using System.Collections.Generic;

public class ControllerJoyPublisher : MonoBehaviour
{
    private ROSConnection ros;

    private JoyMsg left_joy_msg;
    private JoyMsg right_joy_msg;

    public ControllerState left_controller_state;
    public ControllerState right_controller_state;
    
    public string left_controller_joy_topic;
    public string right_controller_joy_topic;

    public float publishRate = 1.0f;
    private float timeElapsed;
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JoyMsg>(left_controller_joy_topic);
        ros.RegisterPublisher<JoyMsg>(right_controller_joy_topic);

        left_joy_msg = new JoyMsg();
        right_joy_msg = new JoyMsg();
    }

    void Update()
    {
        PublishMessage();
    }

    private JoyMsg CreateJoyMsg(ControllerState controller_state){
        JoyMsg joy_msg = new JoyMsg();

        joy_msg.header = new HeaderMsg();

        joy_msg.axes = new float[2];
        joy_msg.buttons = new int[4];

        joy_msg.axes[0] = controller_state.primary2DAxis.x;
        joy_msg.axes[1] = controller_state.primary2DAxis.y;

        joy_msg.buttons[0] = controller_state.isPrimaryButtonPressed ? 1 : 0;
        joy_msg.buttons[1] = controller_state.isSecondaryButtonPressed ? 1 : 0;
        joy_msg.buttons[2] = controller_state.isGripButtonPressed ? 1 : 0;
        joy_msg.buttons[3] = controller_state.isTriggerButtonPressed ? 1 : 0;
        
        return joy_msg;
    }

    private void PublishMessage(){
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishRate)
        {
            left_joy_msg = CreateJoyMsg(left_controller_state);
            right_joy_msg = CreateJoyMsg(right_controller_state);

            timeElapsed = 0.0f;
            ros.Publish(left_controller_joy_topic, left_joy_msg);
            ros.Publish(right_controller_joy_topic, right_joy_msg);
        }
    }
}
using UnityEngine;
using UnityEngine.XR;


public class UpdateController : MonoBehaviour
{
    public ControllerState head_controller_state;
    public ControllerState left_controller_state;
    public ControllerState right_controller_state;


    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
    }

    void Update()
    {
        UpdateControllerState(XRNode.Head, head_controller_state);
        UpdateControllerState(XRNode.LeftHand, left_controller_state);
        UpdateControllerState(XRNode.RightHand, right_controller_state);
    }

    void UpdateControllerState(XRNode node, ControllerState controllerState)
    {
        InputDevice device = InputDevices.GetDeviceAtXRNode(node);

        device.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 position);
        device.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rotation);
        device.TryGetFeatureValue(CommonUsages.deviceVelocity, out Vector3 linear_velocity);
        device.TryGetFeatureValue(CommonUsages.deviceAngularVelocity, out Vector3 angular_velocity);
        device.TryGetFeatureValue(CommonUsages.primaryButton, out bool isPrimaryButtonPressed);
        device.TryGetFeatureValue(CommonUsages.secondaryButton, out bool isSecondaryButtonPressed);
        device.TryGetFeatureValue(CommonUsages.gripButton, out bool isGripButtonPressed);
        device.TryGetFeatureValue(CommonUsages.triggerButton, out bool isTriggerButtonPressed);
        device.TryGetFeatureValue(CommonUsages.primary2DAxis, out Vector2 primary2DAxis);
        device.TryGetFeatureValue(CommonUsages.secondary2DAxis, out Vector2 secondary2DAxis);

        controllerState.position = position;
        controllerState.rotation = rotation;
        controllerState.linear_velocity = linear_velocity;
        controllerState.angular_velocity = angular_velocity;
        controllerState.primary2DAxis = primary2DAxis;
        controllerState.secondary2DAxis = secondary2DAxis;
        controllerState.isPrimaryButtonPressed = isPrimaryButtonPressed;
        controllerState.isSecondaryButtonPressed = isSecondaryButtonPressed;
        controllerState.isGripButtonPressed = isGripButtonPressed;
        controllerState.isTriggerButtonPressed = isTriggerButtonPressed;
    }
}

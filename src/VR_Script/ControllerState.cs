using UnityEngine;

[CreateAssetMenu(fileName = "New Controller State", menuName = "XR/Controller State", order = 1)]
public class ControllerState : ScriptableObject
{
    public Vector3 position = new Vector3(0.0f, 0.0f, 0.0f);
    public Quaternion rotation = new Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
    public Vector3 linear_velocity = new Vector3(0.0f, 0.0f, 0.0f);
    public Vector3 angular_velocity = new Vector3(0.0f, 0.0f, 0.0f);
    public Vector2 primary2DAxis = new Vector2(0.0f, 0.0f);
    public Vector2 secondary2DAxis = new Vector2(0.0f, 0.0f);
    public bool isPrimaryButtonPressed = false;
    public bool isSecondaryButtonPressed = false;
    public bool isGripButtonPressed = false;
    public bool isTriggerButtonPressed = false;
    public bool isTouching = false;
}
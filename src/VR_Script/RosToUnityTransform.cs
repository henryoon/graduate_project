using UnityEngine;

public class RosToUnityTransform : MonoBehaviour
{
    // Method to convert ROS position to Unity position
    public static Vector3 ConvertPosition(Vector3 rosPosition)
    {
        // Apply the transformation: (ros_x, ros_y, ros_z) -> (-ros_y, ros_z, ros_x)
        return new Vector3(-rosPosition.y, rosPosition.z, rosPosition.x);
    }

    public static Vector3 ReverseConvertPosition(Vector3 unityPosition)
    {
        return new Vector3(unityPosition.z, -unityPosition.x, unityPosition.y);
    }

    // Method to convert ROS rotation (quaternion) to Unity rotation
    public static Quaternion ConvertRotation(Quaternion rosRotation)
    {
        // Apply the transformation: (ros_x, ros_y, ros_z, ros_w) -> (ros_x, ros_z, -ros_y, ros_w)
        return new Quaternion(-rosRotation.y, rosRotation.z, rosRotation.x, rosRotation.w);
    }

    public static Quaternion ReverseConvertRotation(Quaternion unityRotation)
    {
        return new Quaternion(unityRotation.z, -unityRotation.x, unityRotation.y, unityRotation.w);
    }

    public static Vector3 ConvertScale(Vector3 rosScale)
    {
        return new Vector3(rosScale.y, rosScale.z, rosScale.x);
    }

    public static Vector3 ReverseConvertScale(Vector3 unityScale)
    {
        return new Vector3(unityScale.z, unityScale.x, unityScale.y);
    }

    // Example usage of the transformation
    public void ApplyRosData(Vector3 rosPosition, Quaternion rosRotation)
    {
        // Convert position and rotation
        Vector3 unityPosition = ConvertPosition(rosPosition);
        Quaternion unityRotation = ConvertRotation(rosRotation);

        // Apply the converted position and rotation to this object
        transform.position = unityPosition;
        transform.rotation = unityRotation;
    }
}
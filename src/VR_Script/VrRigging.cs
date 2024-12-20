using UnityEngine;
using UnityEngine.XR;

public class VrRigging : MonoBehaviour
{
    public float offsetDistance = 0.5f;  // 원하는 오프셋 거리 (예: 0.3m)
    public Transform headTarget;
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public Transform hipTarget;
    public Transform spineRootTarget;
    public Transform spineTipTarget;
    public Transform leftLegTarget;
    public Transform rightLegTarget;

    public ControllerState head_controller_state;
    public ControllerState left_controller_state;
    public ControllerState right_controller_state;

    private Vector3 left_leg_initial_position;
    private Vector3 right_leg_initial_position;
    private Transform head_transform;

    void Start()
    {
        head_transform = new GameObject().transform;
        left_leg_initial_position = new Vector3(leftLegTarget.position.x, leftLegTarget.position.y, leftLegTarget.position.z);
        right_leg_initial_position = new Vector3(rightLegTarget.position.x, rightLegTarget.position.y, rightLegTarget.position.z);
    }

    void Update()
    {
    }

    void LateUpdate()
    {
        if (head_controller_state == null || left_controller_state == null || right_controller_state == null)
        {
            Debug.Log("Controller is null");
            return;
        }
        // Set Target
        leftHandTarget.position = left_controller_state.position;
        leftHandTarget.rotation = left_controller_state.rotation * Quaternion.Euler(0, 90, 0);

        rightHandTarget.position = right_controller_state.position;
        rightHandTarget.rotation = right_controller_state.rotation * Quaternion.Euler(0, -90, 0);

        head_transform.position = head_controller_state.position;
        head_transform.rotation = head_controller_state.rotation;

        headTarget.position = head_transform.position + head_transform.forward * offsetDistance;
        hipTarget.position = new Vector3(head_transform.position.x, hipTarget.position.y, head_transform.position.z);
        leftLegTarget.position = new Vector3(head_transform.position.x, 0.0f, head_transform.position.z) + left_leg_initial_position;
        rightLegTarget.position = new Vector3(head_transform.position.x, 0.0f, head_transform.position.z) + right_leg_initial_position;
    
        UpdateTorsoRotation();
    }


void UpdateTorsoRotation()
{
    // Get the direction vectors from head to left and right hands
    Vector3 headToLeft = left_controller_state.position - head_controller_state.position;
    Vector3 headToRight = right_controller_state.position - head_controller_state.position;

    // Calculate a forward direction that is based on the average of the left and right hands' directions
    Vector3 torsoForward = Vector3.Lerp(headToLeft.normalized, headToRight.normalized, 0.5f);
    torsoForward.y = 0; // Keep the forward direction on the horizontal plane


    if (torsoForward == Vector3.zero)
    {
        return;
    }

    // Calculate the target rotation based on the torso's forward direction
    Quaternion targetRotation = Quaternion.LookRotation(torsoForward, Vector3.up);

    // Apply rotation to hip and spine
    hipTarget.rotation = targetRotation;
    spineRootTarget.rotation = Quaternion.Slerp(spineRootTarget.rotation, targetRotation, 0.5f); // Smoother transition for spine
    spineTipTarget.rotation = Quaternion.Slerp(spineTipTarget.rotation, targetRotation, 0.5f); // Smoother transition for spine
}
}

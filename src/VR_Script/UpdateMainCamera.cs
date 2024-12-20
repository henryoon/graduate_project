using UnityEngine;
using UnityEngine.XR;

public class UpdateMainCamera : MonoBehaviour
{
    public GameObject cameraOffset;
    public ControllerState head_controller_state;
    public ControllerState left_controller_state;

    public float offsetDistance = 0.5f;  // 원하는 오프셋 거리 (예: 0.3m)

    private Transform headTarget;

    void Start()
    {
        headTarget = new GameObject().transform;
        headTarget.position = head_controller_state.position;
        headTarget.rotation = head_controller_state.rotation;
    }

    void Update()
    {
        Vector3 left_controller_joy = new Vector3(left_controller_state.primary2DAxis.x, 0.0f, left_controller_state.primary2DAxis.y);

        Vector3 moveDirection = headTarget.transform.right * left_controller_joy.x + headTarget.transform.forward * left_controller_joy.z;

        headTarget.position = headTarget.position + moveDirection * Time.deltaTime;
        headTarget.rotation = head_controller_state.rotation;
    }

    void LateUpdate()
    {
        Vector3 headPosition = headTarget.position - headTarget.forward * offsetDistance;
        cameraOffset.transform.position = new Vector3(headPosition.x, 0.0f, headPosition.z);
    }

    void FixedUpdate()
    {
    }
}
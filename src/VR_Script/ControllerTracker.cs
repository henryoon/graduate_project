using UnityEngine;
using UnityEngine.XR;

public class ControllerTracker : MonoBehaviour
{
    public GameObject controller;
    public ControllerState controllerState;

    void Start()
    {
    }

    void Update()
    {
        if (controllerState == null)
        {
            Debug.Log("Controller is null");
            return;
        }
        controller.transform.position = controllerState.position;
        controller.transform.rotation = controllerState.rotation;
    }
}
using UnityEngine;
using UnityEngine.UI;

public class OpenUI : MonoBehaviour
{
    public ControllerState right_controller;
    public GameObject UIScreen; // 빈 화면(Panel)을 참조

    private bool is_pressed = false;

    void Start()
    {
        UIScreen.SetActive(true); // 처음에는 화면을 켜놓는다
    }
    
    void Update()
    {
        // 오른손 컨트롤러의 트리거 버튼을 누르면
        if (right_controller.isPrimaryButtonPressed)
        {
            // 버튼을 누르고 있을 때
            if (!is_pressed)
            {
                is_pressed = true;
                ToggleYoutubeScreen();
            }
        }
        else
        {
            // 버튼을 누르지 않았을 때
            is_pressed = false;
        }
    }

    public void ToggleYoutubeScreen()
    {
        // 화면을 켜거나 끄기
        UIScreen.SetActive(!UIScreen.activeSelf);
    }
}

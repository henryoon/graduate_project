using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class XRRayInteractor : MonoBehaviour
{
	// 1. 우선 해당 스크립트를 싱글톤으로 만들었다.
    public static XRRayInteractor instance;
    
    
    // 2. 마우스 역할을 수행할 게임오브젝트를 준비한다.
    // VR 오른쪽 컨트롤러
    public Transform rightHand;
    // 마우스 포인터를 대체할 이미지
    public Transform dot;

    private void Awake()
    {
        instance = this;
    }

    void Update()
    {
    	// 3. Ray를 사용해서 dot(마우스포인터)를 활성화한다.
        Ray ray = new Ray(rightHand.position, rightHand.forward);
        RaycastHit hit;
        if (Physics.Raycast(ray, out hit))
        {
            if (hit.transform.gameObject.layer == LayerMask.NameToLayer("UI"))
            {
                dot.gameObject.SetActive(true);
                dot.position = hit.point;
            }
            else
            {
                dot.gameObject.SetActive(false);
            }
            
            
            // 4. dot이 충돌 중 일 때 클릭할 수 있도록 한다.
            // 만약 점이 활성화 상태면
            if (dot.gameObject.activeSelf)
            {
                if (OVRInput.GetDown(OVRInput.Button.Any))
                {
                    // 버튼 스크립트를 가져온다
                    Button btn = hit.transform.GetComponent<Button>();
                    // 만약 btn이 null이 아니라면
                    if (btn != null)
                    {
                        btn.onClick.Invoke();
                    }
                }
            }
        }
        else
        {
            dot.gameObject.SetActive(false);
        }
    }
}
using UnityEngine;

namespace ExtScripts
{
    public class CameraMotion : MonoBehaviour
    {
        private bool _pressed;

        private void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                _pressed = true;
                if (Camera.main != null)
                {
                    var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                }
            }

            if (Input.GetMouseButtonUp(0))
                _pressed = false;

            if (_pressed)
            {
                {
                    float h;

                    h = 5.0f * Input.GetAxis("Mouse Y");
                    transform.Rotate(h, 0, 0);

                    h = 5.0f * Input.GetAxis("Mouse X");
                    if (Camera.main != null) Camera.main.transform.RotateAround(new Vector3(0, 0, 0), Vector3.up, h);
                }
            }
        }
    }
}
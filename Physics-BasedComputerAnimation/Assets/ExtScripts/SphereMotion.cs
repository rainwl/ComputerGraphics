using UnityEngine;

namespace ExtScripts
{
    public class SphereMotion : MonoBehaviour
    {
        #region Fields

        private bool _pressed;
        private bool _sphereMove;
        private Vector3 _offset;
        private Camera _camera;
        private bool _isCameraNotNull;

        #endregion

        #region Unity Methods

        private void Start()
        {
            _camera = Camera.main;
            _isCameraNotNull = _camera != null;
        }

        private void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                _pressed = true;
                if (_isCameraNotNull)
                {
                    var ray = _camera.ScreenPointToRay(Input.mousePosition);
                    _sphereMove = Vector3.Cross(ray.direction, transform.position - ray.origin).magnitude < 2.5f;
                }

                _offset = Input.mousePosition - _camera.WorldToScreenPoint(transform.position);
            }

            if (Input.GetMouseButtonUp(0))
                _pressed = false;
            if (!_pressed) return;
            if (_sphereMove)
            {
                var mouse = Input.mousePosition;
                mouse -= _offset;
                mouse.z = _camera.WorldToScreenPoint(transform.position).z;
                transform.position = _camera.ScreenToWorldPoint(mouse);
            }
            else
            {
                var h = 2.0f * Input.GetAxis("Mouse X");
                _camera.transform.RotateAround(new Vector3(0, 0, 0), Vector3.up, h);
            }
        }

        #endregion
    }
}
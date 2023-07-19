using UnityEngine;

namespace ExtScripts
{
    public class CameraFlow : MonoBehaviour
    {
        public Transform target;
        public float distanceUp=0.2f;
        public float distanceAway = 1f;
        public float smooth = 2f;
        public float camDepthSmooth = 5f;
        private Camera _camera;
        private bool _isCameraNotNull;

        private void Start()
        {
            _isCameraNotNull = _camera != null;
            _camera = Camera.main;
        }

        private void Update () 
        {
            if (_isCameraNotNull && ((Input.mouseScrollDelta.y < 0 && _camera.fieldOfView >= 3) || Input.mouseScrollDelta.y > 0 && _camera.fieldOfView <= 80))
            {
                _camera.fieldOfView += Input.mouseScrollDelta.y * camDepthSmooth * Time.deltaTime;
            }
        }

        private void LateUpdate()
        {
            var position = target.position;
            var disPos = position + Vector3.up * distanceUp - target.forward * distanceAway;   transform.position=Vector3.Lerp(transform.position,disPos,Time.deltaTime*smooth);
            transform.LookAt(position);
        }
    }
}

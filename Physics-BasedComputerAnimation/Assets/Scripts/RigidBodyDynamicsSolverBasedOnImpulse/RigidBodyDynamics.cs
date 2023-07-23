using System;
using UnityEngine;

namespace RigidBodyDynamicsSolverBasedOnImpulse
{
    public class RigidBodyDynamics : MonoBehaviour
    {
        #region Fields

        [SerializeField] private bool launched;
        [SerializeField] private float dt = 0.015f;
        [SerializeField] private float linearDecay = 0.999f; // for velocity decay
        [SerializeField] private float angularDecay = 0.98f;
        [SerializeField] private float restitution = 0.5f; // for collision
        [SerializeField] private float friction = 0.2f;

        private Vector3 _v = new Vector3(0, 0, 0); // velocity
        private Vector3 _w = new Vector3(0, 0, 0); // angular velocity
        private float _mass; // mass
        private Matrix4x4 _inertiaRef; // reference inertia
        private readonly Vector3 _gravity = new Vector3(0, -9.8f, 0);
        private Mesh _mesh;
        private Vector3[] _vertices;

        #endregion

        #region Unity Methods

        private void Start()
        {
            _mesh = GetComponent<MeshFilter>().mesh;
            _vertices = _mesh.vertices;

            const float m = 1;
            _mass = 0;
            for (var i = 0; i < _vertices.Length; i++)
            {
                _mass += m;
                var diag = m * _vertices[i].sqrMagnitude;
                _inertiaRef[0, 0] += diag;
                _inertiaRef[1, 1] += diag;
                _inertiaRef[2, 2] += diag;
                _inertiaRef[0, 0] -= m * _vertices[i][0] * _vertices[i][0];
                _inertiaRef[0, 1] -= m * _vertices[i][0] * _vertices[i][1];
                _inertiaRef[0, 2] -= m * _vertices[i][0] * _vertices[i][2];
                _inertiaRef[1, 0] -= m * _vertices[i][1] * _vertices[i][0];
                _inertiaRef[1, 1] -= m * _vertices[i][1] * _vertices[i][1];
                _inertiaRef[1, 2] -= m * _vertices[i][1] * _vertices[i][2];
                _inertiaRef[2, 0] -= m * _vertices[i][2] * _vertices[i][0];
                _inertiaRef[2, 1] -= m * _vertices[i][2] * _vertices[i][1];
                _inertiaRef[2, 2] -= m * _vertices[i][2] * _vertices[i][2];
            }

            _inertiaRef[3, 3] = 1;
        }

        private void Update()
        {
            if (Input.GetKey(KeyCode.R))
            {
                var trans = transform;
                trans.position = new Vector3(1, 1f, -1);
                trans.rotation = Quaternion.identity;
                restitution = 0.5f;
                launched = false;
            }

            if (Input.GetKey(KeyCode.Space))
            {
                _v = new Vector3(-2, 2, 2);
                launched = true;
            }


            if (!launched) return;

            // <Update the velocities>
            _v += dt * _gravity;
            _v *= linearDecay;
            _w *= angularDecay;

            // <Do Collision Impulse>
            CollisionImpulse(new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            CollisionImpulse(new Vector3(0, 0, 0), new Vector3(0, 0, -1));
            CollisionImpulse(new Vector3(0, 0, 0), new Vector3(1, 0, 0));

            // <Update the position and orientation>
            var t = transform;
            var x0 = t.position;
            var q0 = t.rotation;

            // <Do leap frog integration>
            var x = x0 + dt * _v;
            var dw = 0.5f * dt * _w;
            var qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
            var q = ExtScripts.ExtFunctions.Add(q0, qw * q0);

            // <Assign to the object>
            t.position = x;
            t.rotation = q;
        }

        #endregion

        #region Private Methods

        private void CollisionImpulse(Vector3 point, Vector3 normal)
        {
            // R <- Matrix.Rotate(q)
            var matrixR = Matrix4x4.Rotate(transform.rotation);
            var T = transform.position;

            // use average of many vertices in collision
            var sum = new Vector3(0, 0, 0);
            var collisionNum = 0;

            // for every vertex: xi = x + Rri
            // test if fun(xi) < 0
            foreach (var ri in _vertices)
            {
                // the current state (world space) : R*ri
                var matrixRri = matrixR.MultiplyVector(ri);
                // Translate by x : xi = x + R*ri
                var xi = T + matrixRri;
                // the distance of the vertex between the plane (constraint)
                var d = Vector3.Dot(xi - point, normal);
                // if inside
                if (!(d < 0.0f)) continue;
                // We cannot directly modify xi or ri,since they are not state variables
                // vi = v+ w X R*ri (linear velocity + angular velocity (cross product))
                var vi = _v + Vector3.Cross(_w, matrixRri);
                // we use v*N to calculate the norm of the VN
                var vNSize = Vector3.Dot(vi, normal);
                // if VN is also in
                if (!(vNSize < 0.0f)) continue;
                sum += ri;
                collisionNum++;
            }

            // if there's no collision
            if (collisionNum == 0) return;

            // inertia:R*I_ref*RT
            var inertiaRot = matrixR * _inertiaRef * Matrix4x4.Transpose(matrixR);
            // inertia.inverse:(R*I_ref*RT).Inverse
            var inertiaRotInverse = Matrix4x4.Inverse(inertiaRot);
            // average rc
            var rCollision = sum / collisionNum;
            // the current state (world space) : R*rc
            var matrixRrCollision = matrixR.MultiplyVector(rCollision);

            // <Rigid Body Collision Response by impulse>
            // vc_new = v_new + w_new X Rrc
            var vCollision = _v + Vector3.Cross(_w, matrixRrCollision);

            // <Compute the wanted Vc_new>
            // VN = (vc * N)N 
            var vN = Vector3.Dot(vCollision, normal) * normal;
            // VT = vc - VN
            var vT = vCollision - vN;
            // VN_new = -μN*VN
            var vNNew = -1.0f * restitution * vN;
            // VT_new = a * VT
            // because of the V_new = VN_new + VT_new
            // ||VT_new - VT|| <&= μN*||VN_new - VN||
            // (1-a)*||VT|| <&= μT*(1+μN)*||VN||
            // a = max(1-μT(1+μN)||Vn|| / ||VT||,0)
            // dynamic friction and static friction
            var a = Math.Max(1.0f - friction * (1.0f + restitution) * vN.magnitude / vT.magnitude, 0.0f);
            var vTNew = a * vT;
            // V_new = VN_new + VT_new
            var vNew = vNNew + vTNew;
            // convert 'r X '(cross product) into a matrix product 'r*'
            // (Rrc)*
            var matrixRriStar = ExtScripts.ExtFunctions.GetCrossMatrix(matrixRrCollision);

            // <Compute the impulse j>
            // K = 1/M*identity - (Rrc)* I.inverse (Rrc)*
            var k = ExtScripts.ExtFunctions.MatrixSubtraction(
                ExtScripts.ExtFunctions.MatrixMultiplyFloat(Matrix4x4.identity, 1.0f / _mass),
                matrixRriStar * inertiaRotInverse * matrixRriStar);
            // j = K.inverse(Vc_new - vc)
            var j = k.inverse.MultiplyVector(vNew - vCollision);

            // <Update v and w>
            // v = v + 1/M * j
            _v += 1.0f / _mass * j;
            // w = w + I.inverse*(Rrc X j)
            _w += inertiaRotInverse.MultiplyVector(Vector3.Cross(matrixRrCollision, j));
        }

        #endregion
    }
}
using System;
using UnityEngine;

namespace RigidBodyDynamicsSolverBasedOnImpulse
{
    public class RigidBodyDynamics : MonoBehaviour
    {
        [SerializeField] private bool launched = false;
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

        private static Matrix4x4 GetCrossMatrix(Vector3 a)
        {
            //Get the cross product matrix of vector a
            var vectorA = Matrix4x4.zero;
            vectorA[0, 0] = 0;
            vectorA[0, 1] = -a[2];
            vectorA[0, 2] = a[1];
            vectorA[1, 0] = a[2];
            vectorA[1, 1] = 0;
            vectorA[1, 2] = -a[0];
            vectorA[2, 0] = -a[1];
            vectorA[2, 1] = a[0];
            vectorA[2, 2] = 0;
            vectorA[3, 3] = 1;
            return vectorA;
        }

        private static Matrix4x4 MatrixSubtraction(Matrix4x4 a, Matrix4x4 b)
        {
            for (var i = 0; i < 4; ++i)
            {
                for (var j = 0; j < 4; ++j)
                {
                    a[i, j] -= b[i, j];
                }
            }

            return a;
        }

        private static Matrix4x4 MatrixMultiplyFloat(Matrix4x4 a, float b)
        {
            for (var i = 0; i < 4; ++i)
            {
                for (var j = 0; j < 4; ++j)
                {
                    a[i, j] *= b;
                }
            }

            return a;
        }

        private static Quaternion Add(Quaternion a, Quaternion b)
        {
            a.x += b.x;
            a.y += b.y;
            a.z += b.z;
            a.w += b.w;
            return a.normalized;
        }


        /// <summary>
        /// In this function, update v and w by the impulse due to the collision with a plane P,N
        /// </summary>
        /// <param name="point"></param>
        /// <param name="normal"></param>
        private void CollisionImpulse(Vector3 point, Vector3 normal)
        {
            var vertices = _mesh.vertices;

            var matrixR = Matrix4x4.Rotate(transform.rotation);
            var T = transform.position;

            var sum = new Vector3(0, 0, 0);
            var collisionNum = 0;


            for (var i = 0; i < vertices.Length; i++)
            {
                var ri = vertices[i];
                var matrixRri = matrixR.MultiplyVector(ri);
                var xi = T + matrixRri;
                var d = Vector3.Dot(xi - point, normal);
                if (d < 0.0f)
                {
                    var vi = _v + Vector3.Cross(_w, matrixRri);
                    var vNSize = Vector3.Dot(vi, normal);
                    if (vNSize < 0.0f)
                    {
                        sum += ri;
                        collisionNum++;
                    }
                }
            }

            if (collisionNum == 0) return;
            var inertiaRot = matrixR * _inertiaRef * Matrix4x4.Transpose(matrixR);
            var inertiaRotInverse = Matrix4x4.Inverse(inertiaRot);
            var rCollision = sum / collisionNum;
            var matrixRrCollision = matrixR.MultiplyVector(rCollision);
            var vCollision = _v + Vector3.Cross(_w, matrixRrCollision);
            var vN = Vector3.Dot(vCollision, normal) * normal;
            var vT = vCollision - vN;
            var vNNew = -1.0f * restitution * vN;
            var a = Math.Max(1.0f - friction * (1.0f + restitution) * vN.magnitude / vT.magnitude, 0.0f);
            var vTNew = a * vT;
            var vNew = vNNew + vTNew;
            var matrixRriStar = GetCrossMatrix(matrixRrCollision);
            var k = MatrixSubtraction(MatrixMultiplyFloat(Matrix4x4.identity, 1.0f / _mass),
                matrixRriStar * inertiaRotInverse * matrixRriStar);
            var j = k.inverse.MultiplyVector(vNew - vCollision);
            _v += 1.0f / _mass * j;
            _w += inertiaRotInverse.MultiplyVector(Vector3.Cross(matrixRrCollision, j));
        }

        private void Start()
        {
            _mesh = GetComponent<MeshFilter>().mesh;
            var vertices = _mesh.vertices;

            const float m = 1;
            _mass = 0;
            for (var i = 0; i < vertices.Length; i++)
            {
                _mass += m;
                var diag = m * vertices[i].sqrMagnitude;
                _inertiaRef[0, 0] += diag;
                _inertiaRef[1, 1] += diag;
                _inertiaRef[2, 2] += diag;
                _inertiaRef[0, 0] -= m * vertices[i][0] * vertices[i][0];
                _inertiaRef[0, 1] -= m * vertices[i][0] * vertices[i][1];
                _inertiaRef[0, 2] -= m * vertices[i][0] * vertices[i][2];
                _inertiaRef[1, 0] -= m * vertices[i][1] * vertices[i][0];
                _inertiaRef[1, 1] -= m * vertices[i][1] * vertices[i][1];
                _inertiaRef[1, 2] -= m * vertices[i][1] * vertices[i][2];
                _inertiaRef[2, 0] -= m * vertices[i][2] * vertices[i][0];
                _inertiaRef[2, 1] -= m * vertices[i][2] * vertices[i][1];
                _inertiaRef[2, 2] -= m * vertices[i][2] * vertices[i][2];
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

            // Update the velocities
            _v += dt * _gravity;
            _v *= linearDecay;
            _w *= angularDecay;

            // Do Collision Impulse
            CollisionImpulse(new Vector3(0, 0, 0), new Vector3(0, 1, 0));
            CollisionImpulse(new Vector3(0, 0, 0), new Vector3(0, 0, -1));
            CollisionImpulse(new Vector3(0, 0, 0), new Vector3(1, 0, 0));

            // Update the position and orientation
            var t0 = transform;
            var x0 = t0.position;
            var q0 = t0.rotation;

            var x = x0 + dt * _v;
            var dw = 0.5f * dt * _w;
            var qw = new Quaternion(dw.x, dw.y, dw.z, 0.0f);
            var q = Add(q0, qw * q0);

            // Part IV: Assign to the object

            var trans1 = transform;
            trans1.position = x;
            trans1.rotation = q;
        }
    }
}
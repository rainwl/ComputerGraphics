using System.Diagnostics.CodeAnalysis;
using System.Linq;
using UnityEngine;
// ReSharper disable InconsistentNaming

namespace ShapingMatching
{
    public class ShapeMatching : MonoBehaviour
    {
        #region Fields

        public bool launched = false;
        private Vector3[] X; // position of all vertices in world coordinate
        private Vector3[] _r; // position of all vertices in local coordinate ,readonly,constant ,rotation update on this
        private Vector3[] V; // velocity of all vertices
        private Matrix4x4 _rrt = Matrix4x4.zero;
        private Vector3[] P = { new Vector3(0, 0.01f, 0), new Vector3(2, 0, 0) }; // Points of walls. 
        private Vector3[] N = { new Vector3(0, 1, 0), new Vector3(-1, 0, 0) }; // Norm of walls.

        private const float DT = 0.015f; // Time step.
        private float g = -9.8f; // Acceleration of gravity.
        private float m = 1; // Mass of a particle
        private Matrix4x4 R;

        #endregion

        #region Private Methods

        /// <summary>
        /// Polar Decomposition
        /// </summary>
        /// <param name="matrix"></param>
        /// <returns></returns>
        [SuppressMessage("ReSharper", "InconsistentNaming")]
        private static Matrix4x4 Polar(Matrix4x4 matrix)
        {
            // ReSharper disable once InconsistentNaming
            var C = Matrix4x4.zero;
            for (var ii = 0; ii < 3; ii++)
            for (var jj = 0; jj < 3; jj++)
            for (var kk = 0; kk < 3; kk++)
                C[ii, jj] += matrix[kk, ii] * matrix[kk, jj];

            // ReSharper disable once InconsistentNaming
            var C2 = Matrix4x4.zero;
            for (var ii = 0; ii < 3; ii++)
            for (var jj = 0; jj < 3; jj++)
            for (var kk = 0; kk < 3; kk++)
                C2[ii, jj] += C[ii, kk] * C[jj, kk];

            var det = matrix[0, 0] * matrix[1, 1] * matrix[2, 2] +
                      matrix[0, 1] * matrix[1, 2] * matrix[2, 0] +
                      matrix[1, 0] * matrix[2, 1] * matrix[0, 2] -
                      matrix[0, 2] * matrix[1, 1] * matrix[2, 0] -
                      matrix[0, 1] * matrix[1, 0] * matrix[2, 2] -
                      matrix[0, 0] * matrix[1, 2] * matrix[2, 1];

            var I_c = C[0, 0] + C[1, 1] + C[2, 2];
            var I_c2 = I_c * I_c;
            var II_c = 0.5f * (I_c2 - C2[0, 0] - C2[1, 1] - C2[2, 2]);
            var III_c = det * det;
            var k = I_c2 - 3 * II_c;

            var inv_U = Matrix4x4.zero;
            if (k < 1e-10f)
            {
                float inv_lambda = 1 / Mathf.Sqrt(I_c / 3);
                inv_U[0, 0] = inv_lambda;
                inv_U[1, 1] = inv_lambda;
                inv_U[2, 2] = inv_lambda;
            }
            else
            {
                var l = I_c * (I_c * I_c - 4.5f * II_c) + 13.5f * III_c;
                var k_root = Mathf.Sqrt(k);
                var value = l / (k * k_root);
                if (value < -1.0f) value = -1.0f;
                if (value > 1.0f) value = 1.0f;
                var phi = Mathf.Acos(value);
                var lambda2 = (I_c + 2 * k_root * Mathf.Cos(phi / 3)) / 3.0f;
                var lambda = Mathf.Sqrt(lambda2);

                var III_u = Mathf.Sqrt(III_c);
                if (det < 0) III_u = -III_u;
                var I_u = lambda + Mathf.Sqrt(-lambda2 + I_c + 2 * III_u / lambda);
                var II_u = (I_u * I_u - I_c) * 0.5f;


                float inv_rate, factor;
                inv_rate = 1 / (I_u * II_u - III_u);
                factor = I_u * III_u * inv_rate;

                var U = Matrix4x4.zero;
                U[0, 0] = factor;
                U[1, 1] = factor;
                U[2, 2] = factor;

                factor = (I_u * I_u - II_u) * inv_rate;
                for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    U[i, j] += factor * C[i, j] - inv_rate * C2[i, j];

                inv_rate = 1 / III_u;
                factor = II_u * inv_rate;
                inv_U[0, 0] = factor;
                inv_U[1, 1] = factor;
                inv_U[2, 2] = factor;

                factor = -I_u * inv_rate;
                for (var i = 0; i < 3; i++)
                for (var j = 0; j < 3; j++)
                    inv_U[i, j] += factor * U[i, j] + inv_rate * C[i, j];
            }

            var R = Matrix4x4.zero;
            for (var ii = 0; ii < 3; ii++)
            for (var jj = 0; jj < 3; jj++)
            for (var kk = 0; kk < 3; kk++)
                R[ii, jj] += matrix[ii, kk] * inv_U[kk, jj];
            R[3, 3] = 1;
            return R;
        }

        
        /// <summary>
        /// Update the mesh vertices according to translation c and rotation R.
        /// It also updates the velocity.
        /// </summary>
        /// <param name="c"></param>
        /// <param name="R"></param>
        /// <param name="invDt"></param>
        // ReSharper disable once InconsistentNaming
        // ReSharper disable once ParameterHidesMember
        private void UpdateMesh(Vector3 c, Matrix4x4 R, float invDt)
        {
            // For every vertex ,update the v and x
            for (var i = 0; i < _r.Length; i++)
            {
                var x = (Vector3)(R * _r[i]) + c;
                V[i] += (x - X[i]) * invDt; // vi = (c+Rri - xi)/dt
                X[i] = x; // xi = c+Rri
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = X;
        }

        private void Collision(float invDt)
        {
            // Get current center.
            var c = new Vector3(0, 0, 0);
            for (var i = 0; i < V.Length; i++)
            {
                c += X[i];
            }

            c /= V.Length;

            for (var i = 0; i < X.Length; i++)
            {
                for (var n = 0; n < N.Length; n++)
                {
                    // Signed Distance Function
                    var phi = Vector3.Dot(c + R.MultiplyPoint(_r[i]) - P[n], N[n]);

                    if (phi < 0)
                    {
                        // Project the particle out of the wall (Similar to PBD method).
                        var x = X[i];
                        X[i] -= phi * N[n];

                        const float alpha = 1.0f; // Stiffness coefficient.
                        const float k = 20000.0f; // Penalty strength.
                        // Update velocity of the particle
                        V[i] += alpha * (X[i] - x) / DT + DT * k * Mathf.Abs(phi) * N[n] / m;

                        V[i] *= 0.4f; // restitution
                    }
                }
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = X;
        }

        #endregion

        #region Unity Methods

        private void Start()
        {
            var mesh = GetComponent<MeshFilter>().mesh;
            V = new Vector3[mesh.vertices.Length];
            X = mesh.vertices;
            _r = mesh.vertices;

            var c = _r.Aggregate(Vector3.zero, (current, t) => current + t);

            c /= _r.Length;
            for (var i = 0; i < _r.Length; i++)
                _r[i] -= c;

            //Get QQ^t ready.
            for (var i = 0; i < _r.Length; i++)
            {
                _rrt[0, 0] += _r[i][0] * _r[i][0];
                _rrt[0, 1] += _r[i][0] * _r[i][1];
                _rrt[0, 2] += _r[i][0] * _r[i][2];
                _rrt[1, 0] += _r[i][1] * _r[i][0];
                _rrt[1, 1] += _r[i][1] * _r[i][1];
                _rrt[1, 2] += _r[i][1] * _r[i][2];
                _rrt[2, 0] += _r[i][2] * _r[i][0];
                _rrt[2, 1] += _r[i][2] * _r[i][1];
                _rrt[2, 2] += _r[i][2] * _r[i][2];
            }

            _rrt[3, 3] = 1;

            //Initial velocity.
            for (var i = 0; i < X.Length; i++)
                V[i][0] = 10.0f;

            var transform1 = transform;
            var rot = transform1.rotation;
            var pos = transform1.position;
            UpdateMesh(pos, Matrix4x4.Rotate(rot), 0);
            pos = Vector3.zero;
            rot = Quaternion.identity;
        }

        private void Update()
        {
            // Launch the rabbit.
            if (Input.GetKey("l"))
            {
                launched = true;
            }

            if (!launched)
            {
                return;
            }

            // Step 1: run a simple particle system.
            for (var i = 0; i < V.Length; i++)
            {
                V[i].y += DT * g;
                X[i] += DT * V[i];
            }

            // Step 2: Perform simple particle collision.
            Collision(1 / DT);

            // Step 3: Use shape matching to get new translation c and 
            // Get current center: c.
            var c = new Vector3(0, 0, 0);

            for (var i = 0; i < V.Length; i++)
            {
                c += X[i];
            }

            c /= V.Length;

            // Get current orientation: R.
            
            // A = (sum:(yi-c)*riT)(sum:ri*riT)
            
            // sum:(yi-c)*riT
            var lrt = Matrix4x4.zero;
            for (var i = 0; i < V.Length; i++)
            {
                var l = X[i] - c;
                for (var j = 0; j < 3; j++)
                {
                    for (var k = 0; k < 3; k++)
                    {
                        lrt[j, k] += l[j] * _r[i][k];
                    }
                }
            }

            lrt[3, 3] = 1;
            
            var matrixA = lrt * _rrt;
            
            // Polar decomposition
            R = Polar(matrixA); 

            // Update vi and xi
            UpdateMesh(c, R, 1 / DT);
        }

        #endregion
    }
}
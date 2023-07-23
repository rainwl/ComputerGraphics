using System.Collections.Generic;
using UnityEngine;

namespace ImplicitClothSolver
{
    public class ImplicitSolver : MonoBehaviour
    {
        #region Fields

        private const float T = 0.0333f; // time step
        private const float Mass = 1; // mass of each vertex

        private const float Damping = 0.99f; // damping of velocity

        // private const float Rho = 0.995f; // Jacobi method
        private const float SpringK = 8000;
        private readonly Vector3 _gravity = new Vector3(0, -9.8f, 0);

        // ReSharper disable once InconsistentNaming
        private int[] E; // Edge spring array,every two elements store the indices of vertices that construct this edge

        // ReSharper disable once InconsistentNaming
        private float[] L; // length of each springs

        // ReSharper disable once InconsistentNaming
        private Vector3[] V; // store vertices

        #endregion

        #region Unity Methods

        private void Start()
        {
            var mesh = GetComponent<MeshFilter>().mesh;

            //Resize the mesh
            const int n = 21;
            // store the position of each vertex
            var x = new Vector3[n * n];
            // store the position of each vertex in UV 
            var uv = new Vector2[n * n];
            // store the index of vertex of each triangle ( index in verticesPos[] )
            var triangles =
                new int[(n - 1) * (n - 1) * 6]; // triangles = 20 * 20 * 2,every triangle has 3 vertices,so * 6

            for (var j = 0; j < n; j++)
            {
                for (var i = 0; i < n; i++)
                {
                    x[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                    uv[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
                }
            }

            // ReSharper disable once LocalVariableHidesMember
            var t = 0;
            for (var j = 0; j < n - 1; j++)
            {
                for (var i = 0; i < n - 1; i++)
                {
                    triangles[t * 6 + 0] = j * n + i;
                    triangles[t * 6 + 1] = j * n + i + 1;
                    triangles[t * 6 + 2] = (j + 1) * n + i + 1;
                    triangles[t * 6 + 3] = j * n + i;
                    triangles[t * 6 + 4] = (j + 1) * n + i + 1;
                    triangles[t * 6 + 5] = (j + 1) * n + i;
                    t++;
                }
            }

            mesh.vertices = x;
            mesh.triangles = triangles;
            mesh.uv = uv;
            mesh.RecalculateNormals();


            // Construct the original E
            // store the index of vertex in each edge
            // construct the spring system
            // ReSharper disable once InconsistentNaming
            var _E = new int[triangles.Length * 2];
            for (var i = 0; i < triangles.Length; i += 3)
            {
                _E[i * 2 + 0] = triangles[i + 0];
                _E[i * 2 + 1] = triangles[i + 1];
                _E[i * 2 + 2] = triangles[i + 1];
                _E[i * 2 + 3] = triangles[i + 2];
                _E[i * 2 + 4] = triangles[i + 2];
                _E[i * 2 + 5] = triangles[i + 0];
            }

            // Reorder the original edge list
            for (var i = 0; i < _E.Length; i += 2)
                if (_E[i] > _E[i + 1])
                    ExtScripts.ExtFunctions.Swap(ref _E[i], ref _E[i + 1]);

            // Sort the original edge list using quicksort
            ExtScripts.ExtFunctions.QuickSort(ref _E, 0, _E.Length / 2 - 1);

            var eNumber = 0;
            for (var i = 0; i < _E.Length; i += 2)
                if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                    eNumber++;

            // remove the repeat value
            E = new int[eNumber * 2];
            for (int i = 0, e = 0; i < _E.Length; i += 2)
                if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                {
                    E[e * 2 + 0] = _E[i + 0];
                    E[e * 2 + 1] = _E[i + 1];
                    e++;
                }

            // rest length of each spring
            L = new float[E.Length / 2];
            for (var e = 0; e < E.Length / 2; e++)
            {
                var v0 = E[e * 2 + 0];
                var v1 = E[e * 2 + 1];
                L[e] = (x[v0] - x[v1]).magnitude;
            }

            V = new Vector3[x.Length];
            for (var i = 0; i < V.Length; i++)
                V[i] = new Vector3(0, 0, 0);
        }

        private void Update()
        {
            var mesh = GetComponent<MeshFilter>().mesh;
            var x = mesh.vertices; // current position
            var lastX = new Vector3[x.Length]; // last position
            var xHat = new Vector3[x.Length]; // copy of current position, for update
            var g = new Vector3[x.Length];

            //Initial Setup.
            for (var i = 0; i < x.Length; i++)
            {
                if (i is 0 or 20) continue;
                V[i] *= Damping;
                xHat[i] = x[i] + T * V[i];
                x[i] = xHat[i];
            }


            const float omega = 1.0f;
            for (var k = 0; k < 32; k++)
            {
                //Chebyshev Acceleration
                // omega = k switch
                // {
                //     0 => 1.0f,
                //     1 => 2.0f / (2.0f - Rho * Rho),
                //     _ => 4.0f / (4 - Rho * Rho * omega)
                // };

                GetGradient(x, xHat, T, g);

                // Update X by gradient
                for (var i = 0; i < x.Length; i++)
                {
                    if (i is 0 or 20) continue;
                    var xNew = omega * (x[i] + (1.0f / (Mass / (T * T) + 4.0f * SpringK)) * -g[i]) +
                               (1.0f - omega) * lastX[i];
                    lastX[i] = x[i];
                    x[i] = xNew;
                }
            }

            //Finishing.
            for (var i = 0; i < x.Length; i++)
            {
                V[i] += (x[i] - xHat[i]) / T;
            }

            mesh.vertices = x;

            CollisionHandling();
            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        private void CollisionHandling()
        {
            var mesh = GetComponent<MeshFilter>().mesh;
            var x = mesh.vertices;

            //For every vertex, detect collision and apply impulse if needed.
            const float radius = 2.7f;
            var sphere = GameObject.Find("Sphere");
            var center = sphere.transform.position;
            for (var i = 0; i < x.Length; i++)
            {
                if (i is 0 or 20)
                    continue;
                var d = x[i] - center;
                if (!(d.magnitude < radius)) continue;
                var a = center + radius * d.normalized;
                V[i] += (a - x[i]) / T;
                x[i] = a;
            }

            mesh.vertices = x;
        }

        private void GetGradient(IReadOnlyList<Vector3> x, IReadOnlyList<Vector3> xHat, float t, IList<Vector3> g)
        {
            //Momentum and Gravity.
            for (var i = 0; i < x.Count; i++)
            {
                g[i] = Mass * (x[i] - xHat[i]) / (t * t) - Mass * _gravity;
            }

            //Spring Force.
            for (var e = 0; e < L.Length; e++)
            {
                var i = E[e * 2];
                var j = E[e * 2 + 1];
                g[i] = g[i] + SpringK * (1 - L[e] / (x[i] - x[j]).magnitude) * (x[i] - x[j]);
                g[j] = g[j] - SpringK * (1 - L[e] / (x[i] - x[j]).magnitude) * (x[i] - x[j]);
            }
        }

        #endregion
    }
}
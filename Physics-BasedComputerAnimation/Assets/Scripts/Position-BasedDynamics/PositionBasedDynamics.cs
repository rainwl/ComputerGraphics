using System;
using UnityEngine;
using Unity.Jobs;
using Unity.Collections;

namespace Position_BasedDynamics
{
    public class PositionBasedDynamics : MonoBehaviour
    {
        #region Fields

        private const float T = 0.0333f;
        private const float Damping = 0.99f;

        // ReSharper disable once InconsistentNaming
        private int[] E;

        // ReSharper disable once InconsistentNaming
        private float[] L;

        // ReSharper disable once InconsistentNaming
        private Vector3[] V;
        private readonly Vector3 _gravity = new Vector3(0, -9.8f, 0);

        #endregion

        public struct MyJob : IJob
        {
            public float a;
            public float b;
            public NativeArray<float> result;

            public void Execute()
            {
                result[0] = a + b;
            }
        }
        
        
        #region Unity Methods

        private void Start()
        {
            var mesh = GetComponent<MeshFilter>().mesh;

            //Resize the mesh.
            const int n = 21;
            var x = new Vector3[n * n];
            if (x == null) throw new ArgumentNullException(nameof(x));
            var uv = new Vector2[n * n];
            if (uv == null) throw new ArgumentNullException(nameof(uv));
            // ReSharper disable once LocalVariableHidesMember
            var T = new int[(n - 1) * (n - 1) * 6];
            for (var j = 0; j < n; j++)
            for (var i = 0; i < n; i++)
            {
                x[j * n + i] = new Vector3(5 - 10.0f * i / (n - 1), 0, 5 - 10.0f * j / (n - 1));
                uv[j * n + i] = new Vector3(i / (n - 1.0f), j / (n - 1.0f));
            }

            var t = 0;
            for (var j = 0; j < n - 1; j++)
            for (var i = 0; i < n - 1; i++)
            {
                T[t * 6 + 0] = j * n + i;
                T[t * 6 + 1] = j * n + i + 1;
                T[t * 6 + 2] = (j + 1) * n + i + 1;
                T[t * 6 + 3] = j * n + i;
                T[t * 6 + 4] = (j + 1) * n + i + 1;
                T[t * 6 + 5] = (j + 1) * n + i;
                t++;
            }

            mesh.vertices = x;
            mesh.triangles = T;
            mesh.uv = uv;
            mesh.RecalculateNormals();

            //Construct the original edge list
            // ReSharper disable once InconsistentNaming
            var _E = new int[T.Length * 2];
            for (var i = 0; i < T.Length; i += 3)
            {
                _E[i * 2 + 0] = T[i + 0];
                _E[i * 2 + 1] = T[i + 1];
                _E[i * 2 + 2] = T[i + 1];
                _E[i * 2 + 3] = T[i + 2];
                _E[i * 2 + 4] = T[i + 2];
                _E[i * 2 + 5] = T[i + 0];
            }

            //Reorder the original edge list
            for (var i = 0; i < _E.Length; i += 2)
                if (_E[i] > _E[i + 1])
                    ExtScripts.ExtFunctions.Swap(ref _E[i], ref _E[i + 1]);
            //Sort the original edge list using quicksort
            ExtScripts.ExtFunctions.QuickSort(ref _E, 0, _E.Length / 2 - 1);

            var eNumber = 0;
            for (var i = 0; i < _E.Length; i += 2)
                if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                    eNumber++;

            E = new int[eNumber * 2];
            for (int i = 0, e = 0; i < _E.Length; i += 2)
                if (i == 0 || _E[i + 0] != _E[i - 2] || _E[i + 1] != _E[i - 1])
                {
                    E[e * 2 + 0] = _E[i + 0];
                    E[e * 2 + 1] = _E[i + 1];
                    e++;
                }

            L = new float[E.Length / 2];
            for (var e = 0; e < E.Length / 2; e++)
            {
                var i = E[e * 2 + 0];
                var j = E[e * 2 + 1];
                L[e] = (x[i] - x[j]).magnitude;
            }

            V = new Vector3[x.Length];
            for (var i = 0; i < x.Length; i++)
                V[i] = new Vector3(0, 0, 0);
        }

        private void Update()
        {
            var mesh = GetComponent<MeshFilter>().mesh;
            var x = mesh.vertices;

            for (var i = 0; i < x.Length; i++)
            {
                if (i is 0 or 20) continue;

                // for every vertex ,damp the velocity 
                // Update the velocity by gravity , and finally update the position: xi = xi + dt * vi
                V[i] *= Damping;
                V[i] += _gravity * T;
                x[i] += V[i] * T;
            }

            mesh.vertices = x;

            for (var l = 0; l < 32; l++)
                StrainLimiting();

            CollisionHandling();

            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        private void StrainLimiting()
        {
            var mesh = GetComponent<MeshFilter>().mesh;
            var vertices = mesh.vertices;

            // implement position-based dynamics in a Jacobi fashion
            var sumX = new Vector3[vertices.Length]; // sum of vertex position updates
            var sumN = new int[vertices.Length];     // sum of vertex count updates
            
            // Apply PBD here,Set both arrays to zeros
            for (var i = 0; i < vertices.Length; i++)
            {
                sumX[i] = new Vector3(0, 0, 0);
                sumN[i] = 0;
            }

            // for every edge e connecting i and j ,update the arrays
            for (var e = 0; e < L.Length; e++)
            {
                var i = E[e * 2];
                var j = E[e * 2 + 1];
                var xij = vertices[i] - vertices[j];
                sumX[i] += 0.5f * (vertices[i] + vertices[j] + xij * (L[e] * (1.0f / xij.magnitude)));
                sumX[j] += 0.5f * (vertices[i] + vertices[j] - xij * (L[e] * (1.0f / xij.magnitude)));
                sumN[i]++;
                sumN[j]++;
            }

            // update each vertex velocity
            for (var i = 0; i < vertices.Length; i++)
            {
                if (i is 0 or 20) continue;
                V[i] += (1.0f / T) * ((0.2f * vertices[i] + sumX[i]) / (0.2f + sumN[i]) - vertices[i]);
                vertices[i] = (0.2f * vertices[i] + sumX[i]) / (0.2f + sumN[i]);
            }

            mesh.vertices = vertices;
        }

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

        #endregion
    }
}
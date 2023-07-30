using System;
using System.IO;
using UnityEngine;

namespace FiniteElementMethod
{
    public class Fvm : MonoBehaviour
    {
        #region Fields

        private const float DT = 0.003f;
        private const float Mass = 1;
        private const float Stiffness0 = 20000.0f;
        private const float Stiffness1 = 5000.0f;
        private const float Damp = 0.999f;

        private int[] _tet;
        private int _tetNumber; //The number of tetrahedron
        private Vector3[] _force;

        // ReSharper disable once InconsistentNaming
        private Vector3[] V;

        // ReSharper disable once InconsistentNaming
        private Vector3[] X;
        private int _number; //The number of vertices
        private Matrix4x4[] _invDm;

        //For Laplacian smoothing.
        private Vector3[] _vSum;
        private int[] _vNum;

        #endregion

        #region Unity Methods

        private void Start()
        {
            {
                var fileContent = File.ReadAllText("Assets/Resources/house2.ele");
                var strings = fileContent.Split(new[] { ' ', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);

                _tetNumber = int.Parse(strings[0]);
                _tet = new int[_tetNumber * 4];

                for (var tet = 0; tet < _tetNumber; tet++)
                {
                    _tet[tet * 4 + 0] = int.Parse(strings[tet * 5 + 4]) - 1;
                    _tet[tet * 4 + 1] = int.Parse(strings[tet * 5 + 5]) - 1;
                    _tet[tet * 4 + 2] = int.Parse(strings[tet * 5 + 6]) - 1;
                    _tet[tet * 4 + 3] = int.Parse(strings[tet * 5 + 7]) - 1;
                }
            }
            {
                var fileContent = File.ReadAllText("Assets/Resources/house2.node");
                var strings = fileContent.Split(new[] { ' ', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);
                _number = int.Parse(strings[0]);
                X = new Vector3[_number];
                for (var i = 0; i < _number; i++)
                {
                    X[i].x = float.Parse(strings[i * 5 + 5]) * 0.4f;
                    X[i].y = float.Parse(strings[i * 5 + 6]) * 0.4f;
                    X[i].z = float.Parse(strings[i * 5 + 7]) * 0.4f;
                }

                //Centralize the model.
                var center = Vector3.zero;
                for (var i = 0; i < _number; i++) center += X[i];
                center /= _number;
                for (var i = 0; i < _number; i++)
                {
                    X[i] -= center;
                    (X[i].y, X[i].z) = (X[i].z, X[i].y);
                }
            }
            
            //Create triangle mesh.
            var vertices = new Vector3[_tetNumber * 12];
            var vertexNumber = 0;
            for (var tet = 0; tet < _tetNumber; tet++)
            {
                vertices[vertexNumber++] = X[_tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 1]];

                vertices[vertexNumber++] = X[_tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 3]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 2]];

                vertices[vertexNumber++] = X[_tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 3]];

                vertices[vertexNumber++] = X[_tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 3]];
            }

            var triangles = new int[_tetNumber * 12];
            for (var t = 0; t < _tetNumber * 4; t++)
            {
                triangles[t * 3 + 0] = t * 3 + 0;
                triangles[t * 3 + 1] = t * 3 + 1;
                triangles[t * 3 + 2] = t * 3 + 2;
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();


            V = new Vector3[_number];
            _force = new Vector3[_number];
            _vSum = new Vector3[_number];
            _vNum = new int[_number];

            //TODO: Need to allocate and assign inv_Dm
            _invDm = new Matrix4x4[_tetNumber]; //初始的位置矩阵
            for (var tet = 0; tet < _tetNumber; tet++)
                _invDm[tet] = BuildEdgeMatrix(tet).inverse;
        }

        private void Update()
        {
            for (var l = 0; l < 10; l++)
                _Update();

            // Dump the vertex array for rendering.
            var vertices = new Vector3[_tetNumber * 12];
            var vertexNumber = 0;
            for (var tet = 0; tet < _tetNumber; tet++)
            {
                vertices[vertexNumber++] = X[_tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 3]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 3]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[_tet[tet * 4 + 3]];
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        private Matrix4x4 BuildEdgeMatrix(int tet)
        {
            var ret = Matrix4x4.zero;
            //TODO: Need to build edge matrix here.
            ret[0, 0] = X[_tet[tet * 4 + 1]].x - X[_tet[tet * 4 + 0]].x;
            ret[1, 0] = X[_tet[tet * 4 + 1]].y - X[_tet[tet * 4 + 0]].y;
            ret[2, 0] = X[_tet[tet * 4 + 1]].z - X[_tet[tet * 4 + 0]].z;
            ret[3, 0] = 0;

            ret[0, 1] = X[_tet[tet * 4 + 2]].x - X[_tet[tet * 4 + 0]].x;
            ret[1, 1] = X[_tet[tet * 4 + 2]].y - X[_tet[tet * 4 + 0]].y;
            ret[2, 1] = X[_tet[tet * 4 + 2]].z - X[_tet[tet * 4 + 0]].z;
            ret[3, 1] = 0;

            ret[0, 2] = X[_tet[tet * 4 + 3]].x - X[_tet[tet * 4 + 0]].x;
            ret[1, 2] = X[_tet[tet * 4 + 3]].y - X[_tet[tet * 4 + 0]].y;
            ret[2, 2] = X[_tet[tet * 4 + 3]].z - X[_tet[tet * 4 + 0]].z;
            ret[3, 2] = 0;

            ret[0, 3] = 0;
            ret[1, 3] = 0;
            ret[2, 3] = 0;
            ret[3, 3] = 1;

            return ret;
        }

        private void SmoothV()
        {
            for (var i = 0; i < _number; i++)
            {
                _vSum[i] = new Vector3(0, 0, 0);
                _vNum[i] = 0;
            }

            for (var tet = 0; tet < _tetNumber; tet++)
            {
                var sum = V[_tet[tet * 4 + 0]] + V[_tet[tet * 4 + 1]] + V[_tet[tet * 4 + 2]] +
                          V[_tet[tet * 4 + 3]];
                _vSum[_tet[tet * 4 + 0]] += sum;
                _vSum[_tet[tet * 4 + 1]] += sum;
                _vSum[_tet[tet * 4 + 2]] += sum;
                _vSum[_tet[tet * 4 + 3]] += sum;
                _vNum[_tet[tet * 4 + 0]] += 4;
                _vNum[_tet[tet * 4 + 1]] += 4;
                _vNum[_tet[tet * 4 + 2]] += 4;
                _vNum[_tet[tet * 4 + 3]] += 4;
            }

            for (var i = 0; i < _number; i++)
            {
                V[i] = 0.9f * V[i] + 0.1f * _vSum[i] / _vNum[i];
            }
        }

        private void _Update()
        {
            // Jump up.
            if (Input.GetKeyDown(KeyCode.Space))
            {
                for (var i = 0; i < _number; i++)
                    V[i].y += 0.2f;
            }

            for (var i = 0; i < _number; i++)
            {
                // Add gravity to Force.
                _force[i] = new Vector3(0, -9.8f * Mass, 0);
            }

            for (var tet = 0; tet < _tetNumber; tet++)
            {
                //TODO: Deformation Gradient
                // ReSharper disable once InconsistentNaming
                var F = BuildEdgeMatrix(tet) * _invDm[tet];
                //TODO: Green Strain
                // ReSharper disable once InconsistentNaming
                var G = F.transpose * F;
                for (var i = 0; i < 4; i++)
                for (var j = 0; j < 4; j++)
                {
                    if (i == j)
                        G[i, j] -= 1;
                    G[i, j] = 0.5f * G[i, j];
                }

                //TODO: Second PK Stress
                // ReSharper disable once InconsistentNaming
                var S = Matrix4x4.zero;
                var traceG = G[0, 0] + G[1, 1] + G[2, 2];
                for (var i = 0; i < 4; i++)
                for (var j = 0; j < 4; j++)
                {
                    S[i, j] = 2 * Stiffness1 * G[i, j];
                    if (i == j)
                        S[i, j] += Stiffness0 * traceG;
                }

                //TODO: Elastic Force
                var force = F * S * _invDm[tet].transpose;
                var volume = 1 / (_invDm[tet].determinant * 6); //注意下这里体积公式 S=||A||/6  S=1/(6*||A^-1||)
                for (var i = 0; i < 4; i++)
                for (var j = 0; j < 4; j++)
                    force[i, j] = -1 * volume * force[i, j];
                _force[_tet[tet * 4 + 1]].x += force[0, 0];
                _force[_tet[tet * 4 + 1]].y += force[1, 0];
                _force[_tet[tet * 4 + 1]].z += force[2, 0];
                _force[_tet[tet * 4 + 2]].x += force[0, 1];
                _force[_tet[tet * 4 + 2]].y += force[1, 1];
                _force[_tet[tet * 4 + 2]].z += force[2, 1];
                _force[_tet[tet * 4 + 3]].x += force[0, 2];
                _force[_tet[tet * 4 + 3]].y += force[1, 2];
                _force[_tet[tet * 4 + 3]].z += force[2, 2];

                _force[_tet[tet * 4 + 0]].x -= (force[0, 0] + force[0, 1] + force[0, 2]);
                _force[_tet[tet * 4 + 0]].y -= (force[1, 0] + force[1, 1] + force[1, 2]);
                _force[_tet[tet * 4 + 0]].z -= (force[2, 0] + force[2, 1] + force[2, 2]);
            }


            SmoothV();

            for (var i = 0; i < _number; i++)
            {
                //TODO: Update X and V here.
                V[i] = (V[i] + DT * _force[i] / Mass) * Damp;
                X[i] += V[i] * DT;
                //TODO: (Particle) collision with floor.
                if (X[i].y < -3f) //这里仅对局部坐标进行相应判断，选取-3的原因是底部物体的世界坐标是-3
                {
                    X[i].y = -3f;
                    if (V[i].y < 0)
                        V[i].y = -V[i].y;
                }
            }
        }

        #endregion
    }
}
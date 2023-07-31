using System;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;

namespace FiniteElementMethod
{
    public class ParallelFvm : MonoBehaviour
    {
        #region Fields

        private const float DT = 0.002f;
        private const float Mass = 1;
        private const float Stiffness0 = 20000.0f;
        private const float Stiffness1 = 5000.0f;
        private const float Damp = 0.999f;
        private const float G = -9.8f;

        // ReSharper disable once InconsistentNaming
        private int[] Tet;
        private int _tetNumber; //The number of tetrahedron

        // ReSharper disable once InconsistentNaming
        private Vector3[] Force;

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
                Tet = new int[_tetNumber * 4];

                for (var tet = 0; tet < _tetNumber; tet++)
                {
                    Tet[tet * 4 + 0] = int.Parse(strings[tet * 5 + 4]) - 1;
                    Tet[tet * 4 + 1] = int.Parse(strings[tet * 5 + 5]) - 1;
                    Tet[tet * 4 + 2] = int.Parse(strings[tet * 5 + 6]) - 1;
                    Tet[tet * 4 + 3] = int.Parse(strings[tet * 5 + 7]) - 1;
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
                for (int i = 0; i < _number; i++) center += X[i];
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
                vertices[vertexNumber++] = X[Tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 1]];

                vertices[vertexNumber++] = X[Tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 3]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 2]];

                vertices[vertexNumber++] = X[Tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 3]];

                vertices[vertexNumber++] = X[Tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 3]];
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
            Force = new Vector3[_number];
            _vSum = new Vector3[_number];
            _vNum = new int[_number];

            //TODO: Need to allocate and assign inv_Dm
            _invDm = new Matrix4x4[_tetNumber];
            for (var tet = 0; tet < _tetNumber; tet++)
            {
                var dm = BuildEdgeMatrix(tet);

                _invDm[tet] = dm.inverse;
            }
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
                vertices[vertexNumber++] = X[Tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 3]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 0]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 3]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 1]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 2]];
                vertices[vertexNumber++] = X[Tet[tet * 4 + 3]];
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        private void _Update()
        {
            // Jump up.
            if (Input.GetKeyDown(KeyCode.Space))
            {
                for (var i = 0; i < _number; i++)
                    V[i].y += 0.5f;
            }

            Parallel.For(0, _number, i =>
            {
                // Add gravity to Force.
                Force[i] = Vector3.zero;
                Force[i].y += Mass * G;
            });

            Parallel.For(0, _tetNumber, tet =>
            {
                //TODO: Deformation Gradient
                var dm = BuildEdgeMatrix(tet);

                // ReSharper disable once InconsistentNaming
                var F = dm * _invDm[tet];
                F[3, 3] = 1;

                //TODO: Green Strain
                var I = Matrix4x4.identity;
                // ReSharper disable once LocalVariableHidesMember
                // ReSharper disable once InconsistentNaming
                var G = MatrixSub(F.transpose * F, I);
                G[3, 3] = 1;
                G = MatrixMulScaler(G, 0.5f);

                //TODO: First PK Stress
                //Second PK Stress: S
                // ReSharper disable once RedundantAssignment
                // ReSharper disable once InconsistentNaming
                var S = new Matrix4x4();
                S = MatrixAdd(MatrixMulScaler(G, 2 * Stiffness1), MatrixMulScaler(I, Stiffness0 * MatrixTrace(G)));
                S[3, 3] = 1;
                //First PK Stress: p
                // ReSharper disable once InconsistentNaming
                // ReSharper disable once RedundantAssignment
                var P = new Matrix4x4();
                P = F * S;
                P[3, 3] = 1;

                //TODO: Elastic Force
                var tmp = -1 / (6 * _invDm[tet].determinant);
                var fs = MatrixMulScaler(P * _invDm[tet].transpose, tmp);
                Vector3 f1 = fs.GetColumn(0);
                Vector3 f2 = fs.GetColumn(1);
                Vector3 f3 = fs.GetColumn(2);
                var f0 = -f1 - f2 - f3;

                Force[Tet[tet * 4 + 0]] += f0;
                Force[Tet[tet * 4 + 1]] += f1;
                Force[Tet[tet * 4 + 2]] += f2;
                Force[Tet[tet * 4 + 3]] += f3;
            });

            // Update V[], damping V[]
            Parallel.For(0, _number, i =>
            {
                V[i] += DT * Force[i];
                V[i] *= Damp;
            });

            // Laplacian Smoothing
            Parallel.For(0, _number, i =>
            {
                _vSum[i] = Vector3.zero;
                _vNum[i] = 0;
            });
            Parallel.For(0, _tetNumber, tet =>
            {
                var tetVSum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] +
                              V[Tet[tet * 4 + 3]];

                _vSum[Tet[tet * 4 + 0]] += tetVSum;
                _vNum[Tet[tet * 4 + 0]] += 4;
                _vSum[Tet[tet * 4 + 1]] += tetVSum;
                _vNum[Tet[tet * 4 + 1]] += 4;
                _vSum[Tet[tet * 4 + 2]] += tetVSum;
                _vNum[Tet[tet * 4 + 2]] += 4;
                _vSum[Tet[tet * 4 + 3]] += tetVSum;
                _vNum[Tet[tet * 4 + 3]] += 4;
            });
            Parallel.For(0, _number, i => { V[i] = _vSum[i] / _vNum[i]; });

            Parallel.For(0, _number, i =>
            {
                X[i] += DT * V[i];

                if (V[i].magnitude < 0.001f)
                {
                    V[i] = Vector3.zero;
                }

                //TODO: (Particle) collision with floor.
                const float floor = -3.0f;
                // ReSharper disable once InconsistentNaming
                var N = new Vector3(0, 1, 0);
                if (X[i].y < floor)
                {
                    X[i].y = floor;
                    if (Vector3.Dot(V[i], N) < 0)
                    {
                        var vn = Vector3.Dot(V[i], N) * N;
                        var vt = V[i] - vn;

                        const float muN = 0.5f;
                        const float a = 0.5f;
                        vn *= -muN;
                        vt *= a;
                        V[i] = vn + vt;
                    }
                }
            });
        }

        private Matrix4x4 BuildEdgeMatrix(int tet)
        {
            var ret = Matrix4x4.zero;
            //TODO: Need to build edge matrix here.
            var x10 = X[Tet[tet * 4 + 1]] - X[Tet[tet * 4 + 0]];
            var x20 = X[Tet[tet * 4 + 2]] - X[Tet[tet * 4 + 0]];
            var x30 = X[Tet[tet * 4 + 3]] - X[Tet[tet * 4 + 0]];
            ret.SetColumn(0, x10);
            ret.SetColumn(1, x20);
            ret.SetColumn(2, x30);
            ret[3, 3] = 1;

            return ret;
        }

        #endregion


        private static Matrix4x4 MatrixSub(Matrix4x4 a, Matrix4x4 b)
        {
            var m = Matrix4x4.identity;
            m[0, 0] = a[0, 0] - b[0, 0];
            m[0, 1] = a[0, 1] - b[0, 1];
            m[0, 2] = a[0, 2] - b[0, 2];
            m[1, 0] = a[1, 0] - b[1, 0];
            m[1, 1] = a[1, 1] - b[1, 1];
            m[1, 2] = a[1, 2] - b[0, 2];
            m[2, 0] = a[2, 0] - b[2, 0];
            m[2, 1] = a[2, 1] - b[2, 1];
            m[2, 2] = a[2, 2] - b[2, 2];
            return m;
        }

        private static Matrix4x4 MatrixAdd(Matrix4x4 a, Matrix4x4 b)
        {
            var m = Matrix4x4.identity;
            m[0, 0] = a[0, 0] + b[0, 0];
            m[0, 1] = a[0, 1] + b[0, 1];
            m[0, 2] = a[0, 2] + b[0, 2];
            m[1, 0] = a[1, 0] + b[1, 0];
            m[1, 1] = a[1, 1] + b[1, 1];
            m[1, 2] = a[1, 2] + b[0, 2];
            m[2, 0] = a[2, 0] + b[2, 0];
            m[2, 1] = a[2, 1] + b[2, 1];
            m[2, 2] = a[2, 2] + b[2, 2];
            return m;
        }

        private static Matrix4x4 MatrixMulScaler(Matrix4x4 a, float s)
        {
            var m = new Matrix4x4
            {
                [0, 0] = a[0, 0] * s,
                [0, 1] = a[0, 1] * s,
                [0, 2] = a[0, 2] * s,
                [1, 0] = a[1, 0] * s,
                [1, 1] = a[1, 1] * s,
                [1, 2] = a[1, 2] * s,
                [2, 0] = a[2, 0] * s,
                [2, 1] = a[2, 1] * s,
                [2, 2] = a[2, 2] * s
            };
            return m;
        }

        private static float MatrixTrace(Matrix4x4 m)
        {
            var tr = m[0, 0] + m[1, 1] + m[2, 2];
            return tr;
        }
    }
}
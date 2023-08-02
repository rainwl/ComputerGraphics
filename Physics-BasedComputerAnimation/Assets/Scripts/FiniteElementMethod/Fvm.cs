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

        private int[] _tetrahedron;
        private int _tetrahedronNumber;
        private Vector3[] _force;
        private Vector3[] _velocity;
        private Vector3[] _position;

        private int _vertexNum;
        private Matrix4x4[] _invDm;

        //For Laplacian smoothing.
        private Vector3[] _neighborhoodVelocitySum;
        private int[] _neighborhoodNum;

        #endregion

        #region Unity Methods

        private void Start()
        {
            {
                var fileContent = File.ReadAllText(Application.streamingAssetsPath+"/house2.ele");
                var strings = fileContent.Split(new[] { ' ', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);

                _tetrahedronNumber = int.Parse(strings[0]);
                _tetrahedron = new int[_tetrahedronNumber * 4];

                for (var tetrahedron = 0; tetrahedron < _tetrahedronNumber; tetrahedron++)
                {
                    _tetrahedron[tetrahedron * 4 + 0] = int.Parse(strings[tetrahedron * 5 + 4]) - 1;
                    _tetrahedron[tetrahedron * 4 + 1] = int.Parse(strings[tetrahedron * 5 + 5]) - 1;
                    _tetrahedron[tetrahedron * 4 + 2] = int.Parse(strings[tetrahedron * 5 + 6]) - 1;
                    _tetrahedron[tetrahedron * 4 + 3] = int.Parse(strings[tetrahedron * 5 + 7]) - 1;
                }
            }
            {
                var fileContent = File.ReadAllText(Application.streamingAssetsPath+"/house2.node");
                var strings = fileContent.Split(new[] { ' ', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);
                _vertexNum = int.Parse(strings[0]);
                _position = new Vector3[_vertexNum];
                for (var i = 0; i < _vertexNum; i++)
                {
                    _position[i].x = float.Parse(strings[i * 5 + 5]) * 0.4f;
                    _position[i].y = float.Parse(strings[i * 5 + 6]) * 0.4f;
                    _position[i].z = float.Parse(strings[i * 5 + 7]) * 0.4f;
                }

                //Centralize the model.
                var center = Vector3.zero;
                for (var i = 0; i < _vertexNum; i++) center += _position[i];
                center /= _vertexNum;
                for (var i = 0; i < _vertexNum; i++)
                {
                    _position[i] -= center;
                    (_position[i].y, _position[i].z) = (_position[i].z, _position[i].y);
                }
            }

            //Create triangle mesh.
            var vertices = new Vector3[_tetrahedronNumber * 12];
            var vertexNumber = 0;
            for (var tet = 0; tet < _tetrahedronNumber; tet++)
            {
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 1]];

                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 2]];

                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 3]];

                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 3]];
            }

            var triangles = new int[_tetrahedronNumber * 12];
            for (var t = 0; t < _tetrahedronNumber * 4; t++)
            {
                triangles[t * 3 + 0] = t * 3 + 0;
                triangles[t * 3 + 1] = t * 3 + 1;
                triangles[t * 3 + 2] = t * 3 + 2;
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();


            _velocity = new Vector3[_vertexNum];
            _force = new Vector3[_vertexNum];
            _neighborhoodVelocitySum = new Vector3[_vertexNum];
            _neighborhoodNum = new int[_vertexNum];

            //TODO: Need to allocate and assign inv_Dm
            _invDm = new Matrix4x4[_tetrahedronNumber];
            for (var tet = 0; tet < _tetrahedronNumber; tet++)
                _invDm[tet] = BuildEdgeMatrix(tet).inverse;
        }

        private void Update()
        {
            // Since this project uses a relatively small time step,
            // the Update function calls Update ten times.
            for (var l = 0; l < 10; l++)
                _Update();

            // Dump the vertex array for rendering.
            var vertices = new Vector3[_tetrahedronNumber * 12];
            var vertexNumber = 0;
            for (var tet = 0; tet < _tetrahedronNumber; tet++)
            {
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tet * 4 + 3]];
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// return the edge matrix of a tetrahedron
        /// </summary>
        /// <param name="tet"></param>
        /// <returns></returns>
        private Matrix4x4 BuildEdgeMatrix(int tet)
        {
            var ret = Matrix4x4.zero;
            // Need to build edge matrix here.
            ret[0, 0] = _position[_tetrahedron[tet * 4 + 1]].x - _position[_tetrahedron[tet * 4 + 0]].x;
            ret[1, 0] = _position[_tetrahedron[tet * 4 + 1]].y - _position[_tetrahedron[tet * 4 + 0]].y;
            ret[2, 0] = _position[_tetrahedron[tet * 4 + 1]].z - _position[_tetrahedron[tet * 4 + 0]].z;
            ret[3, 0] = 0;

            ret[0, 1] = _position[_tetrahedron[tet * 4 + 2]].x - _position[_tetrahedron[tet * 4 + 0]].x;
            ret[1, 1] = _position[_tetrahedron[tet * 4 + 2]].y - _position[_tetrahedron[tet * 4 + 0]].y;
            ret[2, 1] = _position[_tetrahedron[tet * 4 + 2]].z - _position[_tetrahedron[tet * 4 + 0]].z;
            ret[3, 1] = 0;

            ret[0, 2] = _position[_tetrahedron[tet * 4 + 3]].x - _position[_tetrahedron[tet * 4 + 0]].x;
            ret[1, 2] = _position[_tetrahedron[tet * 4 + 3]].y - _position[_tetrahedron[tet * 4 + 0]].y;
            ret[2, 2] = _position[_tetrahedron[tet * 4 + 3]].z - _position[_tetrahedron[tet * 4 + 0]].z;
            ret[3, 2] = 0;

            ret[0, 3] = 0;
            ret[1, 3] = 0;
            ret[2, 3] = 0;
            ret[3, 3] = 1;

            return ret;
        }

        /// <summary>
        /// Laplacian smoothing
        /// </summary>
        private void SmoothV()
        {
            // neighborhood Velocity Sum and Num
            for (var i = 0; i < _vertexNum; i++)
            {
                _neighborhoodVelocitySum[i] = new Vector3(0, 0, 0);
                _neighborhoodNum[i] = 0;
            }

            for (var tet = 0; tet < _tetrahedronNumber; tet++)
            {
                var sum = _velocity[_tetrahedron[tet * 4 + 0]] + _velocity[_tetrahedron[tet * 4 + 1]] +
                          _velocity[_tetrahedron[tet * 4 + 2]] +
                          _velocity[_tetrahedron[tet * 4 + 3]];
                _neighborhoodVelocitySum[_tetrahedron[tet * 4 + 0]] += sum;
                _neighborhoodVelocitySum[_tetrahedron[tet * 4 + 1]] += sum;
                _neighborhoodVelocitySum[_tetrahedron[tet * 4 + 2]] += sum;
                _neighborhoodVelocitySum[_tetrahedron[tet * 4 + 3]] += sum;
                _neighborhoodNum[_tetrahedron[tet * 4 + 0]] += 4;
                _neighborhoodNum[_tetrahedron[tet * 4 + 1]] += 4;
                _neighborhoodNum[_tetrahedron[tet * 4 + 2]] += 4;
                _neighborhoodNum[_tetrahedron[tet * 4 + 3]] += 4;
            }

            for (var i = 0; i < _vertexNum; i++)
            {
                _velocity[i] = 0.9f * _velocity[i] + 0.1f * _neighborhoodVelocitySum[i] / _neighborhoodNum[i];
            }
        }

        private void _Update()
        {
            // Jump up.
            if (Input.GetKeyDown(KeyCode.Space))
            {
                for (var i = 0; i < _vertexNum; i++)
                    _velocity[i].y += 0.2f;
            }

            // Add gravity to Force
            for (var i = 0; i < _vertexNum; i++)
            {
                _force[i] = new Vector3(0, -9.8f * Mass, 0);
            }

            // F G S forces
            for (var tet = 0; tet < _tetrahedronNumber; tet++)
            {
                #region Deformation Gradient

                // ReSharper disable once InconsistentNaming
                var F = BuildEdgeMatrix(tet) * _invDm[tet];

                #endregion
                
                #region Green Strain

                // ReSharper disable once InconsistentNaming
                var G = F.transpose * F;
                for (var i = 0; i < 4; i++)
                {
                    for (var j = 0; j < 4; j++)
                    {
                        if (i == j)
                            G[i, j] -= 1;
                        G[i, j] = 0.5f * G[i, j];
                    }
                }

                #endregion

                #region Second PK Stress

                // ReSharper disable once InconsistentNaming
                var S = Matrix4x4.zero;
                var traceG = G[0, 0] + G[1, 1] + G[2, 2];
                for (var i = 0; i < 4; i++)
                {
                    for (var j = 0; j < 4; j++)
                    {
                        S[i, j] = 2 * Stiffness1 * G[i, j];
                        if (i == j)
                            S[i, j] += Stiffness0 * traceG;
                    }
                }
                

                #endregion

                #region Elastic Force

                var force = F * S * _invDm[tet].transpose;
                var volume = 1 / (_invDm[tet].determinant * 6); //注意下这里体积公式 S=||A||/6  S=1/(6*||A^-1||)
                for (var i = 0; i < 4; i++)
                for (var j = 0; j < 4; j++)
                    force[i, j] = -1 * volume * force[i, j];
                _force[_tetrahedron[tet * 4 + 1]].x += force[0, 0];
                _force[_tetrahedron[tet * 4 + 1]].y += force[1, 0];
                _force[_tetrahedron[tet * 4 + 1]].z += force[2, 0];
                _force[_tetrahedron[tet * 4 + 2]].x += force[0, 1];
                _force[_tetrahedron[tet * 4 + 2]].y += force[1, 1];
                _force[_tetrahedron[tet * 4 + 2]].z += force[2, 1];
                _force[_tetrahedron[tet * 4 + 3]].x += force[0, 2];
                _force[_tetrahedron[tet * 4 + 3]].y += force[1, 2];
                _force[_tetrahedron[tet * 4 + 3]].z += force[2, 2];

                _force[_tetrahedron[tet * 4 + 0]].x -= (force[0, 0] + force[0, 1] + force[0, 2]);
                _force[_tetrahedron[tet * 4 + 0]].y -= (force[1, 0] + force[1, 1] + force[1, 2]);
                _force[_tetrahedron[tet * 4 + 0]].z -= (force[2, 0] + force[2, 1] + force[2, 2]);

                #endregion
                
            }


            // Laplacian smoothing
            SmoothV();

            // update x and v
            for (var i = 0; i < _vertexNum; i++)
            {
                // frictional contact
                _velocity[i] = (_velocity[i] + DT * _force[i] / Mass) * Damp;
                _position[i] += _velocity[i] * DT;
                // (Particle) collision with floor.
                // Here, only the local coordinates are judged accordingly,
                // and the reason for choosing -3 is that the world coordinate of the bottom object is -3
                // ReSharper disable once InvertIf
                if (_position[i].y < -3f)
                {
                    _position[i].y = -3f;
                    if (_velocity[i].y < 0)
                        _velocity[i].y = -_velocity[i].y;
                }
            }
        }

        #endregion
    }
}
using System;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;

namespace FiniteElementMethod
{
    public class FvmColor : MonoBehaviour
    {
        #region Fields

        private const float DT = 0.002f;
        private const float Mass = 1;
        private const float Stiffness0 = 20000.0f;
        private const float Stiffness1 = 5000.0f;
        private const float Damp = 0.999f;

        private int[] _tetrahedron;
        private int _tetrahedronNum;
        private Vector3[] _force;
        private Vector3[] _velocity;
        private Vector3[] _position;

        private int _vertexNum;
        private Matrix4x4[] _invDm;

        //For Laplacian smoothing.
        private Vector3[] _neighborhoodVelocitySum;
        private int[] _neighborhoodNum;

        [Range(0, 0.01f)] public float velocitySmoothWeight = 0.005f;
        public bool useHyperElasticModels;
        private const float CollisionRestitutionN = 0.5f;
        private const float CollisionRestitutionT = 0.5f;
        private Color[] _vertexColor;
        private readonly SVD _svd = new();

        #endregion

        #region Unity Methods

        private void Start()
        {
            {
                //var fileContent = File.ReadAllText("Assets/Resources/house2.ele");
                var fileContent = File.ReadAllText(Application.streamingAssetsPath+"/house2.ele");
                var strings = fileContent.Split(new[] { ' ', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);

                _tetrahedronNum = int.Parse(strings[0]);
                _tetrahedron = new int[_tetrahedronNum * 4];

                for (var tetrahedron = 0; tetrahedron < _tetrahedronNum; tetrahedron++)
                {
                    _tetrahedron[tetrahedron * 4 + 0] = int.Parse(strings[tetrahedron * 5 + 4]) - 1;
                    _tetrahedron[tetrahedron * 4 + 1] = int.Parse(strings[tetrahedron * 5 + 5]) - 1;
                    _tetrahedron[tetrahedron * 4 + 2] = int.Parse(strings[tetrahedron * 5 + 6]) - 1;
                    _tetrahedron[tetrahedron * 4 + 3] = int.Parse(strings[tetrahedron * 5 + 7]) - 1;
                }
            }
            {
                //var fileContent = File.ReadAllText("Assets/Resources/house2.node");
                var fileContent = File.ReadAllText(Application.streamingAssetsPath+"/house2.node");
                var strings = fileContent.Split(new[] { ' ', '\t', '\r', '\n' },
                    StringSplitOptions.RemoveEmptyEntries);
                _vertexNum = int.Parse(strings[0]);
                _position = new Vector3[_vertexNum];

                _vertexColor = new Color[_vertexNum];
                _neighborhoodVelocitySum = new Vector3[_vertexNum];
                _neighborhoodNum = new int[_vertexNum];

                for (var i = 0; i < _vertexNum; i++)
                {
                    _position[i].x = float.Parse(strings[i * 5 + 5]) * 0.4f;
                    _position[i].y = float.Parse(strings[i * 5 + 6]) * 0.4f;
                    _position[i].z = float.Parse(strings[i * 5 + 7]) * 0.4f;

                    _vertexColor[i] = new Color(0, 0, 0, 1);
                    _neighborhoodVelocitySum[i] = new Vector3(0, 0, 0);
                    _neighborhoodNum[i] = 0;
                }

                //Centralize the model.
                var center = Vector3.zero;
                for (var i = 0; i < _vertexNum; i++)
                    center += _position[i];
                center /= _vertexNum;
                for (var i = 0; i < _vertexNum; i++)
                {
                    _position[i] -= center;
                    (_position[i].y, _position[i].z) = (_position[i].z, _position[i].y);
                }
            }

            //Create triangle mesh.
            var vertices = new Vector3[_tetrahedronNum * 12];
            var vertexNumber = 0;
            for (var tetrahedron = 0; tetrahedron < _tetrahedronNum; tetrahedron++)
            {
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 1]];

                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 2]];

                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 3]];

                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 3]];
            }

            var triangles = new int[_tetrahedronNum * 12];
            for (var t = 0; t < _tetrahedronNum * 4; t++)
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

            _invDm = new Matrix4x4[_tetrahedronNum];
            for (var t = 0; t < _tetrahedronNum; t++)
            {
                var edgeMatrix = BuildEdgeMatrix(t);
                _invDm[t] = edgeMatrix.inverse;
            }
        }

        private void Update()
        {
            for (var l = 0; l < 10; l++)
                _Update();

            // Dump the vertex array for rendering.
            var vertices = new Vector3[_tetrahedronNum * 12];

            var colors = new Color[_tetrahedronNum * 12];

            var vertexNumber = 0;
            for (var tetrahedron = 0; tetrahedron < _tetrahedronNum; tetrahedron++)
            {
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 0]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 2]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 1]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 0]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 3]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 2]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 0]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 0]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 1]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 3]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 1]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 1]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 2]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 2]];
                colors[vertexNumber] = _vertexColor[_tetrahedron[tetrahedron * 4 + 3]];
                vertices[vertexNumber++] = _position[_tetrahedron[tetrahedron * 4 + 3]];
            }

            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.colors = colors;
            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        private Matrix4x4 BuildEdgeMatrix(int tetrahedron)
        {
            Vector4 x10 = _position[_tetrahedron[tetrahedron * 4 + 1]] - _position[_tetrahedron[tetrahedron * 4 + 0]];
            Vector4 x20 = _position[_tetrahedron[tetrahedron * 4 + 2]] - _position[_tetrahedron[tetrahedron * 4 + 0]];
            Vector4 x30 = _position[_tetrahedron[tetrahedron * 4 + 3]] - _position[_tetrahedron[tetrahedron * 4 + 0]];
            var edgeMatrix = new Matrix4x4();
            edgeMatrix.SetColumn(0, x10);
            edgeMatrix.SetColumn(1, x20);
            edgeMatrix.SetColumn(2, x30);
            edgeMatrix.SetColumn(3, new Vector4(0, 0, 0, 1));

            return edgeMatrix;
        }

        private void _Update()
        {
            if (Input.GetKeyDown(KeyCode.Space))
            {
                for (var i = 0; i < _vertexNum; i++)
                    _velocity[i].y += 0.5f;
            }
            
            Parallel.For(0, _vertexNum, i =>
            {
                _force[i] = new Vector3(0, -9.8f * Mass, 0);
                _vertexColor[i] = new Color(0, 1, 0, 1);
            });
            
            Parallel.For(0, _tetrahedronNum, tetrahedron =>
            {
                var deformationGradient = BuildEdgeMatrix(tetrahedron) * _invDm[tetrahedron];

                var firstPkStress = useHyperElasticModels ? _Method2(deformationGradient) : _Method1(deformationGradient);

                var force = MatrixMulFloat(firstPkStress, -1.0f / (6.0f * _invDm[tetrahedron].determinant));
                force = force * _invDm[tetrahedron].transpose;

                Vector3 force1 = force.GetColumn(0);
                Vector3 force2 = force.GetColumn(1);
                Vector3 force3 = force.GetColumn(2);
                var force0 = -force1 - force2 - force3;
                var vertexIndex0 = _tetrahedron[tetrahedron * 4 + 0];
                var vertexIndex1 = _tetrahedron[tetrahedron * 4 + 1];
                var vertexIndex2 = _tetrahedron[tetrahedron * 4 + 2];
                var vertexIndex3 = _tetrahedron[tetrahedron * 4 + 3];
                _force[vertexIndex0] += force0;
                _force[vertexIndex1] += force1;
                _force[vertexIndex2] += force2;
                _force[vertexIndex3] += force3;

                _vertexColor[vertexIndex0].r += force0.magnitude * 0.005f;
                _vertexColor[vertexIndex1].r += force1.magnitude * 0.005f;
                _vertexColor[vertexIndex2].r += force2.magnitude * 0.005f;
                _vertexColor[vertexIndex3].r += force3.magnitude * 0.005f;
            });



            Parallel.For(0, _vertexNum, i =>
            {
                _velocity[i] += _force[i] / Mass * DT;
                _velocity[i] *= Damp;
            });
            

            Parallel.For(0, _tetrahedronNum, tetrahedron =>
            {
                var vertexIndex0 = _tetrahedron[tetrahedron * 4 + 0];
                var vertexIndex1 = _tetrahedron[tetrahedron * 4 + 1];
                var vertexIndex2 = _tetrahedron[tetrahedron * 4 + 2];
                var vertexIndex3 = _tetrahedron[tetrahedron * 4 + 3];
                _neighborhoodVelocitySum[vertexIndex0] = _neighborhoodVelocitySum[vertexIndex0] +
                                                         _velocity[vertexIndex1] + _velocity[vertexIndex2] +
                                                         _velocity[vertexIndex3];
                _neighborhoodVelocitySum[vertexIndex1] = _neighborhoodVelocitySum[vertexIndex1] +
                                                         _velocity[vertexIndex0] + _velocity[vertexIndex2] +
                                                         _velocity[vertexIndex3];
                _neighborhoodVelocitySum[vertexIndex2] = _neighborhoodVelocitySum[vertexIndex2] +
                                                         _velocity[vertexIndex0] + _velocity[vertexIndex1] +
                                                         _velocity[vertexIndex3];
                _neighborhoodVelocitySum[vertexIndex3] = _neighborhoodVelocitySum[vertexIndex3] +
                                                         _velocity[vertexIndex0] + _velocity[vertexIndex1] +
                                                         _velocity[vertexIndex2];
                _neighborhoodNum[vertexIndex0] += 3;
                _neighborhoodNum[vertexIndex1] += 3;
                _neighborhoodNum[vertexIndex2] += 3;
                _neighborhoodNum[vertexIndex3] += 3;
            });
            

            Parallel.For(0, _vertexNum, i =>
            {
                _velocity[i] = _velocity[i] * (1 - velocitySmoothWeight) +
                               _neighborhoodVelocitySum[i] / _neighborhoodNum[i] * velocitySmoothWeight;
                _position[i] += _velocity[i] * DT;

                if (_position[i].y < -2.95)
                {
                    if (_velocity[i].y >= 0) return;

                    var collisionVertexVelocityN = new Vector3(0, _velocity[i].y, 0);
                    var collisionVertexVelocityT = _velocity[i] - collisionVertexVelocityN;

                    var a = Mathf.Max(
                        1 - CollisionRestitutionT * (1 + CollisionRestitutionN) * collisionVertexVelocityN.magnitude /
                        collisionVertexVelocityT.magnitude, 0);
                    collisionVertexVelocityN *= -CollisionRestitutionN;
                    collisionVertexVelocityT *= a;

                    _velocity[i] = collisionVertexVelocityN + collisionVertexVelocityT;
                    _position[i].y = -2.94f;
                }
            });
        }

        #endregion


        private static Matrix4x4 MinusMatrix(Matrix4x4 m1, Matrix4x4 m2)
        {
            return new Matrix4x4(
                m1.GetColumn(0) - m2.GetColumn(0),
                m1.GetColumn(1) - m2.GetColumn(1),
                m1.GetColumn(2) - m2.GetColumn(2),
                m1.GetColumn(3) - m2.GetColumn(3));
        }

        private static Matrix4x4 AddMatrix(Matrix4x4 m1, Matrix4x4 m2) =>
            new(
                m1.GetColumn(0) + m2.GetColumn(0),
                m1.GetColumn(1) + m2.GetColumn(1),
                m1.GetColumn(2) + m2.GetColumn(2),
                m1.GetColumn(3) + m2.GetColumn(3));

        private static Matrix4x4 MatrixMulFloat(Matrix4x4 m1, float m2) =>
            new(
                m1.GetColumn(0) * m2,
                m1.GetColumn(1) * m2,
                m1.GetColumn(2) * m2,
                m1.GetColumn(3) * m2);

        private static Matrix4x4 _Method1(Matrix4x4 deformationGradient)
        {
            var greenStrain = deformationGradient.transpose * deformationGradient;
            greenStrain = MinusMatrix(greenStrain, Matrix4x4.identity);
            greenStrain = MatrixMulFloat(greenStrain, 0.5f);

            var stress = MatrixMulFloat(greenStrain, 2 * Stiffness1);
            var greenStrainTrace = greenStrain.m00 + greenStrain.m11 + greenStrain.m22;
            stress = AddMatrix(stress, MatrixMulFloat(Matrix4x4.identity, Stiffness0 * greenStrainTrace));

            return deformationGradient * stress;
        }

        private Matrix4x4 _Method2(Matrix4x4 deformationGradient)
        {
            var u = new Matrix4x4();
            var s = new Matrix4x4();
            var v = new Matrix4x4();
            _svd.svd(deformationGradient, ref u, ref s, ref v);

            var diag = new Matrix4x4();
            var I = s.m00 * s.m00 + s.m11 * s.m11 + s.m22 * s.m22;
            diag.m00 = 2.0f * Stiffness0 * (I - 3.0f) * s.m00 + Stiffness1 * (s.m00 * s.m00 - 1.0f) * s.m00;
            diag.m11 = 2.0f * Stiffness0 * (I - 3.0f) * s.m11 + Stiffness1 * (s.m11 * s.m11 - 1.0f) * s.m11;
            diag.m22 = 2.0f * Stiffness0 * (I - 3.0f) * s.m22 + Stiffness1 * (s.m22 * s.m22 - 1.0f) * s.m22;

            return u * diag * v.transpose;
        }
    }
}
using System;
using System.IO;
using UnityEngine;
using UnityEngine.Serialization;

namespace FiniteElementMethod
{
    public class FvmColor : MonoBehaviour
    {
        [FormerlySerializedAs("mVelocitySmoothWeight")] [FormerlySerializedAs("m_VelocitySmoothWeight")] [Range(0, 0.01f)]
        public float velocitySmoothWeight = 0.005f;
        [FormerlySerializedAs("useHyperelasticModels")] [FormerlySerializedAs("m_UseHyperelasticModels")] public bool useHyperElasticModels = false;
        private const float TimeStep 	= 0.003f;
        private const float Mass 		= 1;
        private const float Stiffness0	= 5000.0f;
        private const float Stiffness1 	= 5000.0f;
        private const float Damp		= 0.999f;
        private const float CollisionRestitutionN = 0.5f;
        private const float CollisionRestitutionT = 0.5f;

        private int[] m_Tetrahedra;
        private int m_TetrahedraNum;

        private Vector3[] 	m_Force;
        private Vector3[] 	m_Velocity;
        private Vector3[] 	m_Position;
        private int         m_VertexNum;

        private Matrix4x4[] m_InvDm;
 
        //For Laplacian smoothing.
        private Vector3[] m_NeighborhoodVelocitySum;
        private int[] m_NeighborhoodNum;
 
        private Color[] m_VertexColor;
 
        private SVD m_SVD = new SVD();

        Matrix4x4 Build_Edge_Matrix(int tetrahedra)
        {
            Vector4 X10 = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]] - m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
            Vector4 X20 = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]] - m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
            Vector4 X30 = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]] - m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
            Matrix4x4 edgeMatrix = new Matrix4x4();
            edgeMatrix.SetColumn(0, X10);
            edgeMatrix.SetColumn(1, X20);
            edgeMatrix.SetColumn(2, X30);
            edgeMatrix.SetColumn(3, new Vector4(0, 0, 0, 1));

            return edgeMatrix;
        }

        void Start()
        {
            // FILO IO: Read the house model from files.
            // The model is from Jonathan Schewchuk's Stellar lib.
            {
                string fileContent = File.ReadAllText("Assets/Resources/house2.ele");
                string[] strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);

                m_TetrahedraNum = int.Parse(strings[0]);
                m_Tetrahedra = new int[m_TetrahedraNum * 4];

                for(int tetrahedra = 0; tetrahedra < m_TetrahedraNum; tetrahedra++)
                {
                    m_Tetrahedra[tetrahedra * 4 + 0] = int.Parse(strings[tetrahedra * 5 + 4]) - 1;
                    m_Tetrahedra[tetrahedra * 4 + 1] = int.Parse(strings[tetrahedra * 5 + 5]) - 1;
                    m_Tetrahedra[tetrahedra * 4 + 2] = int.Parse(strings[tetrahedra * 5 + 6]) - 1;
                    m_Tetrahedra[tetrahedra * 4 + 3] = int.Parse(strings[tetrahedra * 5 + 7]) - 1;
                }
            }
            {
                string fileContent = File.ReadAllText("Assets/Resources/house2.node");
                string[] strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
                m_VertexNum = int.Parse(strings[0]);
                m_Position = new Vector3[m_VertexNum];
                m_VertexColor = new Color[m_VertexNum];
                m_NeighborhoodVelocitySum = new Vector3[m_VertexNum];
                m_NeighborhoodNum = new int[m_VertexNum];
                for (int i = 0; i < m_VertexNum; i++)
                {
                    m_Position[i].x = float.Parse(strings[i * 5 + 5]) * 0.4f;
                    m_Position[i].y = float.Parse(strings[i * 5 + 6]) * 0.4f;
                    m_Position[i].z = float.Parse(strings[i * 5 + 7]) * 0.4f;
                    m_VertexColor[i] = new Color(0, 0, 0, 1);
                    m_NeighborhoodVelocitySum[i] = new Vector3(0, 0, 0);
                    m_NeighborhoodNum[i] = 0;
                }
                //Centralize the model.
                Vector3 center = Vector3.zero;
                for (int i = 0; i < m_VertexNum; i++)
                    center += m_Position[i];
                center = center / m_VertexNum;
                for(int i = 0; i < m_VertexNum; i++)
                {
                    m_Position[i] -= center;
                    float temp = m_Position[i].y;
                    m_Position[i].y = m_Position[i].z;
                    m_Position[i].z = temp;
                }
            }

            //Create triangle mesh.
            Vector3[] vertices = new Vector3[m_TetrahedraNum * 12];
            int vertex_number = 0;
            for(int tetrahedra = 0; tetrahedra < m_TetrahedraNum; tetrahedra++)
            {
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]];

                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]];

                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]];

                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]];
            }

            int[] triangles = new int[m_TetrahedraNum * 12];
            for(int t = 0; t < m_TetrahedraNum * 4; t++)
            {
                triangles[t * 3 + 0] = t * 3 + 0;
                triangles[t * 3 + 1] = t * 3 + 1;
                triangles[t * 3 + 2] = t * 3 + 2;
            }
            Mesh mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.triangles = triangles;
            mesh.RecalculateNormals();

            m_Velocity = new Vector3[m_VertexNum];
            m_Force = new Vector3[m_VertexNum];

            m_InvDm = new Matrix4x4[m_TetrahedraNum];
            for (int t = 0; t < m_TetrahedraNum; t++)
            {
                Matrix4x4 edgeMatrix = Build_Edge_Matrix(t);
                m_InvDm[t] = edgeMatrix.inverse;
            }
        }

        Matrix4x4 MinusMartix(Matrix4x4 m1, Matrix4x4 m2)
        {
            return new Matrix4x4(
                m1.GetColumn(0) - m2.GetColumn(0),
                m1.GetColumn(1) - m2.GetColumn(1),
                m1.GetColumn(2) - m2.GetColumn(2),
                m1.GetColumn(3) - m2.GetColumn(3));
        }
    
        Matrix4x4 AddMartix(Matrix4x4 m1, Matrix4x4 m2)
        {
            return new Matrix4x4(
                m1.GetColumn(0) + m2.GetColumn(0),
                m1.GetColumn(1) + m2.GetColumn(1),
                m1.GetColumn(2) + m2.GetColumn(2),
                m1.GetColumn(3) + m2.GetColumn(3));
        }
    
        Matrix4x4 MartixMulFloat(Matrix4x4 m1, float m2)
        {
            return new Matrix4x4(
                m1.GetColumn(0) * m2,
                m1.GetColumn(1) * m2,
                m1.GetColumn(2) * m2,
                m1.GetColumn(3) * m2);
        }

        Matrix4x4 _Method1(Matrix4x4 deformationGradient)
        {
            Matrix4x4 greenStrain = deformationGradient.transpose * deformationGradient;
            greenStrain = MinusMartix(greenStrain, Matrix4x4.identity);
            greenStrain = MartixMulFloat(greenStrain, 0.5f);

            Matrix4x4 stress = MartixMulFloat(greenStrain, 2 * Stiffness1);
            float greenStrainTrace = greenStrain.m00 + greenStrain.m11 + greenStrain.m22;
            stress = AddMartix(stress, MartixMulFloat(Matrix4x4.identity, Stiffness0 * greenStrainTrace));

            return deformationGradient * stress;   
        }

        Matrix4x4 _Method2(Matrix4x4 deformationGradient)
        {
            Matrix4x4 u = new Matrix4x4();
            Matrix4x4 s = new Matrix4x4();
            Matrix4x4 v = new Matrix4x4();
            m_SVD.svd(deformationGradient, ref u, ref s, ref v);

            Matrix4x4 diag = new Matrix4x4();
            float I = s.m00 * s.m00 + s.m11 * s.m11 + s.m22 * s.m22;
            diag.m00 = 2.0f * Stiffness0 * (I - 3.0f) * s.m00 + Stiffness1 * (s.m00 * s.m00 - 1.0f) * s.m00;
            diag.m11 = 2.0f * Stiffness0 * (I - 3.0f) * s.m11 + Stiffness1 * (s.m11 * s.m11 - 1.0f) * s.m11;
            diag.m22 = 2.0f * Stiffness0 * (I - 3.0f) * s.m22 + Stiffness1 * (s.m22 * s.m22 - 1.0f) * s.m22;

            return u * diag * v.transpose;
        }

        void _Update()
        {
            if(Input.GetKeyDown(KeyCode.Space))
            {
                for (int i = 0; i < m_VertexNum; i++)
                    m_Velocity[i].y += 0.2f;
            }

            for(int i = 0; i < m_VertexNum; i++)
            {
                m_Force[i] = new Vector3(0, -9.8f * Mass, 0);
                m_VertexColor[i] = new Color(0, 0, 0, 1);
            }

            for (int tetrahedra = 0; tetrahedra < m_TetrahedraNum; tetrahedra++)
            {
                Matrix4x4 deformationGradient = Build_Edge_Matrix(tetrahedra) * m_InvDm[tetrahedra];

                Matrix4x4 firstPKStress;
                if (useHyperElasticModels)
                {
                    firstPKStress = _Method2(deformationGradient);
                }
                else
                {
                    firstPKStress = _Method1(deformationGradient);
                }

                Matrix4x4 force = MartixMulFloat(firstPKStress, -1.0f / (6.0f * m_InvDm[tetrahedra].determinant));
                force = force * m_InvDm[tetrahedra].transpose;

                Vector3 force1 = force.GetColumn(0);
                Vector3 force2 = force.GetColumn(1);
                Vector3 force3 = force.GetColumn(2);
                Vector3 force0 = -force1 - force2 - force3;
                int vertexIndex0 = m_Tetrahedra[tetrahedra * 4 + 0];
                int vertexIndex1 = m_Tetrahedra[tetrahedra * 4 + 1];
                int vertexIndex2 = m_Tetrahedra[tetrahedra * 4 + 2];
                int vertexIndex3 = m_Tetrahedra[tetrahedra * 4 + 3];
                m_Force[vertexIndex0] += force0;
                m_Force[vertexIndex1] += force1;
                m_Force[vertexIndex2] += force2;
                m_Force[vertexIndex3] += force3;

                m_VertexColor[vertexIndex0].r += force0.magnitude * 0.005f;
                m_VertexColor[vertexIndex1].r += force1.magnitude * 0.005f;
                m_VertexColor[vertexIndex2].r += force2.magnitude * 0.005f;
                m_VertexColor[vertexIndex3].r += force3.magnitude * 0.005f;
            }

            for(int i = 0; i < m_VertexNum; i++)
            {
                m_Velocity[i] += m_Force[i] / Mass * TimeStep;
                m_Velocity[i] *= Damp;
            }

            for(int tetrahedra = 0; tetrahedra < m_TetrahedraNum; tetrahedra++)
            {
                int vertexIndex0 = m_Tetrahedra[tetrahedra * 4 + 0];
                int vertexIndex1 = m_Tetrahedra[tetrahedra * 4 + 1];
                int vertexIndex2 = m_Tetrahedra[tetrahedra * 4 + 2];
                int vertexIndex3 = m_Tetrahedra[tetrahedra * 4 + 3];
                m_NeighborhoodVelocitySum[vertexIndex0] = m_NeighborhoodVelocitySum[vertexIndex0] + m_Velocity[vertexIndex1] + m_Velocity[vertexIndex2] + m_Velocity[vertexIndex3];
                m_NeighborhoodVelocitySum[vertexIndex1] = m_NeighborhoodVelocitySum[vertexIndex1] + m_Velocity[vertexIndex0] + m_Velocity[vertexIndex2] + m_Velocity[vertexIndex3];
                m_NeighborhoodVelocitySum[vertexIndex2] = m_NeighborhoodVelocitySum[vertexIndex2] + m_Velocity[vertexIndex0] + m_Velocity[vertexIndex1] + m_Velocity[vertexIndex3];
                m_NeighborhoodVelocitySum[vertexIndex3] = m_NeighborhoodVelocitySum[vertexIndex3] + m_Velocity[vertexIndex0] + m_Velocity[vertexIndex1] + m_Velocity[vertexIndex2];
                m_NeighborhoodNum[vertexIndex0] += 3;
                m_NeighborhoodNum[vertexIndex1] += 3;
                m_NeighborhoodNum[vertexIndex2] += 3;
                m_NeighborhoodNum[vertexIndex3] += 3;
            }

            for (int i = 0; i < m_VertexNum; i++)
            {
                m_Velocity[i] = m_Velocity[i] * (1 - velocitySmoothWeight) + m_NeighborhoodVelocitySum[i] / m_NeighborhoodNum[i] * velocitySmoothWeight;
                m_Position[i] += m_Velocity[i] * TimeStep;

                if (m_Position[i].y < -2.95)
                {
                    if (m_Velocity[i].y >= 0) return;

                    Vector3 collisionVertexVelocityN = new Vector3(0, m_Velocity[i].y, 0);
                    Vector3 collisionVertexVelocityT = m_Velocity[i] - collisionVertexVelocityN;

                    float a = Mathf.Max(1 - CollisionRestitutionT * (1 + CollisionRestitutionN) * collisionVertexVelocityN.magnitude / collisionVertexVelocityT.magnitude, 0);
                    collisionVertexVelocityN *= -CollisionRestitutionN;
                    collisionVertexVelocityT *= a;

                    m_Velocity[i] = collisionVertexVelocityN + collisionVertexVelocityT;
                    m_Position[i].y = -2.94f;
                }
            }
        }

        void Update()
        {
            for (int l = 0; l < 10; l++)
                _Update();

            // Dump the vertex array for rendering.
            Vector3[] vertices = new Vector3[m_TetrahedraNum * 12];
            Color[] colors = new Color[m_TetrahedraNum * 12];
            int vertex_number = 0;
            for(int tetrahedra = 0; tetrahedra < m_TetrahedraNum; tetrahedra++)
            {
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 0]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 2]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 1]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 0]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 3]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 2]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 0]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 0]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 1]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 3]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 1]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 1]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 2]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 2]];
                colors[vertex_number] = m_VertexColor[m_Tetrahedra[tetrahedra * 4 + 3]];
                vertices[vertex_number++] = m_Position[m_Tetrahedra[tetrahedra * 4 + 3]];
            }
            Mesh mesh = GetComponent<MeshFilter>().mesh;
            mesh.vertices = vertices;
            mesh.colors = colors;
            mesh.RecalculateNormals();
        }
    }
}

using System.Drawing;
using UnityEngine;

namespace ShallowWave
{
    public class WaveMotion : MonoBehaviour
    {
        #region Fields

        private const int size = 100;
        float rate = 0.005f;
        float gamma = 0.004f;
        private float _damping = 0.996f;
        float[,] old_h;
        float[,] low_h;
        float[,] vh;
        float[,] b;

        bool[,] cg_mask;
        float[,] cg_p;
        float[,] cg_r;
        float[,] cg_Ap;
        bool tag = true;

        Vector3 cube_v = Vector3.zero;
        Vector3 cube_w = Vector3.zero;
        private Bounds _cubeBounds;
        private Bounds _blockBounds;
        
        int _cubeLi, _cubeUi, _cubeLj, _cubeUj;
        int _blockLi, _blockUi, _blockLj, _blockUj;
        
        float Rho = 0.01f; // density of water
        float g = 9.8f;
        GameObject Cube;
        GameObject Block;
        #endregion

        #region Unity Methods

        private void Start()
        {
            Cube = GameObject.Find("Cube");
            Block = GameObject.Find("Block");
            var mesh = GetComponent<MeshFilter>().mesh;
            mesh.Clear();

            // ReSharper disable once InconsistentNaming
            var X = new Vector3[size * size];

            for (var i = 0; i < size; i++)
            for (var j = 0; j < size; j++)
            {
                X[i * size + j].x = i * 0.1f - size * 0.05f;
                X[i * size + j].y = 0;
                X[i * size + j].z = j * 0.1f - size * 0.05f;
            }

            var T = new int[(size - 1) * (size - 1) * 6];
            var index = 0;
            for (var i = 0; i < size - 1; i++)
            for (var j = 0; j < size - 1; j++)
            {
                T[index * 6 + 0] = (i + 0) * size + (j + 0);
                T[index * 6 + 1] = (i + 0) * size + (j + 1);
                T[index * 6 + 2] = (i + 1) * size + (j + 1);
                T[index * 6 + 3] = (i + 0) * size + (j + 0);
                T[index * 6 + 4] = (i + 1) * size + (j + 1);
                T[index * 6 + 5] = (i + 1) * size + (j + 0);
                index++;
            }

            mesh.vertices = X;
            mesh.triangles = T;
            mesh.RecalculateNormals();

            low_h = new float[size, size];
            old_h = new float[size, size];
            vh = new float[size, size];
            b = new float[size, size];

            cg_mask = new bool [size, size];
            cg_p = new float[size, size];
            cg_r = new float[size, size];
            cg_Ap = new float[size, size];

            for (var i = 0; i < size; i++)
            for (var j = 0; j < size; j++)
            {
                low_h[i, j] = 99999;
                old_h[i, j] = 0;
                vh[i, j] = 0;
            }
        }

        private void Update()
        {
            var mesh = GetComponent<MeshFilter>().mesh;
            // ReSharper disable once InconsistentNaming
            var X = mesh.vertices;
            var newH = new float[size, size];
            var h = new float[size, size];

            // Load X.y into h.
            for (var i = 0; i < size; i++)
            for (var j = 0; j < size; j++)
                h[i, j] = X[i * size + j].y;

            // for (var i = 0; i < X.Length; i++)
            // {
            //     h[i / Size, i % Size] = X[i].y;
            // }

            if (Input.GetKeyDown("r"))
            {
                // Add random water

                #region way 1

                // var i = (int)(Random.Range(0f, 1f) * Size);
                // var j = (int)(Random.Range(0f, 1f) * Size);
                // if (j < 1) j = 1;
                // if (i < 1) i = 1;
                // if (j >= Size - 1) j = Size - 2;
                // if (i >= Size - 1) i = Size - 2;
                // i = 40;
                // j = 50;
                // var v = 0.2f * Random.Range(0.5f, 1f) * 4;
                // h[i, j] += v;
                // h[i - 1, j] -= v / 4;
                // h[i + 1, j] -= v / 4;
                // h[i, j - 1] -= v / 4;
                // h[i, j + 1] -= v / 4;

                #endregion

                #region way 2

                // var i = Random.Range(1, Size - 1);
                // var j = Random.Range(1, Size - 1);
                //
                // var r = Random.Range(0.5f, 2.0f);
                // h[i, j] += r;
                // h[i - 1, j] -= r / 8;
                // h[i - 1, j - 1] -= r / 8;
                // h[i + 1, j] -= r / 8;
                // h[i + 1, j + 1] -= r / 8;
                // h[i, j - 1] -= r / 8;
                // h[i + 1, j - 1] -= r / 8;
                // h[i, j + 1] -= r / 8;
                // h[i - 1, j + 1] -= r / 8;

                #endregion

                #region way 3

                var r = Random.Range(0.5f, 1.0f);
                var k = Random.Range(1, size - 2);
                var l = Random.Range(1, size - 2);
                h[k, l] += r;

                //Volume Preservation 
                for (var i = 0; i < size; i++)
                {
                    for (var j = 0; j < size; j++)
                    {
                        if (i != k && j != l)
                        {
                            h[i, j] -= r / (size * size - 1);
                        }
                    }
                }

                #endregion
            }

            for (var l = 0; l < 8; l++)
            {
                ShallowWave(old_h, h, newH);
            }

            // Store h back into X.y and recalculate normal
            for (var i = 0; i < size; i++)
            for (var j = 0; j < size; j++)
                X[i * size + j].y = h[i, j];
            mesh.vertices = X;
            mesh.RecalculateNormals();
        }

        #endregion

        #region Private Methods

        private void A_Times(bool[,] mask, float[,] x, float[,] Ax, int li, int ui, int lj, int uj)
        {
            for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    Ax[i, j] = 0;
                    if (i != 0) Ax[i, j] -= x[i - 1, j] - x[i, j];
                    if (i != size - 1) Ax[i, j] -= x[i + 1, j] - x[i, j];
                    if (j != 0) Ax[i, j] -= x[i, j - 1] - x[i, j];
                    if (j != size - 1) Ax[i, j] -= x[i, j + 1] - x[i, j];
                }
        }

        private float Dot(bool[,] mask, float[,] x, float[,] y, int li, int ui, int lj, int uj)
        {
            float ret = 0;
            for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    ret += x[i, j] * y[i, j];
                }

            return ret;
        }

        private void Conjugate_Gradient(bool[,] mask, float[,] b, float[,] x, int li, int ui, int lj, int uj)
        {
            //Solve the Laplacian problem by CG.
            A_Times(mask, x, cg_r, li, ui, lj, uj);

            for (int i = li; i <= ui; i++)
            for (int j = lj; j <= uj; j++)
                if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                {
                    cg_p[i, j] = cg_r[i, j] = b[i, j] - cg_r[i, j];
                }

            float rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);

            for (int k = 0; k < 128; k++)
            {
                if (rk_norm < 1e-10f) break;
                A_Times(mask, cg_p, cg_Ap, li, ui, lj, uj);
                float alpha = rk_norm / Dot(mask, cg_p, cg_Ap, li, ui, lj, uj);

                for (int i = li; i <= ui; i++)
                for (int j = lj; j <= uj; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        x[i, j] += alpha * cg_p[i, j];
                        cg_r[i, j] -= alpha * cg_Ap[i, j];
                    }

                float _rk_norm = Dot(mask, cg_r, cg_r, li, ui, lj, uj);
                float beta = _rk_norm / rk_norm;
                rk_norm = _rk_norm;

                for (int i = li; i <= ui; i++)
                for (int j = lj; j <= uj; j++)
                    if (i >= 0 && j >= 0 && i < size && j < size && mask[i, j])
                    {
                        cg_p[i, j] = cg_r[i, j] + beta * cg_p[i, j];
                    }
            }
        }

        private void ShallowWave(float[,] old_h, float[,] h, float[,] new_h)
        {
            // TODO: Compute new_h based on the shallow wave model.

            for (var i = 0; i < size; i++)
            for (var j = 0; j < size; j++)
            {
                new_h[i, j] = h[i, j] + _damping * (h[i, j] - old_h[i, j]);

                if (i - 1 >= 0) new_h[i, j] += rate * (h[i - 1, j] - h[i, j]);
                if (i + 1 < size) new_h[i, j] += rate * (h[i + 1, j] - h[i, j]);
                if (j - 1 >= 0) new_h[i, j] += rate * (h[i, j - 1] - h[i, j]);
                if (j + 1 < size) new_h[i, j] += rate * (h[i, j + 1] - h[i, j]);
            }

            var cubeMin = _cubeBounds.min;
            var cubeMax = _cubeBounds.max;
            var blockMin = _blockBounds.min;
            var blockMax = _blockBounds.max;
            
            for (var i = 0; i < size; i++)
                for (var j = 0; j < size; j++)
                {
                    cg_mask[i, j] = false;
                    b[i, j] = 0;
                    low_h[i, j] = 0;
                    vh[i, j] = 0;
                }

            UpdateMask();
              
            for (var i = 0; i < size; i++)
                for (var j = 0; j < size; j++)
                {
                    if (cg_mask[i, j])
                    {
                        b[i, j] = (new_h[i, j] - low_h[i, j]) / rate;
                    }
                }
            
            
            
            //Step 2: Block->Water coupling
            //TODO: for block 1, calculate low_h.
           
            

            //TODO: then set up b and cg_mask for conjugate gradient.
            //TODO: Solve the Poisson equation to obtain vh (virtual height).

            //TODO: for block 2, calculate low_h.
            //TODO: then set up b and cg_mask for conjugate gradient.

            //TODO: Solve the Poisson equation to obtain vh (virtual height).
            Conjugate_Gradient(cg_mask, b, vh, _cubeLi, _cubeUi, _cubeLj, _cubeUj);
            Conjugate_Gradient(cg_mask, b, vh, _blockLi, _blockUi, _blockLj, _blockUj);
            //TODO: Diminish vh.

            // for (var i = 0; i < size; i++)
            // {
            //     for (var j = 0; j < size; j++)
            //     {
            //         if (cg_mask[i, j])
            //         {
            //             vh[i, j] *= gamma;
            //         }
            //     }
            // }

            //TODO: Update new_h by vh.
            for (var i = 0; i < size; i++)
                for (var j = 0; j < size; j++)
                {
                    if (i - 1 >= 0) new_h[i, j] += gamma * rate * (vh[i - 1, j] - vh[i, j]);
                    if (i + 1 < size) new_h[i, j] += gamma * rate * (vh[i + 1, j] - vh[i, j]);
                    if (j - 1 >= 0) new_h[i, j] += gamma * rate * (vh[i, j - 1] - vh[i, j]);
                    if (j + 1 < size) new_h[i, j] += gamma * rate * (vh[i, j + 1] - vh[i, j]);
                }
            var mesh = GetComponent<MeshFilter>().mesh;
            var X = mesh.vertices;
            var _cubeFlotage = new Vector3();
            
            var _blockFlotage = new Vector3();

            var _cubeTorque = new Vector3();
            var _blockTorque = new Vector3();

            for (int i = _cubeLi; i <= _cubeUi; i++)
            {
                for (int j = _cubeLj; j <= _cubeUj; j++)
                {
                    Vector3 _f = new Vector3(0, Rho * 0.01f * vh[i, j] * g, 0);
                    _cubeFlotage += _f ;

                    Vector3 _hitPoint = new Vector3(X[i * size + j].x, low_h[i,j], X[i * size + j].z);
                    Vector3 _r = _hitPoint -Cube.transform.position;

                    Vector3 _torque = Vector3.Cross(_r, _f);
                    _cubeTorque += _torque;
                }
            }
            for (int i = _blockLi; i <= _blockUi; i++)
            {
                for (int j = _blockLj; j <= _blockUj; j++)
                {
                    Vector3 _f = new Vector3(0, Rho * 0.01f * vh[i, j] * g, 0);
                    _blockFlotage += _f;

                    Vector3 _hitPoint = new Vector3(X[i * size + j].x, low_h[i, j], X[i * size + j].z);
                    Vector3 _r = _hitPoint - Block.transform.position;

                    Vector3 _torque = Vector3.Cross(_r, _f);
                    _blockTorque += _torque;
                }
            }
            
            Cube.SendMessage("UpdateForce", _cubeFlotage);
            Block.SendMessage("UpdateForce", _blockFlotage);

            Cube.SendMessage("updateTorque", _cubeTorque);
            Block.SendMessage("updateTorque", _blockTorque);

            // old_h <- h; h <- new_h
            for (int i = 0; i < size; i++)
            {
                for (int j = 0; j < size; j++)
                {
                    old_h[i, j] = h[i, j];
                    h[i, j] = new_h[i, j];
                }
            }
            
            //Step 3
            //TODO: old_h <- h; h <- new_h;
            // for (var i = 0; i < size; i++)
            // {
            //     for (var j = 0; j < size; j++)
            //     {
            //         old_h[i, j] = h[i, j];
            //         h[i, j] = new_h[i, j];
            //     }
            // }
            //
            // for (var i = 0; i < size; i++)
            // {
            //     for (var j = 0; j < size; j++)
            //     {
            //         old_h[i, j] = h[i, j];
            //         h[i, j] = new_h[i, j];
            //     }
            // }
            //Step 4: Water->Block coupling.
            //More TODO here.
        }

        private void UpdateMask()
    {
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] X = mesh.vertices;
		Vector3 _cubeMin = _cubeBounds.min;
		Vector3 _cubeMax = _cubeBounds.max;
		Vector3 _blockMin = _blockBounds.min;
		Vector3 _blockMax = _blockBounds.max;

		
		if(_cubeMin.y < 0)
        {
			_cubeLi = size - 1; _cubeLj = size - 1;
			_cubeUi = 0; _cubeUj = 0;
			for (int i = 0; i < size * size; i++)
			{
				if (X[i].x < _cubeMax.x && X[i].x > _cubeMin.x && X[i].z < _cubeMax.z && X[i].z > _cubeMin.z)
				{
					int _i = i / size;
					int _j = i % size;
					cg_mask[_i, _j] = true;
					//low_h[_i, _j] = _cubeMin.y;

					if (_i < _cubeLi) _cubeLi = _i;
					if (_i > _cubeUi) _cubeUi = _i;
					if (_j < _cubeLj) _cubeLj = _j;
					if (_j > _cubeUj) _cubeUj = _j;
				}
			}
			// update low_h
			for (int i = _cubeLi; i <= _cubeUi; i++)
			{
				for (int j = _cubeLj; j <= _cubeUj; j++)
				{
					RaycastHit hit;
					Ray ray = new Ray(new Vector3(X[i * size + j].x, -2, X[i * size + j].z), new Vector3(0, 1, 0));
					if (Physics.Raycast(ray, out hit))
					{
						if (hit.collider != null)
						{
							low_h[i, j] = hit.point.y;
						}
					}
				}
			}
		}
		
		if(_blockMin.y < 0)
        {
			_blockLi = size - 1; _blockLj = size - 1;
			_blockUi = 0; _blockUj = 0;
			for (int i = 0; i < size * size; i++)
			{
				if (X[i].x < _blockMax.x && X[i].x > _blockMin.x && X[i].z < _blockMax.z && X[i].z > _blockMin.z)
				{
					int _i = i / size;
					int _j = i % size;
					cg_mask[_i, _j] = true;
					//low_h[_i, _j] = _blockMin.y;

					if (_i < _blockLi) _blockLi = _i;
					if (_i > _blockUi) _blockUi = _i;
					if (_j < _blockLj) _blockLj = _j;
					if (_j > _blockUj) _blockUj = _j;
				}
			}
			for (int i = _blockLi; i <= _blockUi; i++)
			{
				for (int j = _blockLj; j <= _blockUj; j++)
				{
					RaycastHit hit;
					Ray ray = new Ray(new Vector3(X[i * size + j].x, -2, X[i * size + j].z), new Vector3(0, 1, 0));
					if (Physics.Raycast(ray, out hit))
					{
						if (hit.collider != null)
						{
							low_h[i, j] = hit.point.y;
						}
					}
				}
			}
		}
	}
        void UpdateCubeBounds(Bounds bounds)
        {
            _cubeBounds = bounds;
        }
        void UpdateBlockBounds(Bounds bounds)
        {
            _blockBounds = bounds;
        }
        #endregion
    }
}
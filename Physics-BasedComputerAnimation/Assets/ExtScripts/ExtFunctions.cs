using UnityEngine;

namespace ExtScripts
{
    public abstract class ExtFunctions
    {
        public static void QuickSort(ref int[] a, int l, int r)
        {
            while (true)
            {
                if (l < r)
                {
                    var j = QuickSortPartition(ref a, l, r);
                    QuickSort(ref a, l, j - 1);
                    l = j + 1;
                    continue;
                }

                break;
            }
        }

        private static int QuickSortPartition(ref int[] a, int l, int r)
        {
            var pivot0 = a[l * 2 + 0];
            var pivot1 = a[l * 2 + 1];
            var i = l;
            var j = r + 1;

            while (true)
            {
                do ++i;
                while (i <= r && (a[i * 2] < pivot0 || a[i * 2] == pivot0 && a[i * 2 + 1] <= pivot1));
                do --j;
                while (a[j * 2] > pivot0 || a[j * 2] == pivot0 && a[j * 2 + 1] > pivot1);
                if (i >= j) break;
                Swap(ref a[i * 2], ref a[j * 2]);
                Swap(ref a[i * 2 + 1], ref a[j * 2 + 1]);
            }

            Swap(ref a[l * 2 + 0], ref a[j * 2 + 0]);
            Swap(ref a[l * 2 + 1], ref a[j * 2 + 1]);
            return j;
        }

        public static void Swap(ref int a, ref int b)
        {
            (a, b) = (b, a);
        }

        public static Matrix4x4 GetCrossMatrix(Vector3 a)
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

        public static Matrix4x4 MatrixSubtraction(Matrix4x4 a, Matrix4x4 b)
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

        public static Matrix4x4 MatrixMultiplyFloat(Matrix4x4 a, float b)
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

        public static Quaternion Add(Quaternion a, Quaternion b)
        {
            a.x += b.x;
            a.y += b.y;
            a.z += b.z;
            a.w += b.w;
            return a.normalized;
        }
    }
}
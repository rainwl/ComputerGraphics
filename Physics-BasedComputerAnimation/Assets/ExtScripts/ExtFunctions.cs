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
    }
}

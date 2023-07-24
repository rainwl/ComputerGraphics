using UnityEngine;
using Unity.Jobs;
using Unity.Collections;

public struct SumJob : IJob
{
    [ReadOnly]
    public NativeArray<int> arrayA;
    public NativeArray<int> arrayB;
    public int index;

    public void Execute()
    {
        arrayB[index] = arrayA[index] + 1;
        Debug.Log($"Thread id:{System.Threading.Thread.CurrentThread.ManagedThreadId}");
    }
}

public class JobSample : MonoBehaviour
{
    public int[] testArray = { 1, 2, 3 };
    private void Start()
    {
        Debug.Log($"main thread:{System.Threading.Thread.CurrentThread.ManagedThreadId}");
        
    }

    private void OnGUI()
    {
        if (GUI.Button(new Rect(0, 0, 120, 40), "RUN"))
        {
            var arrayA = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayB = new NativeArray<int>(testArray, Allocator.TempJob);
            var sumJob = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 2 };
            sumJob.Run();
            
            arrayB.CopyTo(testArray);
            arrayA.Dispose();
            arrayB.Dispose();
        }

        if (GUI.Button(new Rect(0, 50, 120, 40), "Schedule"))
        {
            var arrayA = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayB = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob0 = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 2 };
            var jobHandle = sumJob0.Schedule();
            jobHandle.Complete();
            
            var sumJob1 = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 1 };
            jobHandle = sumJob1.Schedule();
            jobHandle.Complete();
            
            arrayB.CopyTo(testArray);
            arrayA.Dispose();
            arrayB.Dispose();
        }

        if (GUI.Button(new Rect(0, 100, 120, 40), "ScheduleParallel"))
        {
            var arrayA = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayB = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob0 = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 2 };
            var jobHandle = sumJob0.Schedule();
            Debug.Log($"run job handle{jobHandle}");
            
            var arrayC = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayD = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob1 = new SumJob() { arrayA = arrayC, arrayB = arrayD, index = 1 };
            var jobHandle1 = sumJob1.Schedule();
            Debug.Log($"run job handle{jobHandle1}");
            
            jobHandle.Complete();
            jobHandle1.Complete();
            
            arrayB.CopyTo(testArray);
            arrayC.CopyTo(testArray);
            
            arrayA.Dispose();
            arrayB.Dispose();
            arrayC.Dispose();
            arrayD.Dispose();
        }
        if (GUI.Button(new Rect(0, 150, 120, 40), "ScheduleParallelLess"))
        {
            var arrayA = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayB = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob0 = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 2 };
            var jobHandle = sumJob0.Schedule();
            Debug.Log($"run job handle{jobHandle}");
            
            //var arrayC = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayD = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob1 = new SumJob() { arrayA = arrayA, arrayB = arrayD, index = 1 };
            var jobHandle1 = sumJob1.Schedule();
            Debug.Log($"run job handle{jobHandle1}");
            
            jobHandle.Complete();
            jobHandle1.Complete();
            
            arrayB.CopyTo(testArray);
            //arrayC.CopyTo(testArray);
            
            arrayA.Dispose();
            arrayB.Dispose();
            //arrayC.Dispose();
            arrayD.Dispose();
        }
        
        if (GUI.Button(new Rect(0, 200, 120, 40), "ScheduleParallelDependency"))
        {
            var arrayA = new NativeArray<int>(testArray, Allocator.TempJob);
            var arrayB = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob0 = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 2 };
            var jobHandle = sumJob0.Schedule();
            Debug.Log($"run job handle{jobHandle}");
            
            //var arrayC = new NativeArray<int>(testArray, Allocator.TempJob);
            //var arrayD = new NativeArray<int>(testArray, Allocator.TempJob);
            
            var sumJob1 = new SumJob() { arrayA = arrayA, arrayB = arrayB, index = 1 };
            var jobHandle1 = sumJob1.Schedule(jobHandle);
            Debug.Log($"run job handle{jobHandle1}");
            
            jobHandle.Complete();
            jobHandle1.Complete();
            
            arrayB.CopyTo(testArray);
            //arrayC.CopyTo(testArray);
            
            arrayA.Dispose();
            arrayB.Dispose();
            //arrayC.Dispose();
            //arrayD.Dispose();
        }
    }
}
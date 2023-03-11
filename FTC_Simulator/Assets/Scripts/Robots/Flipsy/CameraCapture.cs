using UnityEngine;
using System.IO.MemoryMappedFiles;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;

using System;

public class CameraCapture : MonoBehaviour
{
    public Camera cameraToCapture; // Assign the camera you want to capture in the Inspector
    public string fileName;



    private Texture2D tex;
    private MemoryMappedFile mmf;
    private MemoryMappedViewAccessor accessor;
    private static int width = 640;
    private static int height = 480;
    private RenderTexture rt;

    void Start()
    {
        // create the render texture
        rt = new RenderTexture(width, height, 16, RenderTextureFormat.ARGB32);
        rt.Create();

        // Create a Texture2D to store the camera's render texture
        tex = new Texture2D(width, height, TextureFormat.RGBA32, false);

        int size = width * height * 4 * 4;
        // Create a memory-mapped file with a specified size
        mmf = MemoryMappedFile.CreateNew(fileName, size);

        // Create a memory-mapped view accessor to write to the memory-mapped file
        accessor = mmf.CreateViewAccessor();
    }
    void Update()
    {
        // Read the pixel data from the RenderTexture
        RenderTexture.active = rt;
        cameraToCapture.targetTexture = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        RenderTexture.active = null;

        // Copy the pixel data to a NativeArray
        NativeArray<Color32> pixels = tex.GetRawTextureData<Color32>();

        // the following code is necessary to avoid stuttering due to the garbage collector.
        unsafe
        {
            // acquire pointer to the shared memory region which we will write to
            byte* pointer = null;
            // Lock the shared memory buffer
            accessor.SafeMemoryMappedViewHandle.AcquirePointer(ref pointer);
            try
            {
                // compute total size to write
                int numBytes = pixels.Length * sizeof(Color32);
                // get pointer to the native array which we will copy from
                void* pixelsPtr = NativeArrayUnsafeUtility.GetUnsafePtr(pixels);
                // Copy the pixel data directly to the shared memory buffer
                Buffer.MemoryCopy(pixelsPtr, pointer, numBytes, numBytes);
            }
            finally
            {
                // Unlock the shared memory buffer
                accessor.SafeMemoryMappedViewHandle.ReleasePointer();
            }
        }

        // below is the old code that stuttered
        // Color32[] pixelsArray = pixels.ToArray(); // Convert to Color32[]
        // accessor.WriteArray(0, pixelsArray, 0, pixelsArray.Length);
    }

    void OnApplicationQuit()
    {
        // Dispose of the memory-mapped file and view accessor when the application quits
        accessor.Dispose();
        mmf.Dispose();
    }
}
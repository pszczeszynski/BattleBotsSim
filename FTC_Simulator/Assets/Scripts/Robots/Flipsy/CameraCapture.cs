using UnityEngine;
using System.IO.MemoryMappedFiles;

public class CameraCapture : MonoBehaviour
{
    public Camera cameraToCapture; // Assign the camera you want to capture in the Inspector
    public string fileName;
    private Texture2D tex;
    private MemoryMappedFile mmf;
    private MemoryMappedViewAccessor accessor;
    public RenderTexture rt;
    private int width = 640;
    private int height = 480;

    void GetCameraParams()
    {
        Matrix4x4 projectionMatrix = cameraToCapture.projectionMatrix;
        Matrix4x4 worldToCameraMatrix = cameraToCapture.worldToCameraMatrix;

        Matrix4x4 intrinsic = new Matrix4x4();
        intrinsic[0, 0] = projectionMatrix[0, 0];
        intrinsic[0, 1] = projectionMatrix[0, 1];
        intrinsic[0, 2] = projectionMatrix[0, 2];
        intrinsic[1, 1] = projectionMatrix[1, 1];
        intrinsic[1, 2] = projectionMatrix[1, 2];
        intrinsic[2, 2] = 1;
        UnityEngine.Debug.Log("intrinsic (this is actually 3x3):\n" + intrinsic);

/* Intrinsic:
1.29904	0.00000	0.00000
0.00000	1.73205	0.00000
0.00000	0.00000	1.00000
*/

/* Extrinsic:
-0.00300	 0.00006	-1.00000	-1266.68900
-0.00114	 1.00000	 0.00007	 0.02640
-1.00000	-0.00114	 0.00300	 3.99383
 0.00000	 0.00000	 0.00000	 0.00000
 */




        Matrix4x4 extrinsic = new Matrix4x4();
        extrinsic[0, 0] = worldToCameraMatrix[0, 0];
        extrinsic[0, 1] = worldToCameraMatrix[0, 1];
        extrinsic[0, 2] = worldToCameraMatrix[0, 2];
        extrinsic[0, 3] = worldToCameraMatrix[0, 3];
        extrinsic[1, 0] = worldToCameraMatrix[1, 0];
        extrinsic[1, 1] = worldToCameraMatrix[1, 1];
        extrinsic[1, 2] = worldToCameraMatrix[1, 2];
        extrinsic[1, 3] = worldToCameraMatrix[1, 3];
        extrinsic[2, 0] = worldToCameraMatrix[2, 0];
        extrinsic[2, 1] = worldToCameraMatrix[2, 1];
        extrinsic[2, 2] = worldToCameraMatrix[2, 2];
        extrinsic[2, 3] = worldToCameraMatrix[2, 3];

        UnityEngine.Debug.Log("extrinsic:\n" + extrinsic);
    }
    void Start()
    {
        GetCameraParams();


        // Create a Texture2D to store the camera's render texture
        tex = new Texture2D(width, height, TextureFormat.ARGB32, false);

        // TODO: this size is found to work, dunno what the actual size should be
        int size = width * height * 3 * 8;
        // Create a memory-mapped file with a specified size
        mmf = MemoryMappedFile.CreateNew(fileName, size);
        UnityEngine.Debug.Log("camera dimensions: " + cameraToCapture.pixelWidth + ", " + cameraToCapture.pixelHeight);

        // Create a memory-mapped view accessor to write to the memory-mapped file
        accessor = mmf.CreateViewAccessor();
    }

    void Update()
    {
        // Read the pixel data from the RenderTexture
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        RenderTexture.active = null;

        // Copy the pixel data to a separate array
        Color32[] pixels = tex.GetPixels32();
        accessor.WriteArray(0, pixels, 0, pixels.Length);
    }
    void OnApplicationQuit()
    {
        // Dispose of the memory-mapped file and view accessor when the application quits
        accessor.Dispose();
        mmf.Dispose();
    }
}
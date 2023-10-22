using UnityEngine;
using System.IO;
using System.Drawing;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class VisionAITrainer : MonoBehaviour
{
    [SerializeField]
    private Transform robotTransform;

    [SerializeField]
    private Camera captureCamera;
    [SerializeField]
    private Transform lights;
    [SerializeField]
    private Transform postProcessingVolume;

    [Range(1, 100000)]
    public int numberOfCaptures = 10;

    [SerializeField]
    private Vector2 movementBoundsX;

    [SerializeField]
    private Vector2 movementBoundsY;

    [SerializeField]
    private Vector3 CAMERA_POSITION_ORIENTATION_RANDOMNESS = new Vector3(1, 1, 1);

    [SerializeField]
    private int ROBOT_CROP_SIZE = 256;
    [SerializeField]
    private int CAMERA_WIDTH = 1280;
    [SerializeField]
    private int CAMERA_HEIGHT = 720;
    [SerializeField]
    private string SAVE_PATH_RELATIVE = "../../RobotController/MachineLearning/TrainingData/";
    private string _imagesSavePath;
    private string _labelsSavePath;

    private int currentCaptureIndex = 0;

    private Texture2D captureTexture;
    private RenderTexture rt;



    private Vector3 _originalCameraPosition;
    private Quaternion _originalCameraRotation;

    private void Start()
    {

        // disable all physics
        foreach (Rigidbody rb in FindObjectsOfType<Rigidbody>())
        {
            rb.isKinematic = true;
        }

        _imagesSavePath = Path.Combine(Application.dataPath, SAVE_PATH_RELATIVE, "TrainingInputs");
        _labelsSavePath = Path.Combine(Application.dataPath, SAVE_PATH_RELATIVE, "TrainingKeys");
        if (!Directory.Exists(_imagesSavePath))
            Directory.CreateDirectory(_imagesSavePath);

        if (!Directory.Exists(_labelsSavePath))
            Directory.CreateDirectory(_labelsSavePath);

        rt = new RenderTexture(CAMERA_WIDTH, CAMERA_HEIGHT, 16, RenderTextureFormat.ARGB32);
        rt.Create();
        captureTexture = new Texture2D(CAMERA_WIDTH, CAMERA_HEIGHT, TextureFormat.RGBA32, false);

        // save original camera position and rotation
        _originalCameraPosition = captureCamera.transform.position;
        _originalCameraRotation = captureCamera.transform.rotation;
    }

    private int i = 0;
    public void Update()
    {
        i ++;

        if (i < numberOfCaptures)
        {
            RandomizeRobotPosition();
            RandomizeCamera();
            RandomizeRobotRotation();
            RandomizeLights();
            RandomizePostProcessingVolume();

            CaptureAndSaveData();
        }
        else if (i == numberOfCaptures)
        {
            Debug.Log("Training data generation completed.");
            captureCamera.transform.position = _originalCameraPosition;
            captureCamera.transform.rotation = _originalCameraRotation;
        }

    }

    private void RandomizeCamera()
    {
        // move the camera within CAMERA_POSITION_ORIENTATION_RANDOMNESS from it's original position
        Vector3 randomPosition = _originalCameraPosition + new Vector3(
            Random.Range(-CAMERA_POSITION_ORIENTATION_RANDOMNESS.x, CAMERA_POSITION_ORIENTATION_RANDOMNESS.x),
            Random.Range(-CAMERA_POSITION_ORIENTATION_RANDOMNESS.y, CAMERA_POSITION_ORIENTATION_RANDOMNESS.y),
            Random.Range(-CAMERA_POSITION_ORIENTATION_RANDOMNESS.z, CAMERA_POSITION_ORIENTATION_RANDOMNESS.z)
        );
        captureCamera.transform.position = randomPosition;
    }

    public void GenerateTrainingData()
    {
        for (int i = 0; i < numberOfCaptures; i++)
        {
            RandomizeRobotPosition();
            RandomizeRobotRotation();

            CaptureAndSaveData();
        }

        Debug.Log("Training data generation completed.");
    }

    private void RandomizeLights()
    {
        // // randomly disable or enable lights
        // foreach (Transform light in lights)
        // {
        //     light.gameObject.SetActive(Random.Range(0, 4) != 0);
        // }

        // random rotation
        lights.rotation = Quaternion.Euler(Random.Range(-10, 10), Random.Range(0, 360), Random.Range(-10, 10));
    }

    private void RandomizePostProcessingVolume()
    {
        Volume volume = postProcessingVolume.GetComponent<Volume>();
        volume.profile.TryGet<Bloom>(out Bloom bloom);
        // set bloom thresh
        float bloomThresh = Random.Range(0.75f, 1.1f);
        bloom.threshold.value = bloomThresh;

        // randomize shadows midtones highlights
        volume.profile.TryGet<ShadowsMidtonesHighlights>(out ShadowsMidtonesHighlights colorAdjustments);
        const float MAX_COLOR_ADJUSTMENT = 0.2f;
        colorAdjustments.shadows.value = new Vector4(
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            0
        );
        colorAdjustments.midtones.value = new Vector4(
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            0
        );
        colorAdjustments.highlights.value = new Vector4(
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            1.0f - Random.Range(0, MAX_COLOR_ADJUSTMENT),
            0
        );
    }

    private void RandomizeRobotPosition()
    {
        float x = Random.Range(movementBoundsX.x, movementBoundsX.y);
        float y = Random.Range(movementBoundsY.x, movementBoundsY.y);
        robotTransform.position = new Vector3(x, robotTransform.position.y, y);
    }

    private void RandomizeRobotRotation()
    {
        float rotationY = Random.Range(0f, 360f);
        robotTransform.rotation = Quaternion.Euler(0, rotationY, 0);
    }

    // returns the roi in the camera that the robot is
    private Vector2 GetRobotScreenPos()
    {
        Transform robotBody = robotTransform.Find("Body");
        Vector3 robotPos = robotBody.position;
        // robotPos.y = 0;
        Vector3 robotScreenPos = captureCamera.WorldToScreenPoint(robotPos);
        return new Vector2(robotScreenPos.x, CAMERA_HEIGHT - robotScreenPos.y);
    }

    private void CaptureAndSaveData()
    {
        RenderTexture.active = rt;
        captureCamera.targetTexture = rt;
        captureCamera.Render();


        Vector2 robotScreenPos = GetRobotScreenPos();

        // float startX = Mathf.Clamp(robotScreenPos.x - ROBOT_CROP_SIZE / 2, 0, CAMERA_WIDTH - ROBOT_CROP_SIZE);
        // float startY = Mathf.Clamp(robotScreenPos.y - ROBOT_CROP_SIZE / 2, 0, CAMERA_HEIGHT - ROBOT_CROP_SIZE);

        captureTexture.ReadPixels(new Rect(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT), 0, 0); // new Rect(startX, startY, ROBOT_CROP_SIZE, ROBOT_CROP_SIZE)
        RenderTexture.active = null;

        byte[] imageBytes = captureTexture.EncodeToJPG();
        string imageName = $"image_{currentCaptureIndex}.jpg";
        File.WriteAllBytes(Path.Combine(_imagesSavePath, imageName), imageBytes);
        Debug.Log($"Saved image: {imageName}");
        Debug.Log("Image size: " + imageBytes.Length);

        PositionRotationData data = new PositionRotationData
        {
            position = robotScreenPos,
            rotation = robotTransform.rotation.eulerAngles.y
        };

        string json = JsonUtility.ToJson(data);
        File.WriteAllText(Path.Combine(_labelsSavePath, $"image_{currentCaptureIndex}.json"), json);

        currentCaptureIndex++;
    }

    [System.Serializable]
    public class PositionRotationData
    {
        public Vector2 position;
        public float rotation;
    }
}

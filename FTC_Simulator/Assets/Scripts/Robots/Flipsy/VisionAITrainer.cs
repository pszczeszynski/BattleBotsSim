using UnityEngine;
using System.IO;
using System.Drawing;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using System.Collections;
using System.Collections.Generic;

public class VisionAITrainer : MonoBehaviour
{
    [SerializeField]
    private Transform robotTransform;
    [SerializeField]
    private BoxCollider robotBoundsCollider;
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
    private int CAMERA_WIDTH = 1280;
    [SerializeField]
    private int CAMERA_HEIGHT = 720;
    [SerializeField]
    private string SAVE_PATH_RELATIVE = "../../RobotController/MachineLearning/TrainingData/";
    [SerializeField]
    private bool replaceExisting = true;

    [SerializeField]
    private List<Transform> otherRobots;
    private int currentCaptureIndex = 0;

    private string _imagesSavePath;
    private string _labelsSavePath;


    private Texture2D captureTexture;
    private RenderTexture rt;



    private Vector3 _originalCameraPosition;
    private Quaternion _originalCameraRotation;

    private Transform _orbitronForks;

    private void Start()
    {
        _imagesSavePath = Path.Combine(Application.dataPath, SAVE_PATH_RELATIVE, "TrainingInputs");
        _labelsSavePath = Path.Combine(Application.dataPath, SAVE_PATH_RELATIVE, "TrainingKeys");
        if (!Directory.Exists(_imagesSavePath))
            Directory.CreateDirectory(_imagesSavePath);

        if (!Directory.Exists(_labelsSavePath))
            Directory.CreateDirectory(_labelsSavePath);

        if (!replaceExisting)
        {
            // get max index of existing images
            int maxIndex = -1;
            foreach (string file in Directory.GetFiles(Path.Combine(Application.dataPath, SAVE_PATH_RELATIVE, "TrainingInputs")))
            {
                string fileName = Path.GetFileNameWithoutExtension(file);
                string[] split = fileName.Split('_');
                int index = int.Parse(split[1]);
                if (index > maxIndex)
                    maxIndex = index;
            }

            currentCaptureIndex = maxIndex + 1;

            Debug.Log("Starting with image: " + currentCaptureIndex);
        }

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
        Debug.Log("start: " + captureTexture);
        // save original camera position and rotation
        _originalCameraPosition = captureCamera.transform.position;
        _originalCameraRotation = captureCamera.transform.rotation;


        // Find the "Body" object under orbitron
        Transform _orbitronBody = robotTransform.Find("Body");

        // Find the "Forks" object under the body
        _orbitronForks = _orbitronBody.Find("Forks");

    }

    public void Update()
    {
        if (currentCaptureIndex < numberOfCaptures)
        {
            RandomizeRobotPosition();
            RandomizeCamera();
            RandomizeRobotRotation();
            RandomizeLights();
            RandomizePostProcessingVolume();
            RandomizeOtherRobots();
            RandomizeForks();

            CaptureAndSaveData();
        }
        else if (currentCaptureIndex == numberOfCaptures)
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
        float bloomThresh = Random.Range(0.75f, 2.0f);
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

    private void RandomizeOtherRobots()
    {
        const float MIN_DIST_TO_MAIN_ROBOT = 1.5f;
        
        // move each other robot to a random position + orientation on the field
        // (within movement bounds)
        foreach (Transform otherRobot in otherRobots)
        {
            // keep randomizing until the robot is far enough from the main robot
            float x;
            float y;
            do
            {
                x = Random.Range(movementBoundsX.x, movementBoundsX.y);
                y = Random.Range(movementBoundsY.x, movementBoundsY.y);
            } while (Vector2.Distance(new Vector2(x, y),
                     new Vector2(robotTransform.position.x,
                                 robotTransform.position.z)) < MIN_DIST_TO_MAIN_ROBOT);

            otherRobot.position = new Vector3(x, otherRobot.position.y, y);

            // add random rotation
            otherRobot.rotation = Quaternion.Euler(0, Random.Range(0, 360), 0);

            // randomly enable or disable the robot
            otherRobot.gameObject.SetActive(Random.Range(0, 4) != 0);
        }
    }

    private void RandomizeForks()
    {

        // iterate through all transforms under forks, randomly enable them with 80% probability, disable with 20%
        foreach (Transform fork in _orbitronForks)
        {
            fork.gameObject.SetActive(Random.Range(0, 5) != 0);
        }
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
    private Vector4 GetRobotScreenPos()
    {
        float Xmax = 0, Xmin = int.MaxValue, Ymax = 0, Ymin = int.MaxValue;

        BoxCollider col = robotBoundsCollider;

        var trans = robotBoundsCollider.transform;
        var min = col.center - col.size * 0.5f;
        var max = col.center + col.size * 0.5f;

        ComparePoint(trans.TransformPoint(new Vector3(min.x, min.y, min.z)));
        ComparePoint(trans.TransformPoint(new Vector3(min.x, min.y, max.z)));
        ComparePoint(trans.TransformPoint(new Vector3(min.x, max.y, min.z)));
        ComparePoint(trans.TransformPoint(new Vector3(min.x, max.y, max.z)));
        ComparePoint(trans.TransformPoint(new Vector3(max.x, min.y, min.z)));
        ComparePoint(trans.TransformPoint(new Vector3(max.x, min.y, max.z)));
        ComparePoint(trans.TransformPoint(new Vector3(max.x, max.y, min.z)));
        ComparePoint(trans.TransformPoint(new Vector3(max.x, max.y, max.z)));
        //Debug.DrawLine(trans.TransformPoint(min), trans.TransformPoint(max), UnityEngine.Color.red, 10f);

        void ComparePoint(Vector3 worldPos)
        {
            var screenPos = captureCamera.WorldToScreenPoint(worldPos);
            Xmax = Mathf.Max(Xmax, screenPos.x);
            Xmin = Mathf.Min(Xmin, screenPos.x);
            Ymin = Mathf.Min(Ymin, screenPos.y);
            Ymax = Mathf.Max(Ymax, screenPos.y);
        }

        Debug.Log($"{Xmax} {Xmin} {Ymax} {Ymin}");
        return new Vector4((Xmax+Xmin)/2, CAMERA_HEIGHT - (Ymax+Ymin)/2, Xmax-Xmin, Ymax-Ymin);
    }


    private void CaptureAndSaveData()
    {
        // 1. capture image
        RenderTexture.active = rt;
        captureCamera.targetTexture = rt;
        captureCamera.Render();
        captureTexture.ReadPixels(new Rect(0, 0, CAMERA_WIDTH, CAMERA_HEIGHT), 0, 0);
        RenderTexture.active = null;

        // 2. save image
        byte[] imageBytes = captureTexture.EncodeToJPG();
        string imageName = $"image_{currentCaptureIndex}.jpg";
        File.WriteAllBytes(Path.Combine(_imagesSavePath, imageName), imageBytes);
        Debug.Log($"Saved image: {imageName}");

        // 3. save label
        PositionRotationData data = new PositionRotationData
        {
            position = GetRobotScreenPos(),
            rotation = robotTransform.rotation.eulerAngles.y
        };

        string json = JsonUtility.ToJson(data);
        File.WriteAllText(Path.Combine(_labelsSavePath, $"image_{currentCaptureIndex}.json"), json);

        currentCaptureIndex++;
    }

    [System.Serializable]
    public class PositionRotationData
    {
        public Vector4 position;
        public float rotation;
    }
}
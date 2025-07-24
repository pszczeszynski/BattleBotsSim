using UnityEngine;

[AddComponentMenu("Custom/RandomLocalRotation")]
public class RandomLocalRotation : MonoBehaviour
{
    public enum Axis { X, Y, Z }

    [Header("Rotation Settings")]
    [Tooltip("Which local axis to randomize each frame.")]
    [SerializeField] private Axis rotationAxis = Axis.Y;

    [Tooltip("Minimum local angle (degrees) for the random rotation.")]
    [SerializeField] private float minAngle = 0f;

    [Tooltip("Maximum local angle (degrees) for the random rotation.")]
    [SerializeField] private float maxAngle = 360f;

    void Start()
    {
        // only run if there is an active VisionAITrainer in the scene
        var trainer = GameObject.Find("VisionAITrainer");
        if (trainer == null || !trainer.activeInHierarchy)
        {
            // no valid trainer → disable this component entirely
            enabled = false;
        }
    }

    void Update()
    {
        // Pick a random angle between min and max
        float angle = Random.Range(minAngle, maxAngle);

        // Grab the current local rotation in Euler angles
        Vector3 localEuler = transform.localEulerAngles;

        // Replace the chosen axis with the new random angle
        switch (rotationAxis)
        {
            case Axis.X:
                localEuler.x = angle;
                break;
            case Axis.Y:
                localEuler.y = angle;
                break;
            case Axis.Z:
                localEuler.z = angle;
                break;
        }

        // Apply it back as a local rotation
        transform.localEulerAngles = localEuler;
    }
}

using UnityEngine;
using UnityEngine.Rendering.Universal;

[RequireComponent(typeof(Camera))]
public class FisheyeEffect : MonoBehaviour
{
    [SerializeField]
    private UniversalRenderPipelineAsset urpAsset; // Assign your URP asset here

    private FisheyeRenderFeature feature; // We'll find this manually or via a workaround

    [SerializeField, Range(90f, 180f)]
    private float fieldOfView = 180f;

    [SerializeField, Range(0f, 1f)]
    private float strength = 1f;

    [SerializeField, Range(-1f, 1f)]
    private float centerX = 0f;

    [SerializeField, Range(-1f, 1f)]
    private float centerY = 0f;

    void Start()
    {
        if (urpAsset == null)
        {
            Debug.LogError("URP Asset not assigned! Please assign the UniversalRenderPipelineAsset in the Inspector.");
            return;
        }

        // Attempt to find the FisheyeRenderFeature from the renderer
        var renderer = urpAsset.GetRenderer(0); // Default renderer (e.g., ForwardRenderer)
        if (renderer != null)
        {
            // Since rendererFeatures is protected, we rely on manual setup or assume it's there
            // For now, we'll log a message and require manual connection
            Debug.Log("Please ensure FisheyeRenderFeature is added to the URP Renderer's Renderer Features list.");
        }

        // Note: You’ll need to manually set up the feature in the URP asset as per previous steps
    }

    void Update()
    {
        // If feature is assigned or found, update its settings
        if (feature != null)
        {
            feature.settings.fieldOfView = fieldOfView;
            feature.settings.strength = strength;
            feature.settings.centerX = centerX;
            feature.settings.centerY = centerY;
        }
        else
        {
            // Try to find it manually if not set (this is a fallback)
            FindFeatureManually();
        }
    }

    private void FindFeatureManually()
    {
        if (urpAsset != null && feature == null)
        {
            // This is a hacky workaround since rendererFeatures is protected
            // Ideally, Unity would expose this, but for now, we assume it’s set up
            Debug.LogWarning("FisheyeRenderFeature not assigned. Please assign it manually or ensure it’s in the URP Renderer.");
        }
    }

    // Public method to set the feature (called from elsewhere if needed)
    public void SetFeature(FisheyeRenderFeature fisheyeFeature)
    {
        feature = fisheyeFeature;
        Debug.Log("FisheyeRenderFeature manually assigned.");
    }
}
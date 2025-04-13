using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

public class FisheyeRenderFeature : ScriptableRendererFeature
{
    [System.Serializable]
    public class Settings
    {
        public Shader fisheyeShader;
        [Range(10f, 180f)] public float fieldOfView = 180f;
        [Range(0f, 1f)] public float strength = 1f;
        [Range(-1f, 1f)] public float centerX = 0f;
        [Range(-1f, 1f)] public float centerY = 0f;
        [Range(0, 3)] public int projectionType = 0; // 0: Equidistant, 1: Equisolid, 2: Stereographic, 3: Orthographic
        [Range(0.5f, 3f)] public float fovScale = 1.4f; // Added FOV Scale parameter
    }

    public Settings settings = new Settings();
    private FisheyeRenderPass fisheyePass;

    public override void Create()
    {
        if (settings.fisheyeShader == null)
        {
            Debug.LogError("Fisheye Shader not assigned in FisheyeRenderFeature!");
            return;
        }

        fisheyePass = new FisheyeRenderPass(settings.fisheyeShader, settings.fieldOfView, settings.strength,
            settings.centerX, settings.centerY, settings.projectionType, settings.fovScale);
        fisheyePass.renderPassEvent = RenderPassEvent.BeforeRenderingPostProcessing;
    }

    public override void AddRenderPasses(ScriptableRenderer renderer, ref RenderingData renderingData)
    {
        if (fisheyePass != null)
        {
            fisheyePass.UpdateParameters(settings.fieldOfView, settings.strength, settings.centerX,
                settings.centerY, settings.projectionType, settings.fovScale);
            renderer.EnqueuePass(fisheyePass);
        }
    }
}

public class FisheyeRenderPass : ScriptableRenderPass
{
    private Material fisheyeMaterial;
    private RenderTargetIdentifier source;
    private RenderTargetHandle tempTexture;

    private float fov;
    private float strength;
    private float centerX;
    private float centerY;
    private float projectionType;
    private float fovScale; // Added FOV Scale field

    public FisheyeRenderPass(Shader shader, float fov, float strength, float centerX, float centerY, float projectionType, float fovScale)
    {
        fisheyeMaterial = new Material(shader);
        this.fov = fov;
        this.strength = strength;
        this.centerX = centerX;
        this.centerY = centerY;
        this.projectionType = projectionType;
        this.fovScale = fovScale;

        tempTexture.Init("_TempFisheyeTexture");
    }

    public void UpdateParameters(float fov, float strength, float centerX, float centerY, float projectionType, float fovScale)
    {
        this.fov = fov;
        this.strength = strength;
        this.centerX = centerX;
        this.centerY = centerY;
        this.projectionType = projectionType;
        this.fovScale = fovScale;
    }

    public override void Configure(CommandBuffer cmd, RenderTextureDescriptor cameraTextureDescriptor)
    {
        cmd.GetTemporaryRT(tempTexture.id, cameraTextureDescriptor);
    }

    public override void Execute(ScriptableRenderContext context, ref RenderingData renderingData)
    {
        if (fisheyeMaterial == null)
        {
            Debug.LogWarning("Fisheye material is null, skipping render pass.");
            return;
        }

        CommandBuffer cmd = CommandBufferPool.Get("FisheyeEffect");

        fisheyeMaterial.SetFloat("_FOV", fov);
        fisheyeMaterial.SetFloat("_Strength", strength);
        fisheyeMaterial.SetFloat("_CenterX", centerX);
        fisheyeMaterial.SetFloat("_CenterY", centerY);
        fisheyeMaterial.SetFloat("_ProjectionType", projectionType);
        fisheyeMaterial.SetFloat("_FOVScale", fovScale); // Set the new FOV Scale parameter

        source = renderingData.cameraData.renderer.cameraColorTargetHandle;
        Blit(cmd, source, tempTexture.Identifier(), fisheyeMaterial);
        Blit(cmd, tempTexture.Identifier(), source);

        context.ExecuteCommandBuffer(cmd);
        CommandBufferPool.Release(cmd);
    }

    public override void FrameCleanup(CommandBuffer cmd)
    {
        cmd.ReleaseTemporaryRT(tempTexture.id);
    }
}
Shader "Custom/Fisheye"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _FOV("Field of View", Range(80, 180)) = 180.0
        _Strength("Distortion Strength", Range(0, 1)) = 1.0
        _CenterX("Center X Offset", Range(-1, 1)) = 0.0
        _CenterY("Center Y Offset", Range(-1, 1)) = 0.0
        _ProjectionType("Projection Type", Range(0, 3)) = 0 // 0: Equidistant, 1: Equisolid, 2: Stereographic, 3: Orthographic
        _FOVScale("FOV Scale", Range(1, 3)) = 1.4 // Controls extra FOV sampling
    }
        SubShader
        {
            Tags { "RenderType" = "Opaque" }
            Pass
            {
                CGPROGRAM
                #pragma vertex vert
                #pragma fragment frag
                #include "UnityCG.cginc"

                struct appdata
                {
                    float4 vertex : POSITION;
                    float2 uv : TEXCOORD0;
                };

                struct v2f
                {
                    float2 uv : TEXCOORD0;
                    float4 vertex : SV_POSITION;
                };

                sampler2D _MainTex;
                float _FOV;
                float _Strength;
                float _CenterX;
                float _CenterY;
                float _ProjectionType;
                float _FOVScale;

                v2f vert(appdata v)
                {
                    v2f o;
                    o.vertex = UnityObjectToClipPos(v.vertex);
                    o.uv = v.uv;
                    return o;
                }

                float2 fisheyeUV(float2 uv, float fov, float strength, float centerX, float centerY, float projectionType, float fovScale)
                {
                    // Normalize UVs to [-1, 1] range with aspect correction
                    float2 centeredUV = (uv - 0.5) * 2.0;
                    float aspect = _ScreenParams.x / _ScreenParams.y;
                    centeredUV.x *= aspect; // Adjust for rectangular aspect ratio

                    // Apply center offset
                    centeredUV -= float2(centerX, centerY);

                    // Scale UVs to sample larger FOV
                    float2 scaledUV = centeredUV / fovScale;
                    float r = length(scaledUV);
                    float theta = atan2(scaledUV.y, scaledUV.x);

                    // Original undistorted UVs (adjusted for aspect and offset)
                    float2 undistortedUV = (centeredUV + float2(centerX, centerY)) * 0.5 + 0.5;


                    // Calculate maximum radius based on screen corners
                    float maxR = length(float2(aspect, 1.0)) * fovScale; // Distance to corner with FOV scale
                    float phi = r * radians(fov * 0.5); // No normalization here - use full FOV

                    float rDistorted;
                    // Switch projection type
                    if (projectionType < 0.5) // Equidistant
                    {
                        rDistorted = tan(phi) / tan(radians(fov * 0.5)) * maxR;
                    }
                    else if (projectionType < 1.5) // Equisolid
                    {
                        rDistorted = tan(phi * (2.0 * sin(phi / 2.0))) / tan(radians(fov * 0.5)) * maxR;
                    }
                    else if (projectionType < 2.5) // Stereographic
                    {
                        rDistorted = tan(phi * (2.0 * tan(phi / 2.0))) / tan(radians(fov * 0.5)) * maxR;
                    }
                    else // Orthographic
                    {
                        rDistorted = tan(phi * sin(phi)) / tan(radians(fov * 0.5)) * maxR;
                    }

                    // Blend between distorted and undistorted radius
                    float blendedR = lerp(r, rDistorted, strength);

                    // Convert back to UV space
                    float2 distortedUV = float2(cos(theta), sin(theta)) * blendedR;
                    distortedUV += float2(centerX, centerY); // Reapply center offset
                    distortedUV.x /= aspect; // Correct for aspect ratio
                    float2 finalUV = distortedUV * 0.5 + 0.5;

                    return finalUV;
                }

                fixed4 frag(v2f i) : SV_Target
                {
                    float2 distortedUV = fisheyeUV(i.uv, _FOV, _Strength, _CenterX, _CenterY, _ProjectionType, _FOVScale);
                    fixed4 col = tex2D(_MainTex, distortedUV);
                    return col; // Fill entire screen
                }
                ENDCG
            }
        }
}
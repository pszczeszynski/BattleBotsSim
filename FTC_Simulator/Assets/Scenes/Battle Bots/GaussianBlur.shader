Shader "Custom/GaussianBlur"
{
    Properties
    {
        _MainTex("Texture", 2D) = "white" {}
        _BlurSize("Blur Size", Float) = 0.01
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
                float4 _MainTex_TexelSize;
                float _BlurSize;

                v2f vert(appdata v)
                {
                    v2f o;
                    o.vertex = UnityObjectToClipPos(v.vertex);
                    o.uv = v.uv;
                    return o;
                }

                fixed4 frag(v2f i) : SV_Target
                {
                    fixed4 col = 0;
                    float2 offsets[9] = {
                        float2(-1, -1), float2(0, -1), float2(1, -1),
                        float2(-1, 0),  float2(0, 0),  float2(1, 0),
                        float2(-1, 1),  float2(0, 1),  float2(1, 1)
                    };
                    float kernel[9] = {
                        1 / 16.0, 2 / 16.0, 1 / 16.0,
                        2 / 16.0, 4 / 16.0, 2 / 16.0,
                        1 / 16.0, 2 / 16.0, 1 / 16.0
                    };

                    for (int j = 0; j < 9; j++)
                    {
                        col += tex2D(_MainTex, i.uv + offsets[j] * _BlurSize * _MainTex_TexelSize.xy) * kernel[j];
                    }
                    return col;
                }
                ENDCG
            }
        }
}
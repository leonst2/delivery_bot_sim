// False-colour view of a disparity image (RFloat, pixels) for the HUD camera panel. Maps 0..maxDisparity onto the
// turbo colour map (near = red, far = blue); values <= 0, NaN and Inf (no match) are drawn black.
Shader "Hidden/DeliveryBot/DisparityColormap"
{
    Properties
    {
        [PerRendererData] _MainTex ("Disparity", 2D) = "black" {}
        _MaxDisparity ("Max disparity (px)", Float) = 96
    }
    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "IgnoreProjector" = "True" "CanUseSpriteAtlas" = "False" }
        Cull Off ZWrite Off ZTest [unity_GUIZTestMode]
        Blend SrcAlpha OneMinusSrcAlpha
        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float4 _MainTex_ST;
            float _MaxDisparity;

            struct appdata { float4 vertex : POSITION; float2 uv : TEXCOORD0; float4 color : COLOR; };
            struct v2f { float4 pos : SV_POSITION; float2 uv : TEXCOORD0; float4 color : COLOR; };

            v2f vert(appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _MainTex);
                o.color = v.color;
                return o;
            }

            // Polynomial approximation of Google's turbo colour map.
            float3 Turbo(float t)
            {
                const float4 kR4 = float4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
                const float4 kG4 = float4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
                const float4 kB4 = float4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
                const float2 kR2 = float2(-152.94239396, 59.28637943);
                const float2 kG2 = float2(4.27729857, 2.82956604);
                const float2 kB2 = float2(-89.90310912, 27.34824973);
                t = saturate(t);
                float4 v4 = float4(1.0, t, t * t, t * t * t);
                float2 v2 = v4.zw * v4.z;
                return float3(dot(v4, kR4) + dot(v2, kR2), dot(v4, kG4) + dot(v2, kG2), dot(v4, kB4) + dot(v2, kB2));
            }

            fixed4 frag(v2f i) : SV_Target
            {
                float d = tex2D(_MainTex, i.uv).r;
                // NaN and Inf via the bit pattern: fast-math compilers (Metal) may optimise isnan / NaN compares away.
                bool notFinite = (asuint(d) & 0x7f800000u) == 0x7f800000u;
                if (notFinite || d <= 0) return fixed4(0, 0, 0, i.color.a);
                float3 rgb = Turbo(d / max(_MaxDisparity, 1e-3));
            #if !defined(UNITY_COLORSPACE_GAMMA)
                rgb = GammaToLinearSpace(rgb); // turbo is defined in sRGB; the project renders in linear space
            #endif
                return fixed4(rgb, i.color.a);
            }
            ENDCG
        }
    }
}

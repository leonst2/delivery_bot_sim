// Unlit line for the planned-path overlay (PlanPathOverlay): vertex colour (LineRenderer colours) times an optional
// alpha-clipped texture (dashes). Drawn twice: fully where visible, and faintly where something covers it (for example
// the current step under the robot body, or the path behind a building).
Shader "DeliveryBot/PlanLine"
{
    Properties
    {
        _MainTex ("Texture (alpha = dash)", 2D) = "white" {}
        _Cutoff ("Alpha cutoff", Range(0, 1)) = 0.5
        _HiddenAlpha ("Opacity where covered", Range(0, 1)) = 0.35
    }

    SubShader
    {
        Tags { "Queue" = "Transparent" "RenderType" = "Transparent" "RenderPipeline" = "UniversalPipeline" "IgnoreProjector" = "True" }
        Cull Off
        ZWrite Off
        Blend SrcAlpha OneMinusSrcAlpha

        HLSLINCLUDE
        #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

        TEXTURE2D(_MainTex);
        SAMPLER(sampler_MainTex);

        CBUFFER_START(UnityPerMaterial)
            float4 _MainTex_ST;
            half _Cutoff;
            half _HiddenAlpha;
        CBUFFER_END

        struct Attributes
        {
            float4 positionOS : POSITION;
            half4 color : COLOR;
            float2 uv : TEXCOORD0;
        };

        struct Varyings
        {
            float4 positionCS : SV_POSITION;
            half4 color : COLOR;
            float2 uv : TEXCOORD0;
        };

        Varyings Vert(Attributes input)
        {
            Varyings output;
            output.positionCS = TransformObjectToHClip(input.positionOS.xyz);
            output.color = input.color;
            output.uv = TRANSFORM_TEX(input.uv, _MainTex);
            return output;
        }

        half4 Shade(Varyings input, half opacity)
        {
            half4 tex = SAMPLE_TEXTURE2D(_MainTex, sampler_MainTex, input.uv);
            clip(tex.a - _Cutoff);
            return half4(input.color.rgb * tex.rgb, input.color.a * opacity);
        }
        ENDHLSL

        Pass
        {
            Name "Visible"
            Tags { "LightMode" = "SRPDefaultUnlit" }
            ZTest LEqual
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            half4 Frag(Varyings input) : SV_Target { return Shade(input, 1); }
            ENDHLSL
        }

        Pass
        {
            Name "Covered"
            Tags { "LightMode" = "UniversalForward" }
            ZTest Greater
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            half4 Frag(Varyings input) : SV_Target { return Shade(input, _HiddenAlpha); }
            ENDHLSL
        }
    }
}

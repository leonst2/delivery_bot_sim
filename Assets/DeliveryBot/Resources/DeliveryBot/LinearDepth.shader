// Converts a camera depth buffer into linear depth along the optical axis (metres), for /stereo/left/depth_gt.
// Pixels at the far plane (nothing hit) become +Inf (REP 117). Used by StereoCameraPublisher via Graphics.Blit.
Shader "Hidden/DeliveryBot/LinearDepth"
{
    SubShader
    {
        Cull Off ZWrite Off ZTest Always
        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            UNITY_DECLARE_DEPTH_TEXTURE(_DepthTex);
            float4 _ClipPlanes; // x = near, y = far

            float frag(v2f_img i) : SV_Target
            {
                float d = SAMPLE_DEPTH_TEXTURE(_DepthTex, i.uv);
            #if defined(UNITY_REVERSED_Z)
                d = 1.0 - d;
            #endif
                if (d >= 1.0) return asfloat(0x7f800000); // +Inf
                float n = _ClipPlanes.x;
                float f = _ClipPlanes.y;
                return n * f / (f - d * (f - n));
            }
            ENDCG
        }
    }
}

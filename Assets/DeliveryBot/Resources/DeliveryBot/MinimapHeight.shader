// Colours the top-down minimap by height: the orthographic depth buffer of MinimapPanel's camera becomes the height of
// the topmost surface (Unity Y), shaded in bands that match the static map export (drivable < 0.08 m, obstacle band
// up to 0.6 m, buildings above), with dark outlines where the height jumps. Used once per city render via
// Graphics.Blit, so it costs nothing per frame.
Shader "Hidden/DeliveryBot/MinimapHeight"
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
            float4 _DepthTex_TexelSize;
            float4 _ClipPlanes;   // x = near, y = far, z = camera height (Unity Y)
            float4 _Bands;        // x = obstacle min, y = obstacle max, z = tallest height for the building ramp
            float4 _Background;
            float4 _Ground;
            float4 _Curb;
            float4 _Obstacle;
            float4 _BuildingLow;
            float4 _BuildingHigh;
            float4 _Edge;

            // Height (Unity Y) of the surface seen at uv; very low where nothing was hit.
            float HeightAt(float2 uv)
            {
                float d = SAMPLE_DEPTH_TEXTURE(_DepthTex, uv);
            #if defined(UNITY_REVERSED_Z)
                d = 1.0 - d;
            #endif
                if (d >= 1.0) return -1e4;
                // Orthographic: depth is linear between the clip planes.
                float distance = lerp(_ClipPlanes.x, _ClipPlanes.y, d);
                return _ClipPlanes.z - distance;
            }

            float4 frag(v2f_img i) : SV_Target
            {
                float h = HeightAt(i.uv);
                if (h < -1e3) return _Background;

                float4 c;
                if (h < 0.02) c = _Ground;
                else if (h < _Bands.x) c = _Curb;
                else if (h <= _Bands.y) c = _Obstacle;
                else c = lerp(_BuildingLow, _BuildingHigh, saturate((h - _Bands.y) / max(_Bands.z - _Bands.y, 0.01)));

                // Outline where the height jumps by more than a curb between neighbouring pixels.
                float2 t = _DepthTex_TexelSize.xy;
                float jump = max(max(abs(HeightAt(i.uv + float2(t.x, 0)) - h), abs(HeightAt(i.uv - float2(t.x, 0)) - h)),
                                 max(abs(HeightAt(i.uv + float2(0, t.y)) - h), abs(HeightAt(i.uv - float2(0, t.y)) - h)));
                float edge = saturate((jump - 0.04) * 10.0);
                c.rgb = lerp(c.rgb, _Edge.rgb, edge * _Edge.a);
                c.a = 1.0;
                return c;
            }
            ENDCG
        }
    }
}

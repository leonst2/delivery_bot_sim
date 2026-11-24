using System;
using UnityEngine;
using UnityEngine.UI;

namespace DeliveryBot
{
    /// <summary>
    /// Robot-centred polar plot of a 2D scan, drawn as one UI mesh (no GameObject per point). Plot coordinates are
    /// metres in base_link (x forward, y left); forward is up. The mesh is rebuilt only when new data is set, so a 10 Hz
    /// scan costs ten rebuilds a second.
    /// </summary>
    [RequireComponent(typeof(CanvasRenderer))]
    public class LidarPlotGraphic : MaskableGraphic
    {
        /// <summary>A tracked obstacle in base_link.</summary>
        public struct Obstacle
        {
            public Vector2 position; // m
            public Vector2 velocity; // m/s
            public float radius;     // m
        }

        [Header("Scale")]
        public float maxRange = 12f;
        public float ringStep = 2f;

        [Header("Robot (base_link, m)")]
        [Tooltip("Outline drawn for the robot: front and rear extent from base_link along x, and half the width. "
                 + "Defaults: the URDF's body collision box (0.66 × 0.54 m centred at x = 0.245).")]
        public float footprintFront = 0.575f;
        public float footprintRear = 0.085f;
        public float footprintHalfWidth = 0.27f;

        [Header("Style")]
        public float pointSize = 3.5f;
        public float lineWidth = 1.5f;
        [Tooltip("Velocity arrows show this many seconds of travel.")]
        public float arrowSeconds = 1f;
        public Color ringColor = new Color(1f, 1f, 1f, 0.18f);
        public Color majorRingColor = new Color(1f, 1f, 1f, 0.35f);
        public Color robotColor = new Color(1f, 1f, 1f, 0.9f);
        public Color obstacleColor = new Color(1f, 0.25f, 0.85f, 1f);

        const int k_RingSegments = 64;
        const int k_ObstacleSegments = 24;

        static Color32[] s_RangeLut;

        // Data, referenced (not copied); the owner keeps it alive and calls the setters after changing it.
        float[] m_Angles = Array.Empty<float>();
        float[] m_Ranges = Array.Empty<float>();
        int m_Count;
        Vector2 m_LidarOffset;
        Obstacle[] m_Obstacles = Array.Empty<Obstacle>();
        int m_ObstacleCount;
        Vector2[] m_Zone = Array.Empty<Vector2>();
        int m_ZoneCount;
        Color m_ZoneColor = Color.white;

        Vector2 m_Centre;
        float m_Scale;

        public void SetScan(float[] angles, float[] ranges, int count, Vector2 lidarOffset)
        {
            m_Angles = angles;
            m_Ranges = ranges;
            m_Count = count;
            m_LidarOffset = lidarOffset;
            SetVerticesDirty();
        }

        public void SetObstacles(Obstacle[] obstacles, int count)
        {
            m_Obstacles = obstacles;
            m_ObstacleCount = count;
            SetVerticesDirty();
        }

        public void SetStopZone(Vector2[] polygon, int count, Color color)
        {
            m_Zone = polygon;
            m_ZoneCount = count;
            m_ZoneColor = color;
            SetVerticesDirty();
        }

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();
            Rect rect = GetPixelAdjustedRect();
            m_Centre = rect.center;
            m_Scale = 0.5f * Mathf.Min(rect.width, rect.height) / Mathf.Max(0.1f, maxRange);
            if (m_Scale <= 0f) return;

            // 1. Range rings (every ringStep, brighter every 3rd) and the forward axis.
            for (float r = ringStep; r <= maxRange + 1e-3f; r += ringStep)
            {
                bool major = Mathf.Abs(r / (3f * ringStep) - Mathf.Round(r / (3f * ringStep))) < 1e-3f;
                AddCircle(vh, Vector2.zero, r, k_RingSegments, major ? majorRingColor : ringColor);
            }
            AddLine(vh, ToUi(Vector2.zero), ToUi(new Vector2(maxRange, 0f)), lineWidth, ringColor);

            // 2. Robot footprint and lidar position.
            var fl = new Vector2(footprintFront, footprintHalfWidth);
            var fr = new Vector2(footprintFront, -footprintHalfWidth);
            var rr = new Vector2(-footprintRear, -footprintHalfWidth);
            var rl = new Vector2(-footprintRear, footprintHalfWidth);
            AddLine(vh, ToUi(fl), ToUi(fr), lineWidth, robotColor);
            AddLine(vh, ToUi(fr), ToUi(rr), lineWidth, robotColor);
            AddLine(vh, ToUi(rr), ToUi(rl), lineWidth, robotColor);
            AddLine(vh, ToUi(rl), ToUi(fl), lineWidth, robotColor);
            AddQuad(vh, ToUi(m_LidarOffset), pointSize + 1f, robotColor);

            // 3. Scan points, coloured by range.
            var lut = RangeLut();
            for (int i = 0; i < m_Count; i++)
            {
                float r = m_Ranges[i];
                if (!(r > 0f) || float.IsInfinity(r)) continue; // also rejects NaN
                float a = m_Angles[i];
                var p = m_LidarOffset + r * new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                int idx = Mathf.Clamp((int)(r / maxRange * (lut.Length - 1)), 0, lut.Length - 1);
                AddQuad(vh, ToUi(p), pointSize, lut[idx]);
            }

            // 4. Safety stop zone.
            for (int i = 0; i < m_ZoneCount; i++)
                AddLine(vh, ToUi(m_Zone[i]), ToUi(m_Zone[(i + 1) % m_ZoneCount]), lineWidth + 0.5f, m_ZoneColor);

            // 5. Tracked obstacles with velocity arrows.
            for (int i = 0; i < m_ObstacleCount; i++)
            {
                var o = m_Obstacles[i];
                AddCircle(vh, o.position, Mathf.Max(o.radius, 0.1f), k_ObstacleSegments, obstacleColor);
                Vector2 tip = o.position + o.velocity * arrowSeconds;
                if ((tip - o.position).sqrMagnitude < 1e-4f) continue;
                Vector2 a = ToUi(o.position), b = ToUi(tip);
                AddLine(vh, a, b, lineWidth + 0.5f, obstacleColor);
                Vector2 back = (a - b).normalized * 8f;
                Vector2 side = new Vector2(-back.y, back.x) * 0.5f;
                AddLine(vh, b, b + back + side, lineWidth + 0.5f, obstacleColor);
                AddLine(vh, b, b + back - side, lineWidth + 0.5f, obstacleColor);
            }
        }

        /// <summary>base_link (x forward, y left, m) to local UI pixels: forward up, left left.</summary>
        Vector2 ToUi(Vector2 p) => m_Centre + m_Scale * new Vector2(-p.y, p.x);

        void AddCircle(VertexHelper vh, Vector2 centre, float radius, int segments, Color32 color)
        {
            Vector2 prev = ToUi(centre + new Vector2(radius, 0f));
            for (int s = 1; s <= segments; s++)
            {
                float a = s * 2f * Mathf.PI / segments;
                Vector2 next = ToUi(centre + radius * new Vector2(Mathf.Cos(a), Mathf.Sin(a)));
                AddLine(vh, prev, next, lineWidth, color);
                prev = next;
            }
        }

        static void AddLine(VertexHelper vh, Vector2 a, Vector2 b, float width, Color32 color)
        {
            Vector2 d = b - a;
            float len = d.magnitude;
            if (len < 1e-4f) return;
            Vector2 n = new Vector2(-d.y, d.x) * (0.5f * width / len);
            int i = vh.currentVertCount;
            vh.AddVert(a - n, color, Vector4.zero);
            vh.AddVert(a + n, color, Vector4.zero);
            vh.AddVert(b + n, color, Vector4.zero);
            vh.AddVert(b - n, color, Vector4.zero);
            vh.AddTriangle(i, i + 1, i + 2);
            vh.AddTriangle(i + 2, i + 3, i);
        }

        static void AddQuad(VertexHelper vh, Vector2 c, float size, Color32 color)
        {
            float h = 0.5f * size;
            int i = vh.currentVertCount;
            vh.AddVert(new Vector3(c.x - h, c.y - h), color, Vector4.zero);
            vh.AddVert(new Vector3(c.x - h, c.y + h), color, Vector4.zero);
            vh.AddVert(new Vector3(c.x + h, c.y + h), color, Vector4.zero);
            vh.AddVert(new Vector3(c.x + h, c.y - h), color, Vector4.zero);
            vh.AddTriangle(i, i + 1, i + 2);
            vh.AddTriangle(i + 2, i + 3, i);
        }

        /// <summary>Turbo colour map (polynomial fit), near = blue to far = red, as 256 sRGB entries.</summary>
        static Color32[] RangeLut()
        {
            if (s_RangeLut != null) return s_RangeLut;
            s_RangeLut = new Color32[256];
            for (int i = 0; i < 256; i++)
            {
                float x = i / 255f;
                float r = 0.13572138f + x * (4.61539260f + x * (-42.66032258f + x * (132.13108234f + x * (-152.94239396f + x * 59.28637943f))));
                float g = 0.09140261f + x * (2.19418839f + x * (4.84296658f + x * (-14.18503333f + x * (4.27729857f + x * 2.82956604f))));
                float b = 0.10667330f + x * (12.64194608f + x * (-60.58204836f + x * (110.36276771f + x * (-89.90310912f + x * 27.34824973f))));
                s_RangeLut[i] = new Color(Mathf.Clamp01(r), Mathf.Clamp01(g), Mathf.Clamp01(b), 1f);
            }
            return s_RangeLut;
        }
    }
}

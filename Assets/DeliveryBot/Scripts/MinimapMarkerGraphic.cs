using System;
using DeliveryBot.Minimap;
using UnityEngine;
using UnityEngine.UI;

namespace DeliveryBot
{
    /// <summary>
    /// Everything the minimap draws on top of the city image, as one UI mesh: the plan, the goal, the cyclists, the
    /// stack's estimated pose (with its 1σ circle) and the robot's true pose. Positions are map metres; the graphic
    /// maps them through <see cref="View"/> onto its rect. Icon sizes are in canvas units, so they stay readable however
    /// large the city is. The owner sets the data and calls <see cref="Graphic.SetVerticesDirty"/> when it changed.
    /// </summary>
    [RequireComponent(typeof(CanvasRenderer))]
    public class MinimapMarkerGraphic : MaskableGraphic
    {
        public struct Pose2D
        {
            public Vector2 position; // m, map
            public float yaw;        // rad, CCW from map +x
        }

        public struct Cyclist
        {
            public Vector2 position; // m, map
            public float yaw;        // rad, direction of travel
            public float radius;     // m
        }

        [Header("Sizes (canvas units)")]
        public float robotIconSize = 16f;
        public float goalIconSize = 12f;
        public float cyclistMinSize = 5f;
        public float lineWidth = 2f;

        [Header("Colours")]
        public Color truePoseColor = new Color(1f, 1f, 1f, 1f);
        public Color estimatedPoseColor = new Color(1f, 0.25f, 0.85f, 1f);
        public Color sigmaColor = new Color(1f, 0.25f, 0.85f, 0.45f);
        public Color planColor = new Color(0.1f, 0.85f, 1f, 0.95f);
        public Color goalColor = new Color(0.2f, 1f, 0.35f, 1f);
        public Color pendingGoalColor = new Color(0.2f, 1f, 0.35f, 0.55f);
        public Color cyclistColor = new Color(1f, 0.55f, 0.05f, 1f);

        const int k_CircleSegments = 24;

        public MinimapView View { get; set; } = new MinimapView(new Vector2(-50, -50), new Vector2(100, 100));

        // Data, referenced (not copied); the owner keeps it alive.
        public bool HasTruePose { get; set; }
        public Pose2D TruePose { get; set; }
        public bool HasEstimatedPose { get; set; }
        public Pose2D EstimatedPose { get; set; }
        public float EstimatedSigma { get; set; }
        public bool HasGoal { get; set; }
        public Pose2D Goal { get; set; }
        /// <summary>A goal being placed with the mouse (drawn fainter than the published goal).</summary>
        public bool HasPendingGoal { get; set; }
        public Pose2D PendingGoal { get; set; }

        Vector2[] m_Plan = Array.Empty<Vector2>();
        int m_PlanCount;
        Cyclist[] m_Cyclists = Array.Empty<Cyclist>();
        int m_CyclistCount;

        Rect m_Rect;

        public void SetPlan(Vector2[] points, int count)
        {
            m_Plan = points ?? Array.Empty<Vector2>();
            m_PlanCount = Mathf.Min(count, m_Plan.Length);
        }

        public void SetCyclists(Cyclist[] cyclists, int count)
        {
            m_Cyclists = cyclists ?? Array.Empty<Cyclist>();
            m_CyclistCount = Mathf.Min(count, m_Cyclists.Length);
        }

        Vector2 ToLocal(Vector2 map)
        {
            Vector2 n = View.MapToNormalized(map);
            return new Vector2(m_Rect.xMin + n.x * m_Rect.width, m_Rect.yMin + n.y * m_Rect.height);
        }

        float MetresToLocal(float metres) => View.size.x > 0f ? metres * m_Rect.width / View.size.x : 0f;

        protected override void OnPopulateMesh(VertexHelper vh)
        {
            vh.Clear();
            m_Rect = GetPixelAdjustedRect();
            if (m_Rect.width <= 0f || m_Rect.height <= 0f) return;

            // Back to front: plan, goal, cyclists, estimated pose, true pose.
            Color32 plan = planColor;
            for (int i = 1; i < m_PlanCount; i++)
                AddLine(vh, ToLocal(m_Plan[i - 1]), ToLocal(m_Plan[i]), lineWidth, plan);

            if (HasGoal) AddGoal(vh, Goal, goalColor);
            if (HasPendingGoal) AddGoal(vh, PendingGoal, pendingGoalColor);

            Color32 cyclist = cyclistColor;
            for (int i = 0; i < m_CyclistCount; i++)
            {
                var c = m_Cyclists[i];
                Vector2 p = ToLocal(c.position);
                float r = Mathf.Max(MetresToLocal(c.radius), 0.5f * cyclistMinSize);
                AddDisc(vh, p, r, cyclist);
                Vector2 dir = new Vector2(Mathf.Cos(c.yaw), Mathf.Sin(c.yaw));
                AddLine(vh, p, p + dir * (r + 0.6f * cyclistMinSize), 0.75f * lineWidth, cyclist);
            }

            if (HasEstimatedPose)
            {
                Vector2 p = ToLocal(EstimatedPose.position);
                float sigma = MetresToLocal(EstimatedSigma);
                if (sigma > 1f) AddRing(vh, p, sigma, 0.75f * lineWidth, sigmaColor);
                // Outlined arrow, so the true pose underneath stays visible when both coincide.
                AddArrowOutline(vh, p, EstimatedPose.yaw, robotIconSize * 1.25f, lineWidth, estimatedPoseColor);
            }

            if (HasTruePose)
                AddArrow(vh, ToLocal(TruePose.position), TruePose.yaw, robotIconSize, truePoseColor);
        }

        // ---------------------------------------------------------------- shapes

        void AddGoal(VertexHelper vh, Pose2D goal, Color color)
        {
            Vector2 p = ToLocal(goal.position);
            float h = 0.5f * goalIconSize;
            Color32 c = color;
            AddRing(vh, p, h, lineWidth, c);
            AddLine(vh, p + new Vector2(-h, -h) * 0.6f, p + new Vector2(h, h) * 0.6f, lineWidth, c);
            AddLine(vh, p + new Vector2(-h, h) * 0.6f, p + new Vector2(h, -h) * 0.6f, lineWidth, c);
            Vector2 dir = new Vector2(Mathf.Cos(goal.yaw), Mathf.Sin(goal.yaw));
            AddLine(vh, p + dir * h, p + dir * (h + goalIconSize), lineWidth, c);
        }

        static Vector2[] ArrowCorners(Vector2 p, float yaw, float size)
        {
            Vector2 f = new Vector2(Mathf.Cos(yaw), Mathf.Sin(yaw));
            Vector2 l = new Vector2(-f.y, f.x);
            return new[]
            {
                p + f * (0.6f * size),
                p - f * (0.4f * size) + l * (0.4f * size),
                p - f * (0.15f * size),
                p - f * (0.4f * size) - l * (0.4f * size),
            };
        }

        static void AddArrow(VertexHelper vh, Vector2 p, float yaw, float size, Color32 color)
        {
            var c = ArrowCorners(p, yaw, size);
            int i = vh.currentVertCount;
            foreach (var v in c) vh.AddVert(v, color, Vector4.zero);
            vh.AddTriangle(i, i + 1, i + 2);
            vh.AddTriangle(i, i + 2, i + 3);
        }

        static void AddArrowOutline(VertexHelper vh, Vector2 p, float yaw, float size, float width, Color32 color)
        {
            var c = ArrowCorners(p, yaw, size);
            for (int k = 0; k < c.Length; k++)
                AddLine(vh, c[k], c[(k + 1) % c.Length], width, color);
        }

        static void AddDisc(VertexHelper vh, Vector2 centre, float radius, Color32 color)
        {
            int i0 = vh.currentVertCount;
            vh.AddVert(centre, color, Vector4.zero);
            for (int k = 0; k < k_CircleSegments; k++)
            {
                float a = k * 2f * Mathf.PI / k_CircleSegments;
                vh.AddVert(centre + radius * new Vector2(Mathf.Cos(a), Mathf.Sin(a)), color, Vector4.zero);
            }
            for (int k = 0; k < k_CircleSegments; k++)
                vh.AddTriangle(i0, i0 + 1 + k, i0 + 1 + (k + 1) % k_CircleSegments);
        }

        static void AddRing(VertexHelper vh, Vector2 centre, float radius, float width, Color32 color)
        {
            Vector2 prev = centre + new Vector2(radius, 0f);
            for (int k = 1; k <= k_CircleSegments; k++)
            {
                float a = k * 2f * Mathf.PI / k_CircleSegments;
                Vector2 next = centre + radius * new Vector2(Mathf.Cos(a), Mathf.Sin(a));
                AddLine(vh, prev, next, width, color);
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
    }
}

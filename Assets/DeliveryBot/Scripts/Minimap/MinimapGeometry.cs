using System;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot.Minimap
{
    /// <summary>
    /// The part of the map frame the minimap shows: an axis-aligned rectangle in map metres (ROS FLU, origin = Unity
    /// world origin, #15), drawn with map +x to the right and map +y up, like RViz with the default top-down view.
    /// Normalised view coordinates are (0, 0) at the bottom-left corner and (1, 1) at the top-right corner.
    /// </summary>
    [Serializable]
    public struct MinimapView
    {
        public Vector2 min;  // m, map frame
        public Vector2 size; // m

        public MinimapView(Vector2 min, Vector2 size)
        {
            this.min = min;
            this.size = size;
        }

        public Vector2 Max => min + size;
        public Vector2 Center => min + 0.5f * size;
        public float Aspect => size.y > 0f ? size.x / size.y : 1f;

        public Vector2 MapToNormalized(Vector2 map) =>
            new Vector2((map.x - min.x) / size.x, (map.y - min.y) / size.y);

        public Vector2 NormalizedToMap(Vector2 normalized) =>
            new Vector2(min.x + normalized.x * size.x, min.y + normalized.y * size.y);

        public bool Contains(Vector2 map) =>
            map.x >= min.x && map.y >= min.y && map.x <= min.x + size.x && map.y <= min.y + size.y;

        /// <summary>
        /// The view that shows <paramref name="boundsMin"/>…<paramref name="boundsMax"/> plus
        /// <paramref name="padding"/> metres on every side, widened symmetrically to <paramref name="aspect"/>
        /// (width / height) if one is given.
        /// </summary>
        public static MinimapView Fit(Vector2 boundsMin, Vector2 boundsMax, float padding, float aspect = 0f)
        {
            Vector2 min = Vector2.Min(boundsMin, boundsMax) - Vector2.one * padding;
            Vector2 size = Vector2.Max(Vector2.Max(boundsMin, boundsMax) + Vector2.one * padding - min, Vector2.one * 1e-3f);
            if (aspect > 0f)
            {
                float current = size.x / size.y;
                if (current < aspect)
                {
                    float grow = size.y * aspect - size.x;
                    min.x -= 0.5f * grow;
                    size.x += grow;
                }
                else if (current > aspect)
                {
                    float grow = size.x / aspect - size.y;
                    min.y -= 0.5f * grow;
                    size.y += grow;
                }
            }
            return new MinimapView(min, size);
        }
    }

    /// <summary>Conversions between Unity world space and the 2D map frame, and small pose helpers.</summary>
    public static class MinimapGeometry
    {
        /// <summary>
        /// Rotation of the top-down minimap camera: looking straight down with map +x (Unity +Z) to the right of the
        /// image and map +y (Unity −X) at the top.
        /// </summary>
        public static Quaternion TopDownRotation => Quaternion.LookRotation(Vector3.down, Vector3.left);

        /// <summary>
        /// Makes <paramref name="cam"/> an orthographic top-down camera whose image covers exactly
        /// <paramref name="view"/> (viewport (0, 0) = view min, (1, 1) = view max), above everything between
        /// <paramref name="bottomY"/> and <paramref name="topY"/> (Unity Y).
        /// </summary>
        public static void PlaceTopDownCamera(Camera cam, MinimapView view, float topY, float bottomY)
        {
            float height = topY + 10f;
            cam.orthographic = true;
            cam.transform.SetPositionAndRotation(MapToUnity(view.Center, height), TopDownRotation);
            cam.orthographicSize = 0.5f * view.size.y;
            cam.aspect = view.Aspect;
            cam.nearClipPlane = 0.1f;
            cam.farClipPlane = Mathf.Max(1f, height - bottomY + 10f);
        }

        /// <summary>A Unity world point as map (x, y). The map frame is the Unity world origin with ROS axes.</summary>
        public static Vector2 UnityToMap(Vector3 world)
        {
            Vector3<FLU> flu = world.To<FLU>();
            return new Vector2(flu.x, flu.y);
        }

        /// <summary>A map (x, y) point as a Unity world point at height <paramref name="unityY"/>.</summary>
        public static Vector3 MapToUnity(Vector2 map, float unityY = 0f)
        {
            Vector3 world = new Vector3<FLU>(map.x, map.y, 0f).toUnity;
            world.y = unityY;
            return world;
        }

        /// <summary>Yaw in the map frame (rad, counter-clockwise from +x) of a Unity world direction.</summary>
        public static float UnityDirectionToYaw(Vector3 direction)
        {
            Vector3<FLU> flu = direction.To<FLU>();
            return Mathf.Atan2(flu.y, flu.x);
        }

        /// <summary>Yaw (rad) of a ROS quaternion (rotation about z only is used).</summary>
        public static float YawFromQuaternion(double x, double y, double z, double w) =>
            (float)Math.Atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

        /// <summary>ROS quaternion (x, y, z, w) of a pure yaw rotation.</summary>
        public static (double x, double y, double z, double w) QuaternionFromYaw(double yaw) =>
            (0.0, 0.0, Math.Sin(0.5 * yaw), Math.Cos(0.5 * yaw));

        /// <summary>
        /// Heading of a goal: towards the drag point if the drag is longer than <paramref name="minDrag"/> metres,
        /// otherwise from <paramref name="robot"/> towards the goal, otherwise 0. Deterministic for the same input.
        /// </summary>
        public static float GoalYaw(Vector2 goal, Vector2? dragTo, Vector2? robot, float minDrag)
        {
            if (dragTo.HasValue && (dragTo.Value - goal).sqrMagnitude >= minDrag * minDrag)
                return Mathf.Atan2(dragTo.Value.y - goal.y, dragTo.Value.x - goal.x);
            if (robot.HasValue && (goal - robot.Value).sqrMagnitude > 1e-6f)
                return Mathf.Atan2(goal.y - robot.Value.y, goal.x - robot.Value.x);
            return 0f;
        }

        /// <summary>1σ radius (m) of a 6×6 pose covariance (row-major): the larger of the x and y standard deviations.</summary>
        public static float PositionSigma(double[] covariance)
        {
            if (covariance == null || covariance.Length < 8) return 0f;
            double v = Math.Max(covariance[0], covariance[7]);
            return v > 0 && !double.IsNaN(v) && !double.IsInfinity(v) ? (float)Math.Sqrt(v) : 0f;
        }
    }

    /// <summary>
    /// Colours of an occupancy grid / costmap cell (nav_msgs/OccupancyGrid: 0 = free … 100 = lethal, −1 = unknown).
    /// Free cells are transparent so the city shows through; costs fade from blue to yellow; lethal is red.
    /// </summary>
    public static class CostmapPalette
    {
        public static readonly Color32 Free = new Color32(0, 0, 0, 0);
        public static readonly Color32 Unknown = new Color32(128, 128, 128, 90);
        public static readonly Color32 Lethal = new Color32(255, 40, 40, 220);
        public static readonly Color32 Invalid = new Color32(255, 0, 255, 160);

        /// <summary>Lookup table indexed by the cell value as a byte ((byte)(sbyte)v).</summary>
        public static Color32[] BuildLut()
        {
            var lut = new Color32[256];
            for (int i = 0; i < 256; i++) lut[i] = ColorOf((sbyte)(byte)i);
            return lut;
        }

        public static Color32 ColorOf(sbyte value)
        {
            if (value == 0) return Free;
            if (value == -1) return Unknown;
            if (value >= 100) return Lethal;
            if (value < 0) return Invalid;
            // 1..99: inflation / cost gradient, blue (low) → yellow (high), more opaque with higher cost.
            float t = (value - 1) / 98f;
            byte r = (byte)Mathf.RoundToInt(Mathf.Lerp(40f, 255f, t));
            byte g = (byte)Mathf.RoundToInt(Mathf.Lerp(120f, 220f, t));
            byte b = (byte)Mathf.RoundToInt(Mathf.Lerp(255f, 40f, t));
            byte a = (byte)Mathf.RoundToInt(Mathf.Lerp(70f, 190f, t));
            return new Color32(r, g, b, a);
        }

        /// <summary>
        /// Writes one pixel per cell. OccupancyGrid row 0 is the lowest y and Unity texture row 0 is the bottom row,
        /// so the data maps 1:1 without flipping.
        /// </summary>
        public static void Fill(sbyte[] cells, Color32[] lut, Unity.Collections.NativeArray<Color32> pixels)
        {
            int n = Math.Min(cells.Length, pixels.Length);
            for (int i = 0; i < n; i++) pixels[i] = lut[(byte)cells[i]];
            for (int i = n; i < pixels.Length; i++) pixels[i] = Invalid;
        }
    }
}

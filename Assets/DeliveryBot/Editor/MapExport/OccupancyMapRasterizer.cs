using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot.MapExport
{
    /// <summary>Axis-aligned rectangle in the ROS map frame (FLU, metres).</summary>
    public struct MapRect
    {
        public double MinX, MinY, MaxX, MaxY;

        public MapRect(double minX, double minY, double maxX, double maxY)
        {
            MinX = minX; MinY = minY; MaxX = maxX; MaxY = maxY;
        }

        public override string ToString() => $"x [{MinX:0.##}, {MaxX:0.##}], y [{MinY:0.##}, {MaxY:0.##}]";
    }

    [Serializable]
    public class RasterSettings
    {
        public const string RobotLayer = "Robot";
        public const string DynamicLayer = "Dynamic";

        /// <summary>
        /// Cell size in metres. A double on purpose: 0.05f is 0.0500000007, which shifts the origin and adds a
        /// cell at the far edge.
        /// </summary>
        public double Resolution = 0.05;

        /// <summary>
        /// Lower edge of the obstacle band (Unity world Y, ground at 0). Anything that reaches into the band is
        /// occupied. Below it the robot drives over the geometry: the 0.05 m sidewalk curbs must stay free
        /// (wheel radius 0.085 m, chassis underside ≈ 0.075 m).
        /// </summary>
        public float MinObstacleHeight = 0.08f;

        /// <summary>
        /// Upper edge of the obstacle band. Covers the LiDAR plane (0.515 m) and the robot's top (0.52 m),
        /// so low walls below the LiDAR are occupied too. Geometry entirely above it (signs, overhangs) is ignored.
        /// </summary>
        public float MaxObstacleHeight = 0.6f;

        /// <summary>A cell is free only if a downward ray from the band hits ground within this depth; otherwise unknown.</summary>
        public float GroundProbeDepth = 1.0f;

        /// <summary>Layers that are never in the map. Robot and Dynamic are always excluded.</summary>
        public LayerMask ExcludedLayers;

        public int QueryMask()
        {
            int mask = Physics.AllLayers & ~ExcludedLayers.value;
            int robot = LayerMask.NameToLayer(RobotLayer);
            int dynamic = LayerMask.NameToLayer(DynamicLayer);
            if (robot >= 0) mask &= ~(1 << robot);
            if (dynamic >= 0) mask &= ~(1 << dynamic);
            mask &= ~(1 << 2); // Ignore Raycast
            return mask;
        }
    }

    /// <summary>
    /// Rasterises the static colliders of the loaded scenes into an <see cref="OccupancyGrid"/>.
    /// Map frame = Unity world origin, axes ROS FLU (x = Unity +Z, y = Unity −X), converted only through
    /// ROS-TCP-Connector's <c>To&lt;FLU&gt;()</c> / <c>Vector3&lt;FLU&gt;.toUnity</c>.
    ///
    /// Per cell: occupied if any collider intersects the cell's column within the obstacle band
    /// (this covers obstacles at LiDAR height as well as low ones); otherwise free if ground lies below,
    /// else unknown. Deterministic: same scene and settings give the same bytes.
    /// </summary>
    public static class OccupancyMapRasterizer
    {
        // Shrinks the query box so that geometry that only touches a cell boundary doesn't mark the neighbour.
        const float BoundaryEpsilon = 1e-3f;

        /// <summary>Unity world point → ROS map frame.</summary>
        public static Vector3<FLU> ToMap(Vector3 unityWorld) => unityWorld.To<FLU>();

        /// <summary>ROS map frame point → Unity world.</summary>
        public static Vector3 ToUnity(double x, double y, double z) => new Vector3<FLU>((float)x, (float)y, (float)z).toUnity;

        /// <summary>Colliders that end up in the map (enabled, solid, on an included layer), sorted by name path.</summary>
        public static List<Collider> CollectStaticColliders(RasterSettings settings)
        {
            int mask = settings.QueryMask();
            var result = new List<Collider>();
            foreach (var c in UnityEngine.Object.FindObjectsByType<Collider>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
                if (c.enabled && !c.isTrigger && ((1 << c.gameObject.layer) & mask) != 0)
                    result.Add(c);
            result.Sort((a, b) => string.CompareOrdinal(PathOf(a.transform), PathOf(b.transform)));
            return result;
        }

        /// <summary>Colliders that are included but look movable. They should be on the Dynamic layer.</summary>
        public static List<Collider> FindSuspiciousColliders(IEnumerable<Collider> colliders)
        {
            var suspicious = new List<Collider>();
            foreach (var c in colliders)
                if ((c.attachedRigidbody != null && !c.attachedRigidbody.isKinematic) || c.attachedArticulationBody != null)
                    suspicious.Add(c);
            return suspicious;
        }

        /// <summary>Footprint of the colliders in the map frame.</summary>
        public static MapRect FootprintOf(IReadOnlyList<Collider> colliders)
        {
            if (colliders.Count == 0) throw new InvalidOperationException("No static colliders in the loaded scenes.");
            double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
            foreach (var c in colliders)
            {
                Bounds b = c.bounds;
                for (int i = 0; i < 4; i++)
                {
                    var corner = new Vector3((i & 1) == 0 ? b.min.x : b.max.x, 0f, (i & 2) == 0 ? b.min.z : b.max.z);
                    Vector3<FLU> m = ToMap(corner);
                    minX = Math.Min(minX, m.x); maxX = Math.Max(maxX, m.x);
                    minY = Math.Min(minY, m.y); maxY = Math.Max(maxY, m.y);
                }
            }
            return new MapRect(minX, minY, maxX, maxY);
        }

        /// <summary>Grid covering <paramref name="area"/>, snapped outwards to whole cells.</summary>
        public static OccupancyGrid CreateGrid(MapRect area, double resolution)
        {
            long x0 = OccupancyGrid.ToCellIndex(area.MinX, resolution);
            long y0 = OccupancyGrid.ToCellIndex(area.MinY, resolution);
            long x1 = (long)Math.Ceiling(area.MaxX / resolution - 1e-9);
            long y1 = (long)Math.Ceiling(area.MaxY / resolution - 1e-9);
            return new OccupancyGrid((int)Math.Max(1, x1 - x0), (int)Math.Max(1, y1 - y0), resolution, x0, y0);
        }

        /// <param name="progress">Called with 0..1; return true to cancel (the method then returns null).</param>
        public static OccupancyGrid Rasterize(RasterSettings settings, MapRect area, Func<float, bool> progress = null)
        {
            if (settings.MaxObstacleHeight <= settings.MinObstacleHeight)
                throw new ArgumentException("MaxObstacleHeight must be above MinObstacleHeight.");

            Physics.SyncTransforms();
            PhysicsScene physics = Physics.defaultPhysicsScene;
            int mask = settings.QueryMask();
            var grid = CreateGrid(area, settings.Resolution);

            float res = (float)grid.Resolution;
            float bandCenter = 0.5f * (settings.MinObstacleHeight + settings.MaxObstacleHeight);
            // The cell is square, so the half extents don't depend on the axis permutation between FLU and RUF.
            var halfExtents = new Vector3(0.5f * res - BoundaryEpsilon,
                0.5f * (settings.MaxObstacleHeight - settings.MinObstacleHeight), 0.5f * res - BoundaryEpsilon);
            float probeLength = settings.MinObstacleHeight + settings.GroundProbeDepth;
            var overlaps = new Collider[1];

            for (int row = 0; row < grid.Height; row++)
            {
                if (progress != null && (row & 15) == 0 && progress((float)row / grid.Height))
                    return null;
                double y = grid.CellCenterY(row);
                for (int col = 0; col < grid.Width; col++)
                {
                    Vector3 center = ToUnity(grid.CellCenterX(col), y, bandCenter);
                    if (physics.OverlapBox(center, halfExtents, overlaps, Quaternion.identity, mask, QueryTriggerInteraction.Ignore) > 0)
                    {
                        grid[col, row] = OccupancyGrid.Occupied;
                        continue;
                    }
                    var probeStart = new Vector3(center.x, settings.MinObstacleHeight, center.z);
                    bool ground = physics.Raycast(probeStart, Vector3.down, out _, probeLength, mask, QueryTriggerInteraction.Ignore);
                    grid[col, row] = ground ? OccupancyGrid.Free : OccupancyGrid.Unknown;
                }
            }
            progress?.Invoke(1f);
            return grid;
        }

        static string PathOf(Transform t) => t.parent == null ? t.name : PathOf(t.parent) + "/" + t.name;
    }
}

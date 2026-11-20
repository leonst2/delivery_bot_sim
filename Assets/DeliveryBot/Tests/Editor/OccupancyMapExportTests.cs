using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace DeliveryBot.MapExport.Tests
{
    /// <summary>
    /// Frame and content checks for the occupancy map export. Expected cells are worked out by hand from
    /// the FLU convention (map x = Unity +Z, map y = Unity −X), independently of the exporter code.
    ///
    /// Test world: a 10 × 10 m ground slab centred on the origin (top at Unity Y = 0), exported for the map
    /// area x, y ∈ [−5, 5] at 0.05 m → 200 × 200 cells, origin [−5, −5]. Map cell (col, row) covers
    /// x ∈ [−5 + 0.05·col, …), y ∈ [−5 + 0.05·row, …). PGM pixel (px, py) = (col, 199 − row).
    /// Everything sits far from the city so the open scene doesn't interfere: the world is shifted by
    /// Offset in Unity and the map area accordingly.
    /// </summary>
    public class OccupancyMapExportTests
    {
        // Unity offset (x = −500 → map y = +500, z = 1000 → map x = +1000). Whole metres, so cell edges stay exact.
        static readonly Vector3 Offset = new Vector3(-500f, 0f, 1000f);
        const double MapOffsetX = 1000.0, MapOffsetY = 500.0;
        const int N = 200;

        readonly List<GameObject> m_Objects = new List<GameObject>();
        RasterSettings m_Settings;
        MapRect m_Area;

        [SetUp]
        public void SetUp()
        {
            m_Settings = new RasterSettings();
            m_Area = new MapRect(MapOffsetX - 5, MapOffsetY - 5, MapOffsetX + 5, MapOffsetY + 5);
            Box("Ground", new Vector3(0f, -0.05f, 0f), new Vector3(10f, 0.1f, 10f));
        }

        [TearDown]
        public void TearDown()
        {
            foreach (var go in m_Objects) Object.DestroyImmediate(go);
            m_Objects.Clear();
        }

        GameObject Box(string name, Vector3 center, Vector3 size, int layer = 0)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = "MapExportTest_" + name;
            go.layer = layer;
            go.transform.position = center + Offset;
            go.transform.localScale = size;
            m_Objects.Add(go);
            return go;
        }

        OccupancyGrid Rasterize()
        {
            var grid = OccupancyMapRasterizer.Rasterize(m_Settings, m_Area);
            Assert.AreEqual(N, grid.Width);
            Assert.AreEqual(N, grid.Height);
            return grid;
        }

        [Test]
        public void BuildingCorner_AppearsAtExpectedPixel()
        {
            // Building: Unity x ∈ [−3, 1], z ∈ [2, 4], 3 m high.
            // In the map: x = Unity z ∈ [2, 4], y = −Unity x ∈ [−1, 3].
            Box("Building", new Vector3(-1f, 1.5f, 3f), new Vector3(4f, 3f, 2f));
            var grid = Rasterize();

            // Corner at Unity (1, 0, 2) → map (2, −1) → the cell just inside is col (2+5)/0.05 = 140, row (−1+5)/0.05 = 80.
            Assert.AreEqual(OccupancyGrid.Occupied, grid[140, 80], "cell inside the corner");
            Assert.AreEqual(OccupancyGrid.Free, grid[139, 80], "left of the corner (x < 2)");
            Assert.AreEqual(OccupancyGrid.Free, grid[140, 79], "below the corner (y < −1)");
            Assert.AreEqual(OccupancyGrid.Free, grid[139, 79], "diagonal");

            // Opposite corner at Unity (−3, 0, 4) → map (4, 3) → last occupied cell col 179, row 159.
            Assert.AreEqual(OccupancyGrid.Occupied, grid[179, 159]);
            Assert.AreEqual(OccupancyGrid.Free, grid[180, 159]);
            Assert.AreEqual(OccupancyGrid.Free, grid[179, 160]);

            // Exactly 40 × 80 cells occupied, nothing else.
            Assert.AreEqual(40 * 80, grid.Count(OccupancyGrid.Occupied));

            // The same corner in the written PGM: pixel (140, 199 − 80) = (140, 119), image row 0 at the top.
            byte[] pixels = OccupancyGrid.ReadPgmPixels(grid.ToPgm(), out int w, out int h);
            Assert.AreEqual(N, w);
            Assert.AreEqual(N, h);
            Assert.AreEqual(OccupancyGrid.Occupied, pixels[119 * w + 140]);
            Assert.AreEqual(OccupancyGrid.Free, pixels[119 * w + 139]);
            Assert.AreEqual(OccupancyGrid.Free, pixels[120 * w + 140]);
        }

        [Test]
        public void Yaml_OriginIsLowerLeftCornerInMapFrame()
        {
            var grid = Rasterize();
            string yaml = grid.ToYaml("map.pgm");
            StringAssert.Contains("image: map.pgm\n", yaml);
            StringAssert.Contains("resolution: 0.05\n", yaml);
            StringAssert.Contains("origin: [995.0, 495.0, 0.0]\n", yaml);
            StringAssert.Contains("negate: 0\n", yaml);
            StringAssert.Contains("occupied_thresh: 0.65\n", yaml);
            StringAssert.Contains("free_thresh: 0.1\n", yaml);
        }

        [Test]
        public void MapOrigin_IsUnityWorldOrigin()
        {
            var atOrigin = OccupancyMapRasterizer.ToMap(Vector3.zero);
            Assert.AreEqual(0f, atOrigin.x);
            Assert.AreEqual(0f, atOrigin.y);

            // Unity forward (+Z) is map +x, Unity left (−X) is map +y.
            var forward = OccupancyMapRasterizer.ToMap(new Vector3(0f, 0f, 2f));
            var left = OccupancyMapRasterizer.ToMap(new Vector3(-3f, 0f, 0f));
            Assert.AreEqual(2f, forward.x); Assert.AreEqual(0f, forward.y);
            Assert.AreEqual(0f, left.x); Assert.AreEqual(3f, left.y);

            var grid = OccupancyMapRasterizer.CreateGrid(new MapRect(-50, -50, 50, 50), 0.05);
            Assert.AreEqual(2000, grid.Width);
            Assert.AreEqual(-50.0, grid.OriginX, 1e-9);
            Assert.IsTrue(grid.TryGetCell(0.0, 0.0, out int col, out int row));
            Assert.AreEqual(1000, col);
            Assert.AreEqual(1000, row);
        }

        [Test]
        public void LowWallBelowLidar_IsOccupied_CurbIsFree()
        {
            // 0.3 m wall (below the LiDAR plane at 0.515 m): Unity x ∈ [0, 0.5], z ∈ [−2, −1] → map x ∈ [−2, −1], y ∈ [−0.5, 0].
            Box("LowWall", new Vector3(0.25f, 0.15f, -1.5f), new Vector3(0.5f, 0.3f, 1f));
            // 0.05 m curb: Unity x ∈ [−4, −2], z ∈ [−4, −2] → map x ∈ [−4, −2], y ∈ [2, 4].
            Box("Curb", new Vector3(-3f, 0.025f, -3f), new Vector3(2f, 0.05f, 2f));
            var grid = Rasterize();

            Assert.AreEqual(OccupancyGrid.Occupied, grid[70, 95], "low wall (map −1.5, −0.25)");
            Assert.AreEqual(20 * 10, grid.Count(OccupancyGrid.Occupied), "only the wall is occupied");
            Assert.AreEqual(OccupancyGrid.Free, grid[40, 160], "curb top (map −3, 3) is drivable");
        }

        [Test]
        public void DynamicLayer_IsNeverInMap()
        {
            int dynamic = LayerMask.NameToLayer(RasterSettings.DynamicLayer);
            if (dynamic < 0) Assert.Ignore("Layer 'Dynamic' is missing (Tools > Delivery Bot > Setup > Apply project settings).");
            int robot = LayerMask.NameToLayer(RasterSettings.RobotLayer);

            Box("Cyclist", new Vector3(0f, 0.9f, 0f), new Vector3(0.6f, 1.8f, 1.8f), dynamic);
            if (robot >= 0) Box("Robot", new Vector3(2f, 0.3f, 2f), new Vector3(0.6f, 0.6f, 0.6f), robot);
            var grid = Rasterize();

            Assert.AreEqual(0, grid.Count(OccupancyGrid.Occupied));
        }

        [Test]
        public void OutsideGround_IsUnknown()
        {
            m_Area = new MapRect(MapOffsetX - 6, MapOffsetY - 5, MapOffsetX + 5, MapOffsetY + 5);
            var grid = OccupancyMapRasterizer.Rasterize(m_Settings, m_Area);
            Assert.AreEqual(220, grid.Width);
            // Column 0..19 lies at map x ∈ [−6, −5): no ground below.
            Assert.AreEqual(OccupancyGrid.Unknown, grid[0, 100]);
            Assert.AreEqual(OccupancyGrid.Unknown, grid[19, 100]);
            Assert.AreEqual(OccupancyGrid.Free, grid[20, 100]);
        }

        [Test]
        public void Tiles_KeepGlobalCellAlignment()
        {
            var grid = OccupancyMapRasterizer.CreateGrid(new MapRect(-50, -50, 50, 50), 0.05);
            var tile = grid.Crop(400, 800, 400, 400);
            Assert.AreEqual(-30.0, tile.OriginX, 1e-9);
            Assert.AreEqual(-10.0, tile.OriginY, 1e-9);

            string dir = Path.Combine(Path.GetTempPath(), "dbot_map_export_test");
            tile.Save(dir, "tile");
            Assert.IsTrue(File.Exists(Path.Combine(dir, "tile.pgm")));
            StringAssert.Contains("origin: [-30.0, -10.0, 0.0]", File.ReadAllText(Path.Combine(dir, "tile.yaml")));
            Directory.Delete(dir, true);
        }
    }
}

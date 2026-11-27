using DeliveryBot.Minimap;
using NUnit.Framework;
using Unity.Collections;
using UnityEngine;

namespace DeliveryBot.Minimap.Tests
{
    public class MinimapGeometryTests
    {
        const float Cell = 0.05f; // m, resolution of the city map / global costmap

        static readonly MinimapView CityView = MinimapView.Fit(new Vector2(-50f, -50f), new Vector2(50f, 50f), 2f);

        [Test]
        public void MapAxes_FollowRosFlu()
        {
            // map x = Unity +Z (forward), map y = Unity −X (left).
            Assert.That(MinimapGeometry.UnityToMap(new Vector3(0f, 3f, 2f)), Is.EqualTo(new Vector2(2f, 0f)));
            Assert.That(MinimapGeometry.UnityToMap(new Vector3(-4f, 0f, 0f)), Is.EqualTo(new Vector2(0f, 4f)));
            Assert.That(MinimapGeometry.MapToUnity(new Vector2(2f, 4f), 1f), Is.EqualTo(new Vector3(-4f, 1f, 2f)));
            Assert.That(MinimapGeometry.UnityDirectionToYaw(Vector3.forward), Is.EqualTo(0f).Within(1e-6f));
            Assert.That(MinimapGeometry.UnityDirectionToYaw(Vector3.left), Is.EqualTo(Mathf.PI / 2f).Within(1e-6f));
        }

        [Test]
        public void Fit_PadsAndWidensToAspect()
        {
            Assert.That(CityView.min, Is.EqualTo(new Vector2(-52f, -52f)));
            Assert.That(CityView.size, Is.EqualTo(new Vector2(104f, 104f)));

            var wide = MinimapView.Fit(new Vector2(0f, 0f), new Vector2(10f, 10f), 0f, 2f);
            Assert.That(wide.size.x, Is.EqualTo(20f).Within(1e-4f));
            Assert.That(wide.size.y, Is.EqualTo(10f).Within(1e-4f));
            Assert.That(wide.Center, Is.EqualTo(new Vector2(5f, 5f)));
        }

        [Test]
        public void ClickRoundTrip_IsWithinOneCell()
        {
            // A click is turned into normalised view coordinates and back; the goal must land in the clicked cell.
            for (float x = -49.975f; x < 50f; x += 7.3f)
            for (float y = -49.975f; y < 50f; y += 5.9f)
            {
                var map = new Vector2(x, y);
                Vector2 back = CityView.NormalizedToMap(CityView.MapToNormalized(map));
                Assert.That((back - map).magnitude, Is.LessThan(Cell), $"at {map}");
            }
        }

        [Test]
        public void TopDownCamera_ImageMatchesView()
        {
            // The city image and the costmap overlay use the same view transform; the camera must agree with it to
            // well within one cell, otherwise the overlay is offset from the rendered city.
            var go = new GameObject("MinimapTestCamera");
            try
            {
                var cam = go.AddComponent<Camera>();
                var view = MinimapView.Fit(CityView.min, CityView.Max, 0f, 1024f / 1024f);
                MinimapGeometry.PlaceTopDownCamera(cam, view, 20f, -1f);
                foreach (var map in new[]
                         {
                             new Vector2(-50f, -50f), new Vector2(50f, 50f), new Vector2(-50f, 50f),
                             new Vector2(12.3f, -31.7f), new Vector2(0f, 0f),
                         })
                {
                    Vector3 viewport = cam.WorldToViewportPoint(MinimapGeometry.MapToUnity(map, 0.3f));
                    Vector2 expected = view.MapToNormalized(map);
                    float errorMetres = Vector2.Scale(new Vector2(viewport.x, viewport.y) - expected, view.size).magnitude;
                    Assert.That(errorMetres, Is.LessThan(0.1f * Cell), $"at {map}: viewport {viewport}, expected {expected}");
                    Assert.That(viewport.z, Is.GreaterThan(cam.nearClipPlane).And.LessThan(cam.farClipPlane));
                }
            }
            finally
            {
                Object.DestroyImmediate(go);
            }
        }

        [Test]
        public void GoalYaw_DragThenRobotThenZero()
        {
            var goal = new Vector2(10f, 0f);
            Assert.That(MinimapGeometry.GoalYaw(goal, new Vector2(10f, 5f), new Vector2(0f, 0f), 0.5f),
                Is.EqualTo(Mathf.PI / 2f).Within(1e-5f), "drag sets the heading");
            Assert.That(MinimapGeometry.GoalYaw(goal, new Vector2(10.1f, 0f), new Vector2(10f, -4f), 0.5f),
                Is.EqualTo(Mathf.PI / 2f).Within(1e-5f), "short drag: from the robot towards the goal");
            Assert.That(MinimapGeometry.GoalYaw(goal, null, null, 0.5f), Is.EqualTo(0f));

            var q = MinimapGeometry.QuaternionFromYaw(1.2);
            Assert.That(MinimapGeometry.YawFromQuaternion(q.x, q.y, q.z, q.w), Is.EqualTo(1.2f).Within(1e-5f));
        }

        [Test]
        public void PositionSigma_UsesLargerAxis()
        {
            var cov = new double[36];
            cov[0] = 0.04;
            cov[7] = 0.25;
            Assert.That(MinimapGeometry.PositionSigma(cov), Is.EqualTo(0.5f).Within(1e-6f));
            Assert.That(MinimapGeometry.PositionSigma(null), Is.EqualTo(0f));
        }

        [Test]
        public void Palette_MapsCostValues()
        {
            var lut = CostmapPalette.BuildLut();
            Assert.That(lut[0].a, Is.EqualTo(0), "free is transparent");
            Assert.That(lut[255], Is.EqualTo(CostmapPalette.Unknown));
            Assert.That(lut[100], Is.EqualTo(CostmapPalette.Lethal));
            Assert.That(lut[99].a, Is.GreaterThan(lut[1].a), "higher cost is more opaque");

            var cells = new sbyte[] { 0, 50, 100, -1 };
            using var pixels = new NativeArray<Color32>(6, Allocator.Temp);
            CostmapPalette.Fill(cells, lut, pixels);
            Assert.That(pixels[2], Is.EqualTo(CostmapPalette.Lethal));
            Assert.That(pixels[3], Is.EqualTo(CostmapPalette.Unknown));
            Assert.That(pixels[5], Is.EqualTo(CostmapPalette.Invalid), "missing cells are flagged");
        }
    }
}

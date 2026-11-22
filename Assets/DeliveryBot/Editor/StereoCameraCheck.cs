using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Check Stereo Cameras (Play mode, with Unity in front).
    /// Puts a small magenta marker at known points in front of the left camera, waits for fresh stereo pairs and
    /// checks that camera_info reprojects each point onto the observed marker centre within 1 px in both images (the
    /// right one through P, so the baseline in P[3] is tested too). Also checks the camera poses against the URDF and,
    /// if enabled, the ground-truth depth against a raycast.
    /// </summary>
    public static class StereoCameraCheck
    {
        const float MaxPixelError = 1f;
        const float MaxDepthError = 0.01f; // m
        const float UrdfBaseline = 0.062f;

        // Points in the left camera's optical frame (x right, y down, z forward), metres. Kept above the floor
        // (the camera is 0.42 m high) and inside the test ground's walls.
        static readonly Vector3[] Points =
        {
            new Vector3(0f, 0f, 3f),
            new Vector3(1.6f, -1.2f, 3f),
            new Vector3(-1.6f, 0.25f, 3f),
            new Vector3(0.25f, -0.15f, 1f),
            new Vector3(-0.6f, -0.9f, 8f),
        };

        static StereoCameraPublisher s_Publisher;
        static GameObject s_Marker;
        static int s_PointIndex;
        static int s_PairsSincePlaced;
        static readonly List<string> s_Failures = new List<string>();

        [MenuItem("Tools/Delivery Bot/Check Stereo Cameras", priority = 20)]
        public static void Run()
        {
            if (!EditorApplication.isPlaying)
            {
                Debug.LogError("[StereoCheck] Enter Play mode first.");
                return;
            }
            s_Publisher = Object.FindFirstObjectByType<StereoCameraPublisher>();
            if (s_Publisher == null || s_Publisher.LeftCamera == null)
            {
                Debug.LogError("[StereoCheck] No active StereoCameraPublisher in the scene.");
                return;
            }

            s_Failures.Clear();
            CheckPoses();

            s_Marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s_Marker.name = "StereoCheckMarker";
            var material = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
            material.SetColor("_BaseColor", Color.magenta);
            s_Marker.GetComponent<Renderer>().sharedMaterial = material;

            s_PointIndex = 0;
            PlaceMarker();
            s_Publisher.PairCaptured += OnPair;
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
        }

        static void CheckPoses()
        {
            var left = s_Publisher.LeftCamera.transform;
            var right = s_Publisher.RightCamera.transform;
            Vector3 offset = left.InverseTransformPoint(right.position); // Unity camera-local: x right, y up, z forward
            Expect(Mathf.Abs(offset.x - UrdfBaseline) < 0.001f && Mathf.Abs(offset.y) < 0.001f && Mathf.Abs(offset.z) < 0.001f,
                $"right camera at {Format(offset)} m in the left camera frame, expected ({UrdfBaseline}, 0, 0)");
            Expect(Quaternion.Angle(left.rotation, right.rotation) < 0.05f, "cameras are not parallel");

            var robot = s_Publisher.transform;
            Expect(Vector3.Angle(left.forward, robot.forward) < 0.1f, "left camera doesn't look along the robot's forward axis");
            Expect(System.Math.Abs(s_Publisher.RightIntrinsics.Tx + s_Publisher.RightIntrinsics.Fx * s_Publisher.Baseline) < 1e-6,
                "right camera P[3] != -fx * baseline");
        }

        static void PlaceMarker()
        {
            Vector3 optical = Points[s_PointIndex];
            s_Marker.transform.position = s_Publisher.LeftCamera.transform.TransformPoint(StereoIntrinsics.UnityLocalToOptical(optical));
            s_Marker.transform.localScale = Vector3.one * (0.02f * optical.z); // ~5 px radius at any distance
            s_PairsSincePlaced = 0;
        }

        static void OnPair(double stamp)
        {
            // The first pair may have been rendered before the marker moved.
            if (++s_PairsSincePlaced < 2) return;

            Vector3 point = Points[s_PointIndex];
            CheckImage("left", s_Publisher.LatestLeftRgb, s_Publisher.LeftIntrinsics, point, 0.0);
            CheckImage("right", s_Publisher.LatestRightRgb, s_Publisher.RightIntrinsics, point, s_Publisher.RightIntrinsics.Tx);
            if (s_Publisher.LatestDepth != null)
                CheckDepth(point);

            if (++s_PointIndex < Points.Length)
            {
                PlaceMarker();
                return;
            }
            Finish();
        }

        /// <summary>Expected pixel through the camera's projection matrix P, with the point in the LEFT optical frame.</summary>
        static Vector2 ProjectWithP(StereoIntrinsics k, Vector3 p, double tx)
        {
            return new Vector2((float)((k.Fx * p.x + k.Cx * p.z + tx) / p.z), (float)((k.Fy * p.y + k.Cy * p.z) / p.z));
        }

        static void CheckImage(string side, byte[] rgb, StereoIntrinsics k, Vector3 point, double tx)
        {
            Vector2 expected = ProjectWithP(k, point, tx);
            if (!FindMarker(rgb, k.Width, k.Height, out Vector2 observed, out int count))
            {
                Expect(false, $"{side}: marker for point {Format(point)} not found (expected at {expected})");
                return;
            }
            float error = Vector2.Distance(expected, observed);
            Debug.Log($"[StereoCheck] {side} {Format(point)}: expected {expected}, observed {observed} ({count} px), error {error:F2} px");
            Expect(error <= MaxPixelError, $"{side}: point {Format(point)} off by {error:F2} px");
        }

        static bool FindMarker(byte[] rgb, int width, int height, out Vector2 centre, out int count)
        {
            double sumX = 0, sumY = 0;
            count = 0;
            for (int v = 0; v < height; v++)
            {
                for (int u = 0; u < width; u++)
                {
                    int i = (v * width + u) * 3;
                    if (rgb[i] > 200 && rgb[i + 1] < 80 && rgb[i + 2] > 200)
                    {
                        sumX += u;
                        sumY += v;
                        count++;
                    }
                }
            }
            centre = count > 0 ? new Vector2((float)(sumX / count), (float)(sumY / count)) : Vector2.zero;
            return count > 0;
        }

        static void CheckDepth(Vector3 point)
        {
            var k = s_Publisher.LeftIntrinsics;
            Vector2 pixel = k.Project(point);
            int u = Mathf.RoundToInt(pixel.x), v = Mathf.RoundToInt(pixel.y);
            float measured = s_Publisher.LatestDepth[v * k.Width + u];

            // Ground truth: the camera ray through that pixel centre hits the marker's front surface.
            var cam = s_Publisher.LeftCamera.transform;
            Vector3 direction = new Vector3((float)((u - k.Cx) / k.Fx), (float)((v - k.Cy) / k.Fy), 1f);
            Vector3 worldDir = cam.TransformDirection(StereoIntrinsics.UnityLocalToOptical(direction));
            int mask = ~(1 << LayerMask.NameToLayer(DeliveryBotSetupWindow.RobotLayer));
            if (!Physics.Raycast(cam.position, worldDir, out RaycastHit hit, 50f, mask, QueryTriggerInteraction.Ignore))
            {
                Expect(false, $"depth: raycast to {Format(point)} missed");
                return;
            }
            float expected = cam.InverseTransformPoint(hit.point).z;
            Debug.Log($"[StereoCheck] depth {Format(point)}: expected {expected:F4} m, depth_gt {measured:F4} m");
            Expect(Mathf.Abs(measured - expected) <= MaxDepthError, $"depth at {Format(point)} off by {measured - expected:F4} m");
        }

        static void Finish()
        {
            Cleanup();
            if (s_Failures.Count == 0)
                Debug.Log($"[StereoCheck] PASS: poses, baseline, P and {Points.Length} reprojections" +
                          (s_Publisher.LatestDepth != null ? " plus ground-truth depth" : " (depth_gt disabled, not checked)"));
            else
                Debug.LogError("[StereoCheck] FAIL:\n" + string.Join("\n", s_Failures));
        }

        static void OnPlayModeChanged(PlayModeStateChange change)
        {
            if (change == PlayModeStateChange.ExitingPlayMode)
                Cleanup();
        }

        static void Cleanup()
        {
            if (s_Publisher != null) s_Publisher.PairCaptured -= OnPair;
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
            if (s_Marker != null) Object.Destroy(s_Marker);
        }

        static void Expect(bool condition, string failure)
        {
            if (!condition) s_Failures.Add(failure);
        }

        static string Format(Vector3 v) => $"({v.x:F3}, {v.y:F3}, {v.z:F3})";
    }
}

using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Check Cameras (Play mode, with Unity in front).
    /// Checks the stereo pair and, if present, the two side cameras:
    /// - Poses against the URDF (stereo baseline and parallel axes; side camera position in base_link and the ±21.7°
    ///   outward yaw).
    /// - camera_info: stereo P[3] = -fx * baseline; side cameras with zero distortion, P without Tx and the
    ///   configured horizontal FOV.
    /// - Reprojection: a small magenta marker is put at known points in front of each camera, and camera_info must
    ///   reproject each point onto the observed marker centre within 1 px (the stereo right image through P, so the
    ///   baseline is tested too). Ground-truth depth against a raycast if enabled.
    /// - Stamps: at equal rates, side captures carry the same stamps as stereo pairs.
    /// </summary>
    public static class CameraCheck
    {
        const string Tag = "[CameraCheck]";
        const float MaxPixelError = 1f;
        const float MaxDepthError = 0.01f; // m
        const float UrdfBaseline = 0.062f;
        // mono_{left,right}_camera_joint in the URDF: xyz in base_link (FLU) and yaw.
        static readonly Vector3 UrdfSideCameraLeft = new Vector3(0.5326f, 0.1575f, 0.4180f);
        const float UrdfSideYawDeg = 0.3787f * Mathf.Rad2Deg;

        // Points in the checked camera's optical frame (x right, y down, z forward), metres. Kept above the floor
        // (the cameras are about 0.42 m high) and inside the test ground's walls.
        static readonly Vector3[] StereoPoints =
        {
            new Vector3(0f, 0f, 3f),
            new Vector3(1.6f, -1.2f, 3f),
            new Vector3(-1.6f, 0.25f, 3f),
            new Vector3(0.25f, -0.15f, 1f),
            new Vector3(-0.6f, -0.9f, 8f),
        };
        static readonly Vector3[] SidePoints =
        {
            new Vector3(0f, 0f, 2.5f),
            new Vector3(1.2f, -0.5f, 2.5f),
            new Vector3(-1.2f, 0.2f, 2.5f),
            new Vector3(0.2f, -0.1f, 1f),
            new Vector3(-0.5f, -0.6f, 4f),
        };

        /// <summary>One camera (or the stereo pair) whose captures are checked against a list of points.</summary>
        class Stage
        {
            public string name;
            public Transform opticalParent; // Unity camera whose optical frame the points are given in
            public Vector3[] points;
            public Action<Action<double>> subscribe, unsubscribe;
            public Action<Vector3> checkPoint;
        }

        static StereoCameraPublisher s_Stereo;
        static SideCameraPublisher s_Side;
        static readonly List<Stage> s_Stages = new List<Stage>();
        static int s_StageIndex, s_PointIndex, s_CapturesSincePlaced;
        static GameObject s_Marker;
        static readonly List<string> s_Failures = new List<string>();
        static readonly HashSet<double> s_StereoStamps = new HashSet<double>();
        static readonly HashSet<double> s_SideStamps = new HashSet<double>();
        static readonly Action<double> s_OnCapture = OnCapture;
        static readonly Action<double> s_OnStereoStamp = stamp => s_StereoStamps.Add(stamp);
        static readonly Action<double> s_OnSideStamp = stamp => s_SideStamps.Add(stamp);

        [MenuItem("Tools/Delivery Bot/Check Cameras", priority = 20)]
        public static void Run()
        {
            if (!EditorApplication.isPlaying)
            {
                Debug.LogError($"{Tag} Enter Play mode first.");
                return;
            }
            Cleanup();
            s_Stereo = Object.FindFirstObjectByType<StereoCameraPublisher>();
            if (s_Stereo == null || s_Stereo.LeftCamera == null)
            {
                Debug.LogError($"{Tag} No active StereoCameraPublisher in the scene.");
                return;
            }
            s_Side = Object.FindFirstObjectByType<SideCameraPublisher>();
            if (s_Side != null && s_Side.Left == null) s_Side = null;

            s_Failures.Clear();
            s_StereoStamps.Clear();
            s_SideStamps.Clear();
            s_Stages.Clear();

            CheckStereoPoses();
            s_Stages.Add(new Stage
            {
                name = "stereo",
                opticalParent = s_Stereo.LeftCamera.transform,
                points = StereoPoints,
                subscribe = a => s_Stereo.PairCaptured += a,
                unsubscribe = a => s_Stereo.PairCaptured -= a,
                checkPoint = CheckStereoPoint,
            });

            if (s_Side != null)
            {
                foreach (var feed in new[] { s_Side.Left, s_Side.Right })
                {
                    CheckSidePose(feed);
                    CheckSideCameraInfo(feed);
                    var f = feed;
                    string name = feed.Side == CameraSide.Left ? "side left" : "side right";
                    s_Stages.Add(new Stage
                    {
                        name = name,
                        opticalParent = feed.Camera.transform,
                        points = SidePoints,
                        subscribe = a => f.Captured += a,
                        unsubscribe = a => f.Captured -= a,
                        checkPoint = p => CheckImage(name, f.LatestRgb, f.Intrinsics, p, 0.0),
                    });
                }
                s_Stereo.PairCaptured += s_OnStereoStamp;
                s_Side.Captured += s_OnSideStamp;
            }
            else
            {
                Debug.LogWarning($"{Tag} No active SideCameraPublisher; only the stereo pair is checked.");
            }

            s_Marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s_Marker.name = "CameraCheckMarker";
            var material = new Material(Shader.Find("Universal Render Pipeline/Unlit"));
            material.SetColor("_BaseColor", Color.magenta);
            s_Marker.GetComponent<Renderer>().sharedMaterial = material;

            s_StageIndex = 0;
            StartStage();
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
        }

        // ---------------------------------------------------------------- poses and camera_info

        static void CheckStereoPoses()
        {
            var left = s_Stereo.LeftCamera.transform;
            var right = s_Stereo.RightCamera.transform;
            Vector3 offset = left.InverseTransformPoint(right.position); // Unity camera-local: x right, y up, z forward
            Expect(Mathf.Abs(offset.x - UrdfBaseline) < 0.001f && Mathf.Abs(offset.y) < 0.001f && Mathf.Abs(offset.z) < 0.001f,
                $"stereo: right camera at {Format(offset)} m in the left camera frame, expected ({UrdfBaseline}, 0, 0)");
            Expect(Quaternion.Angle(left.rotation, right.rotation) < 0.05f, "stereo: cameras are not parallel");

            var robot = FindLink(s_Stereo.transform, "base_link");
            Expect(Vector3.Angle(left.forward, robot.forward) < 0.1f, "stereo: left camera doesn't look along the robot's forward axis");
            Expect(Math.Abs(s_Stereo.RightIntrinsics.Tx + s_Stereo.RightIntrinsics.Fx * s_Stereo.Baseline) < 1e-6,
                "stereo: right camera P[3] != -fx * baseline");
        }

        static void CheckSidePose(SideCameraPublisher.Feed feed)
        {
            string name = feed.Side == CameraSide.Left ? "side left" : "side right";
            float sign = feed.Side == CameraSide.Left ? 1f : -1f;
            // The base_link articulation body, not the robot root: the body settles and tilts slightly under physics.
            var robot = FindLink(s_Side.transform, "base_link");
            var cam = feed.Camera.transform;

            var flu = robot.InverseTransformPoint(cam.position).To<FLU>();
            var position = new Vector3(flu.x, flu.y, flu.z);
            var expected = new Vector3(UrdfSideCameraLeft.x, sign * UrdfSideCameraLeft.y, UrdfSideCameraLeft.z);
            Expect(Vector3.Distance(position, expected) < 0.001f,
                $"{name}: camera at {Format(position)} m in base_link (FLU), URDF says {Format(expected)}");

            // Unity's signed angle around up is clockwise from above; ROS yaw is counter-clockwise.
            float yaw = -Vector3.SignedAngle(robot.forward, cam.forward, robot.up);
            Expect(Mathf.Abs(yaw - sign * UrdfSideYawDeg) < 0.1f,
                $"{name}: yaw {yaw:F2}° from the robot's forward axis, URDF says {sign * UrdfSideYawDeg:F2}°");
            Expect(Vector3.Angle(cam.up, robot.up) < 0.1f, $"{name}: camera is pitched or rolled ({Vector3.Angle(cam.up, robot.up):F2}°)");
            Debug.Log($"{Tag} {name}: position {Format(position)} m, yaw {yaw:F2}°");
        }

        static void CheckSideCameraInfo(SideCameraPublisher.Feed feed)
        {
            string name = feed.Side == CameraSide.Left ? "side left" : "side right";
            var k = feed.Intrinsics;
            var info = k.ToCameraInfo(new RosMessageTypes.Std.HeaderMsg());
            Expect(Array.TrueForAll(info.D, d => d == 0.0), $"{name}: distortion is not zero");
            Expect(info.P[3] == 0.0, $"{name}: P[3] = {info.P[3]}, expected 0 (monocular)");
            Expect(info.width == s_Side.width && info.height == s_Side.height,
                $"{name}: camera_info is {info.width}x{info.height}, configured {s_Side.width}x{s_Side.height}");
            double hfov = k.HorizontalFovRad;
            double configured = s_Side.horizontalFovDeg * Mathf.Deg2Rad;
            Expect(Math.Abs(hfov - configured) < 1e-4,
                $"{name}: horizontal FOV from fx is {hfov:F5} rad, configured {configured:F5} rad");
            Debug.Log($"{Tag} {name}: {info.width}x{info.height}, fx = fy = {k.Fx:F3}, c = ({k.Cx}, {k.Cy}), horizontal FOV {hfov:F4} rad");
        }

        static Transform FindLink(Transform robot, string linkName)
        {
            foreach (var t in robot.GetComponentsInChildren<Transform>(true))
                if (t.name == linkName) return t;
            return robot;
        }

        // ---------------------------------------------------------------- reprojection

        static void StartStage()
        {
            var stage = s_Stages[s_StageIndex];
            s_PointIndex = 0;
            PlaceMarker();
            stage.subscribe(s_OnCapture);
        }

        static void PlaceMarker()
        {
            var stage = s_Stages[s_StageIndex];
            Vector3 optical = stage.points[s_PointIndex];
            s_Marker.transform.position = stage.opticalParent.TransformPoint(StereoIntrinsics.UnityLocalToOptical(optical));
            s_Marker.transform.localScale = Vector3.one * (0.02f * optical.z); // ~5 px radius at any distance
            s_CapturesSincePlaced = 0;
        }

        static void OnCapture(double stamp)
        {
            // The first capture may have been rendered before the marker moved.
            if (++s_CapturesSincePlaced < 2) return;

            var stage = s_Stages[s_StageIndex];
            stage.checkPoint(stage.points[s_PointIndex]);

            if (++s_PointIndex < stage.points.Length)
            {
                PlaceMarker();
                return;
            }
            stage.unsubscribe(s_OnCapture);
            if (++s_StageIndex < s_Stages.Count)
            {
                StartStage();
                return;
            }
            Finish();
        }

        static void CheckStereoPoint(Vector3 point)
        {
            CheckImage("stereo left", s_Stereo.LatestLeftRgb, s_Stereo.LeftIntrinsics, point, 0.0);
            CheckImage("stereo right", s_Stereo.LatestRightRgb, s_Stereo.RightIntrinsics, point, s_Stereo.RightIntrinsics.Tx);
            if (s_Stereo.LatestDepth != null)
                CheckDepth(point);
        }

        /// <summary>Expected pixel through the camera's projection matrix P, with the point in the stage's optical frame.</summary>
        static Vector2 ProjectWithP(StereoIntrinsics k, Vector3 p, double tx)
        {
            return new Vector2((float)((k.Fx * p.x + k.Cx * p.z + tx) / p.z), (float)((k.Fy * p.y + k.Cy * p.z) / p.z));
        }

        static void CheckImage(string name, byte[] rgb, StereoIntrinsics k, Vector3 point, double tx)
        {
            Vector2 expected = ProjectWithP(k, point, tx);
            if (!FindMarker(rgb, k.Width, k.Height, out Vector2 observed, out int count))
            {
                Expect(false, $"{name}: marker for point {Format(point)} not found (expected at {expected})");
                return;
            }
            float error = Vector2.Distance(expected, observed);
            Debug.Log($"{Tag} {name} {Format(point)}: expected {expected}, observed {observed} ({count} px), error {error:F2} px");
            Expect(error <= MaxPixelError, $"{name}: point {Format(point)} off by {error:F2} px");
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
            var k = s_Stereo.LeftIntrinsics;
            Vector2 pixel = k.Project(point);
            int u = Mathf.RoundToInt(pixel.x), v = Mathf.RoundToInt(pixel.y);
            float measured = s_Stereo.LatestDepth[v * k.Width + u];

            // Ground truth: the camera ray through that pixel centre hits the marker's front surface.
            var cam = s_Stereo.LeftCamera.transform;
            Vector3 direction = new Vector3((float)((u - k.Cx) / k.Fx), (float)((v - k.Cy) / k.Fy), 1f);
            Vector3 worldDir = cam.TransformDirection(StereoIntrinsics.UnityLocalToOptical(direction));
            int mask = ~(1 << LayerMask.NameToLayer(DeliveryBotSetupWindow.RobotLayer));
            if (!Physics.Raycast(cam.position, worldDir, out RaycastHit hit, 50f, mask, QueryTriggerInteraction.Ignore))
            {
                Expect(false, $"depth: raycast to {Format(point)} missed");
                return;
            }
            float expected = cam.InverseTransformPoint(hit.point).z;
            Debug.Log($"{Tag} depth {Format(point)}: expected {expected:F4} m, depth_gt {measured:F4} m");
            Expect(Mathf.Abs(measured - expected) <= MaxDepthError, $"depth at {Format(point)} off by {measured - expected:F4} m");
        }

        // ---------------------------------------------------------------- stamps

        static string CheckStamps()
        {
            if (s_Side == null) return null;
            if (!Mathf.Approximately(s_Side.rateHz, s_Stereo.rateHz))
                return $"stamps not compared (side {s_Side.rateHz} Hz, stereo {s_Stereo.rateHz} Hz)";

            int common = 0;
            foreach (double stamp in s_SideStamps)
                if (s_StereoStamps.Contains(stamp)) common++;
            // Every capture lies on the grid k / rate; either sensor may skip a tick if its readback is slow.
            foreach (double stamp in s_SideStamps)
                Expect(OnGrid(stamp, s_Side.rateHz), $"stamps: side capture at {stamp:F4} s is off the {s_Side.rateHz} Hz grid");
            foreach (double stamp in s_StereoStamps)
                Expect(OnGrid(stamp, s_Stereo.rateHz), $"stamps: stereo capture at {stamp:F4} s is off the {s_Stereo.rateHz} Hz grid");
            Expect(common >= s_SideStamps.Count / 2 && common > 0,
                $"stamps: only {common} of {s_SideStamps.Count} side captures share a stamp with a stereo pair");
            return $"{common} of {s_SideStamps.Count} side captures share their stamp with a stereo pair";
        }

        static bool OnGrid(double stamp, double rateHz)
        {
            double ticks = stamp * rateHz;
            return Math.Abs(ticks - Math.Round(ticks)) < 1e-4;
        }

        // ---------------------------------------------------------------- end

        static void Finish()
        {
            string stamps = CheckStamps();
            Cleanup();
            if (s_Failures.Count == 0)
                Debug.Log($"{Tag} PASS: stereo poses, baseline, P and {StereoPoints.Length} reprojections" +
                          (s_Stereo.LatestDepth != null ? " plus ground-truth depth" : " (depth_gt disabled, not checked)") +
                          (s_Side != null
                              ? $"; side cameras: poses, camera_info and {SidePoints.Length} reprojections each; {stamps}"
                              : "; no side cameras"));
            else
                Debug.LogError($"{Tag} FAIL:\n" + string.Join("\n", s_Failures));
        }

        static void OnPlayModeChanged(PlayModeStateChange change)
        {
            if (change == PlayModeStateChange.ExitingPlayMode)
                Cleanup();
        }

        static void Cleanup()
        {
            if (s_StageIndex < s_Stages.Count) s_Stages[s_StageIndex].unsubscribe(s_OnCapture);
            s_Stages.Clear();
            if (s_Stereo != null) s_Stereo.PairCaptured -= s_OnStereoStamp;
            if (s_Side != null) s_Side.Captured -= s_OnSideStamp;
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

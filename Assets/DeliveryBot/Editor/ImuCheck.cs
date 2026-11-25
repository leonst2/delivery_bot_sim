using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Check IMU (Play mode, CitySim, Unity in front). With the IMU noise-free:
    /// at rest, the specific force is +9.81 m/s² on z and the angular velocity zero; on a constant-radius circle the yaw
    /// rate equals v / R within 2 %, with R from a least-squares circle fit of the base_link path and v from its length;
    /// one reading per physics step (50 Hz).
    /// </summary>
    public static class ImuCheck
    {
        const string Tag = "[ImuCheck]";
        // Same start as Check Determinism: inside the central intersection, facing north (map +x). Exactly at (0, 0)
        // the reset doesn't settle reliably.
        static readonly Vector2 StartMapXY = new Vector2(1.1f, 0.4f);
        const float Speed = 1.0f;     // m/s
        const float Steering = 15f;   // deg, left: R ≈ 1.8 m
        const float RestTolerance = 0.05f;      // m/s²
        const float RestRateTolerance = 0.005f; // rad/s
        const float YawRateTolerance = 0.02f;   // relative

        static ImuCheckProbe s_Probe;

        [MenuItem("Tools/Delivery Bot/Check IMU", priority = 22)]
        public static void Run()
        {
            if (!EditorApplication.isPlaying)
            {
                Debug.LogError($"{Tag} Enter Play mode first (CitySim).");
                return;
            }
            var sim = Object.FindFirstObjectByType<SimControl>();
            if (sim == null || sim.Robot == null || sim.Robot.GetComponent<ImuPublisher>() == null)
            {
                Debug.LogError($"{Tag} Needs a SimControl and a robot with an ImuPublisher in the scene (CitySim).");
                return;
            }
            Cleanup();
            s_Probe = sim.gameObject.AddComponent<ImuCheckProbe>();
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            Vector3 position = new Vector3<FLU>(StartMapXY.x, StartMapXY.y, 0f).toUnity;
            Quaternion rotation = new Quaternion<FLU>(0f, 0f, 0f, 1f).toUnity;
            float seconds = (ImuCheckProbe.RestWaitSteps + ImuCheckProbe.RestSteps + ImuCheckProbe.DriveSettleSteps +
                             ImuCheckProbe.DriveSteps) * Time.fixedDeltaTime;
            Debug.Log($"{Tag} Running: reset, rest, then {Speed} m/s at {Steering}° steering ({seconds:F0} s sim time after the reset) ...");
            s_Probe.Begin(sim, position, rotation, Speed, Steering, Finish);
        }

        static void Finish(ImuCheckProbe.Recording r)
        {
            Cleanup();
            var failures = new List<string>();
            var lines = new List<string>();
            if (!r.resetOk)
            {
                Debug.LogError($"{Tag} FAIL: reset failed: {r.resetMessage}");
                return;
            }

            Vector3 rest = r.restAccelerationSum / Mathf.Max(1, r.restSamples);
            lines.Add($"at rest: specific force ({rest.x:F3}, {rest.y:F3}, {rest.z:F3}) m/s², max |ω| {r.restMaxAngularRate:F4} rad/s");
            if (Mathf.Abs(rest.z - 9.81f) > RestTolerance || new Vector2(rest.x, rest.y).magnitude > RestTolerance)
                failures.Add($"at rest the specific force is not (0, 0, 9.81) ± {RestTolerance}");
            if (r.restMaxAngularRate > RestRateTolerance)
                failures.Add($"at rest |ω| reaches {r.restMaxAngularRate:F4} rad/s");

            int n = r.drivePositions.Count;
            double path = 0, yawRate = 0;
            for (int i = 1; i < n; i++) path += Vector2.Distance(r.drivePositions[i], r.drivePositions[i - 1]);
            foreach (float w in r.driveYawRates) yawRate += w;
            yawRate /= Math.Max(1, r.driveYawRates.Count);
            double seconds = (n - 1) * Time.fixedDeltaTime;
            double v = path / Math.Max(seconds, 1e-9);
            double radius = FitCircleRadius(r.drivePositions);
            double expected = v / radius;
            double error = Math.Abs(yawRate - expected) / Math.Abs(expected);
            lines.Add($"circle: v {v:F4} m/s, R {radius:F4} m, v/R {expected:F4} rad/s, IMU yaw rate {yawRate:F4} rad/s, error {error:P2}");
            if (!(error <= YawRateTolerance)) failures.Add($"yaw rate differs from v/R by {error:P2} (limit {YawRateTolerance:P0})");
            if (yawRate <= 0) failures.Add("a left turn must give a positive yaw rate");

            double rate = r.driveReadings / (ImuCheckProbe.DriveSteps * Time.fixedDeltaTime);
            lines.Add($"rate: {rate:F2} readings/s");
            if (Math.Abs(rate - 50.0) > 2.5) failures.Add($"rate {rate:F2} Hz is outside 50 ± 2.5 Hz");

            string details = string.Join("\n", lines);
            if (failures.Count == 0)
                Debug.Log($"{Tag} PASS\n{details}");
            else
                Debug.LogError($"{Tag} FAIL: {string.Join("; ", failures)}\n{details}");
        }

        /// <summary>Least-squares circle through the points (Kåsa fit): radius in m.</summary>
        static double FitCircleRadius(IReadOnlyList<Vector2> points)
        {
            // Minimise Σ (x² + y² + D x + E y + F)²: normal equations of a 3 × 3 linear system.
            double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
            int n = points.Count;
            foreach (var p in points)
            {
                double x = p.x, y = p.y, z = x * x + y * y;
                sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y;
                sxz += x * z; syz += y * z; sz += z;
            }
            // [sxx sxy sx; sxy syy sy; sx sy n] [D E F]^T = -[sxz syz sz]^T
            double[,] a = { { sxx, sxy, sx }, { sxy, syy, sy }, { sx, sy, n } };
            double[] b = { -sxz, -syz, -sz };
            double[] s = Solve3(a, b);
            double cx = -s[0] / 2, cy = -s[1] / 2;
            return Math.Sqrt(cx * cx + cy * cy - s[2]);
        }

        static double[] Solve3(double[,] a, double[] b)
        {
            double Det(double[,] m) =>
                m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) -
                m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) +
                m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]);
            double d = Det(a);
            var result = new double[3];
            for (int col = 0; col < 3; col++)
            {
                var m = (double[,])a.Clone();
                for (int row = 0; row < 3; row++) m[row, col] = b[row];
                result[col] = Det(m) / d;
            }
            return result;
        }

        static void OnPlayModeChanged(PlayModeStateChange change)
        {
            if (change == PlayModeStateChange.ExitingPlayMode) Cleanup();
        }

        static void Cleanup()
        {
            EditorApplication.playModeStateChanged -= OnPlayModeChanged;
            if (s_Probe != null) Object.Destroy(s_Probe);
            s_Probe = null;
        }
    }
}

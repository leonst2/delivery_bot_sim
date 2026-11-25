using System;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Check Determinism (Play mode, CitySim, Unity in front).
    /// Runs the same reset + command script three times through <see cref="SimControl"/>: twice with seed 42, once with
    /// seed 43. PASS: the two seed-42 runs give bit-identical base_link trajectories and LiDAR scans, and seed 43 gives
    /// different scan noise.
    /// </summary>
    public static class DeterminismCheck
    {
        const string Tag = "[DeterminismCheck]";
        // The central intersection, facing north (map +x).
        static readonly Vector2 StartMapXY = new Vector2(1.1f, 0.4f);
        const float StartMapYawDeg = 0f;
        static readonly uint[] Seeds = { 42, 42, 43 };

        static SimDeterminismProbe s_Probe;
        static readonly SimDeterminismProbe.Recording[] s_Runs = new SimDeterminismProbe.Recording[3];
        static int s_RunIndex;

        [MenuItem("Tools/Delivery Bot/Check Determinism", priority = 21)]
        public static void Run()
        {
            if (!EditorApplication.isPlaying)
            {
                Debug.LogError($"{Tag} Enter Play mode first (CitySim).");
                return;
            }
            var sim = Object.FindFirstObjectByType<SimControl>();
            if (sim == null || sim.Robot == null)
            {
                Debug.LogError($"{Tag} Needs a SimControl and a robot in the scene (CitySim).");
                return;
            }
            Cleanup();
            s_Probe = sim.gameObject.AddComponent<SimDeterminismProbe>();
            s_RunIndex = 0;
            EditorApplication.playModeStateChanged += OnPlayModeChanged;
            Debug.Log($"{Tag} Running {Seeds.Length} runs of {(SimDeterminismProbe.SettleSteps + SimDeterminismProbe.CommandSteps) * Time.fixedDeltaTime:F1} s sim time ...");
            StartRun(sim);
        }

        static void StartRun(SimControl sim)
        {
            Vector3 position = new Vector3<FLU>(StartMapXY.x, StartMapXY.y, 0f).toUnity;
            float yaw = StartMapYawDeg * Mathf.Deg2Rad;
            Quaternion rotation = new Quaternion<FLU>(0f, 0f, Mathf.Sin(0.5f * yaw), Mathf.Cos(0.5f * yaw)).toUnity;
            s_Probe.Begin(sim, position, rotation, Seeds[s_RunIndex], recording =>
            {
                s_Runs[s_RunIndex] = recording;
                if (++s_RunIndex < Seeds.Length)
                    StartRun(sim);
                else
                    Finish();
            });
        }

        static void Finish()
        {
            var a = s_Runs[0];
            var b = s_Runs[1];
            var c = s_Runs[2];
            Cleanup();

            var failures = new System.Collections.Generic.List<string>();
            for (int i = 0; i < s_Runs.Length; i++)
                if (!s_Runs[i].resetOk) failures.Add($"run {i + 1}: reset failed: {s_Runs[i].resetMessage}");

            int steps = Math.Min(a.positions.Count, b.positions.Count);
            double maxPosition = 0, maxAngle = 0;
            int firstDiff = -1;
            for (int i = 0; i < steps; i++)
            {
                double dp = Vector3.Distance(a.positions[i], b.positions[i]);
                double da = Quaternion.Angle(a.rotations[i], b.rotations[i]);
                if ((dp > 0 || a.rotations[i] != b.rotations[i]) && firstDiff < 0) firstDiff = i;
                maxPosition = Math.Max(maxPosition, dp);
                maxAngle = Math.Max(maxAngle, da);
            }
            float travelled = 0f;
            for (int i = 1; i < a.positions.Count; i++) travelled += Vector3.Distance(a.positions[i], a.positions[i - 1]);

            if (a.positions.Count != b.positions.Count) failures.Add($"runs recorded {a.positions.Count} and {b.positions.Count} steps");
            if (firstDiff >= 0)
                failures.Add($"trajectories differ from command step {firstDiff}: max {maxPosition * 1000.0:F4} mm, {maxAngle:F4}°");

            bool scansEqual = SameScans(a, b, out string scanNote);
            if (!scansEqual) failures.Add("seed 42 runs: LiDAR scans differ (" + scanNote + ")");
            bool seedMatters = !SameScans(a, c, out _);
            if (!seedMatters) failures.Add("seed 43 gave the same LiDAR noise as seed 42");

            string summary = $"{steps} steps, {travelled:F2} m driven, {a.scans.Count} scans per run; " +
                             $"max deviation {maxPosition * 1000.0:F4} mm / {maxAngle:F4}°";
            if (failures.Count == 0)
                Debug.Log($"{Tag} PASS: seed 42 twice gives bit-identical trajectories and scans, seed 43 different scan noise ({summary})");
            else
                Debug.LogError($"{Tag} FAIL ({summary}):\n" + string.Join("\n", failures));
        }

        static bool SameScans(SimDeterminismProbe.Recording a, SimDeterminismProbe.Recording b, out string note)
        {
            note = "";
            if (a.scans.Count != b.scans.Count)
            {
                note = $"{a.scans.Count} vs {b.scans.Count} scans";
                return false;
            }
            for (int s = 0; s < a.scans.Count; s++)
            {
                var x = a.scans[s];
                var y = b.scans[s];
                if (x.Length != y.Length)
                {
                    note = $"scan {s}: {x.Length} vs {y.Length} beams";
                    return false;
                }
                for (int i = 0; i < x.Length; i++)
                {
                    // Bitwise, so NaN/Inf compare equal to themselves.
                    if (BitConverter.SingleToInt32Bits(x[i]) == BitConverter.SingleToInt32Bits(y[i])) continue;
                    note = $"scan {s} beam {i}: {x[i]} vs {y[i]}";
                    return false;
                }
            }
            return true;
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

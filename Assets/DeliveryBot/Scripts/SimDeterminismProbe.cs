using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RosMessageTypes.DeliveryBotInterfaces;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Test helper for Tools > Delivery Bot > Check Determinism: resets the simulation through <see cref="SimControl"/>,
    /// drives a fixed command script keyed on the physics step and records the base_link pose every step and every
    /// LiDAR scan. Everything happens in FixedUpdate, so two runs see exactly the same step sequence. The reset starts
    /// on a step of the 10 Hz sensor grid, so the sensors run in the same phase in every run.
    /// </summary>
    public class SimDeterminismProbe : MonoBehaviour
    {
        public const int SettleSteps = 50; // 1 s for the reset to settle before the commands start

        /// <summary>(first step, speed m/s, steering deg), applied from that step on.</summary>
        public static readonly (int step, float speed, float steering)[] Commands =
        {
            (0, 1.0f, 0f),
            (75, 1.0f, 15f),
            (175, -0.5f, 0f),
            (225, 0f, 0f),
        };
        public const int CommandSteps = 250;

        public class Recording
        {
            public bool resetOk;
            public string resetMessage;
            public readonly List<Vector3> positions = new List<Vector3>();
            public readonly List<Quaternion> rotations = new List<Quaternion>();
            public readonly List<float[]> scans = new List<float[]>();
        }

        SimControl m_Sim;
        LidarScanPublisher m_Lidar;
        Transform m_BaseLink;
        Vector3 m_Position;
        Quaternion m_Rotation;
        uint m_Seed;
        Action<Recording> m_Done;
        Recording m_Recording;
        Task<ResetSimResponse> m_Reset;
        bool m_WaitForGrid;
        int m_Step = -1;

        /// <summary>Starts one run; <paramref name="done"/> gets the recording.</summary>
        public void Begin(SimControl sim, Vector3 position, Quaternion rotation, uint seed, Action<Recording> done)
        {
            m_Sim = sim;
            m_Position = position;
            m_Rotation = rotation;
            m_Seed = seed;
            m_Done = done;
            m_Recording = new Recording();
            var robot = sim.Robot;
            m_BaseLink = robot.Root.transform;
            m_Lidar = robot.GetComponentInChildren<LidarScanPublisher>();
            if (m_Lidar != null) m_Lidar.ScanPublished += OnScan;
            m_WaitForGrid = true;
            m_Step = -1;
        }

        void OnDestroy()
        {
            if (m_Lidar != null) m_Lidar.ScanPublished -= OnScan;
        }

        void OnScan(float[] ranges, double stamp)
        {
            if (m_Step >= SettleSteps) m_Recording.scans.Add((float[])ranges.Clone());
        }

        void FixedUpdate()
        {
            if (m_Done == null) return;
            if (m_WaitForGrid)
            {
                double ticks = Time.fixedTimeAsDouble * 10.0;
                if (Math.Abs(ticks - Math.Round(ticks)) > 1e-3) return;
                m_WaitForGrid = false;
                m_Reset = m_Sim.ResetAsync(m_Position, m_Rotation, m_Seed);
                m_Step = 0;
                return;
            }

            m_Step++;
            if (m_Step < SettleSteps) return;
            if (m_Step == SettleSteps)
            {
                m_Recording.resetOk = m_Reset.IsCompleted && m_Reset.Result.success;
                m_Recording.resetMessage = m_Reset.IsCompleted ? m_Reset.Result.message : "reset still settling after 1 s";
            }

            int commandStep = m_Step - SettleSteps;
            var robot = m_Sim.Robot;
            foreach (var c in Commands)
                if (c.step == commandStep) robot.SetCommand(c.speed, c.steering);

            m_Recording.positions.Add(m_BaseLink.position);
            m_Recording.rotations.Add(m_BaseLink.rotation);

            if (commandStep + 1 < CommandSteps) return;
            robot.SetCommand(0f, 0f);
            if (m_Lidar != null) m_Lidar.ScanPublished -= OnScan;
            var done = m_Done;
            m_Done = null;
            done(m_Recording);
        }
    }
}

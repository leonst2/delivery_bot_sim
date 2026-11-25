using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// A 2D laser scan for display: beam angles (rad, lidar frame, CCW from forward) and ranges (m; Inf or NaN = no
    /// return). The arrays are reused between scans; <see cref="Version"/> changes whenever a new scan arrived.
    /// </summary>
    public interface ILaserScanSource : IDisposable
    {
        string Name { get; }
        bool Available { get; }
        float[] Angles { get; }
        float[] Ranges { get; }
        int Count { get; }
        double Stamp { get; }
        int Version { get; }

        /// <summary>Called once per frame by the consumer; sources that generate data do it here.</summary>
        void Poll();
    }

    /// <summary>Base class with reusable buffers.</summary>
    public abstract class LaserScanBuffer : ILaserScanSource
    {
        public abstract string Name { get; }
        public bool Available => Count > 0;
        public float[] Angles { get; private set; } = Array.Empty<float>();
        public float[] Ranges { get; private set; } = Array.Empty<float>();
        public int Count { get; private set; }
        public double Stamp { get; private set; }
        public int Version { get; private set; }

        public virtual void Poll() { }
        public virtual void Dispose() { }

        /// <summary>Makes room for <paramref name="count"/> beams; allocates only when the scan grows.</summary>
        protected void Begin(int count)
        {
            if (Ranges.Length < count)
            {
                Ranges = new float[count];
                Angles = new float[count];
            }
            Count = count;
        }

        protected void Commit(double stamp)
        {
            Stamp = stamp;
            Version++;
        }
    }

    /// <summary>The raw scan straight from <see cref="LidarScanPublisher"/>, without a ROS round trip.</summary>
    public sealed class UnityLidarSource : LaserScanBuffer
    {
        readonly LidarScanPublisher m_Lidar;

        public UnityLidarSource(LidarScanPublisher lidar)
        {
            m_Lidar = lidar;
            m_Lidar.ScanPublished += OnScan;
        }

        public override string Name => "LidarScanPublisher";
        public LidarScanPublisher Lidar => m_Lidar;
        public bool Alive => m_Lidar != null && m_Lidar.isActiveAndEnabled;

        void OnScan(float[] ranges, double stamp)
        {
            var angles = m_Lidar.Angles;
            int count = Mathf.Min(ranges.Length, angles.Length);
            Begin(count);
            Array.Copy(ranges, Ranges, count);
            Array.Copy(angles, Angles, count);
            Commit(stamp);
        }

        public override void Dispose()
        {
            if (m_Lidar != null) m_Lidar.ScanPublished -= OnScan;
        }
    }

    /// <summary>A sensor_msgs/LaserScan topic (/scan, /scan/filtered).</summary>
    public sealed class RosLaserScanSource : LaserScanBuffer
    {
        readonly string m_Topic;

        public RosLaserScanSource(string topic)
        {
            m_Topic = topic;
            ROSConnection.GetOrCreateInstance().Subscribe<LaserScanMsg>(topic, OnScan);
        }

        public override string Name => m_Topic;

        void OnScan(LaserScanMsg msg)
        {
            int count = msg.ranges.Length;
            Begin(count);
            Array.Copy(msg.ranges, Ranges, count);
            for (int i = 0; i < count; i++)
                Angles[i] = msg.angle_min + i * msg.angle_increment;
            Commit(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9);
        }
    }

    /// <summary>
    /// Synthetic scan for testing the panel without the robot or the stack: a 10×8 m room with a pillar and a disc
    /// orbiting the lidar at 3 m. Deterministic: the scene depends only on the scan time (sim time, 10 Hz).
    /// </summary>
    public sealed class MockLaserScanSource : LaserScanBuffer
    {
        public const int Beams = 360;
        public const float RateHz = 10f;
        public const float OrbitRadius = 3f;
        public const float OrbitSpeed = 0.4f; // rad/s
        public const float DiscRadius = 0.4f;
        const float k_MaxRange = 12f;
        static readonly Vector2 k_RoomMin = new Vector2(-4f, -4f); // lidar frame (x forward, y left)
        static readonly Vector2 k_RoomMax = new Vector2(6f, 4f);
        static readonly Vector2 k_Pillar = new Vector2(2.5f, -2f);
        const float k_PillarRadius = 0.3f;

        long m_LastTick = -1;

        public override string Name => "mock";

        /// <summary>The orbiting disc at the last scan, in the lidar frame (x forward, y left).</summary>
        public Vector2 DiscPosition { get; private set; }
        public Vector2 DiscVelocity { get; private set; }

        public override void Poll()
        {
            long tick = (long)Math.Floor(Time.timeAsDouble * RateHz);
            if (tick == m_LastTick) return;
            m_LastTick = tick;
            double t = tick / (double)RateHz;

            float phase = (float)(t * OrbitSpeed);
            DiscPosition = OrbitRadius * new Vector2(Mathf.Cos(phase), Mathf.Sin(phase));
            DiscVelocity = OrbitRadius * OrbitSpeed * new Vector2(-Mathf.Sin(phase), Mathf.Cos(phase));

            Begin(Beams);
            for (int i = 0; i < Beams; i++)
            {
                float angle = -Mathf.PI + i * (2f * Mathf.PI / Beams);
                var dir = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle));
                float r = RayBox(dir);
                r = Mathf.Min(r, RayCircle(dir, k_Pillar, k_PillarRadius));
                r = Mathf.Min(r, RayCircle(dir, DiscPosition, DiscRadius));
                Angles[i] = angle;
                Ranges[i] = r <= k_MaxRange ? r : float.PositiveInfinity;
            }
            Commit(t);
        }

        static float RayBox(Vector2 dir)
        {
            // The ray starts inside the room, so the exit distance is the smallest positive slab distance.
            float tx = dir.x > 0 ? k_RoomMax.x / dir.x : dir.x < 0 ? k_RoomMin.x / dir.x : float.PositiveInfinity;
            float ty = dir.y > 0 ? k_RoomMax.y / dir.y : dir.y < 0 ? k_RoomMin.y / dir.y : float.PositiveInfinity;
            return Mathf.Min(tx, ty);
        }

        static float RayCircle(Vector2 dir, Vector2 centre, float radius)
        {
            float b = Vector2.Dot(dir, centre);
            float c = centre.sqrMagnitude - radius * radius;
            float disc = b * b - c;
            if (disc < 0f) return float.PositiveInfinity;
            float t = b - Mathf.Sqrt(disc);
            return t > 0f ? t : float.PositiveInfinity;
        }
    }
}

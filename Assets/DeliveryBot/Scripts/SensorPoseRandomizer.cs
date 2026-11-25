using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Jitters the local pose of a sensor frame around its nominal URDF pose each episode
    /// (domain randomisation: +/-5 mm, +/-1.5 deg by default). Draws from the simulation seed (<see cref="SimRandom"/>),
    /// one stream per sensor frame, so the jitter is reproducible.
    /// </summary>
    public class SensorPoseRandomizer : MonoBehaviour
    {
        public float positionJitter = 0.005f;
        public float angleJitter = 1.5f;

        Vector3 m_NominalPosition;
        Quaternion m_NominalRotation;
        bool m_Captured;

        void Awake() => Capture();

        /// <summary>Link name / mount name, e.g. "lidar_link/LidarSensor": stable across scenes and runs.</summary>
        string StreamName() => transform.parent != null ? transform.parent.name + "/" + name : name;

        void Capture()
        {
            if (m_Captured) return;
            m_NominalPosition = transform.localPosition;
            m_NominalRotation = transform.localRotation;
            m_Captured = true;
        }

        public void Randomize()
        {
            Capture();
            var rng = SimRandom.Stream("sensor_pose/" + StreamName());
            float Range(float limit) => (float)((rng.NextDouble() * 2.0 - 1.0) * limit);
            // Uniform in the ball: rejection sampling from the cube.
            Vector3 offset;
            do offset = new Vector3(Range(1f), Range(1f), Range(1f));
            while (offset.sqrMagnitude > 1f);
            transform.localPosition = m_NominalPosition + offset * positionJitter;
            transform.localRotation = m_NominalRotation * Quaternion.Euler(
                Range(angleJitter), Range(angleJitter), Range(angleJitter));
        }
    }
}

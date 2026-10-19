using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Jitters the local pose of a sensor frame around its nominal URDF pose each episode
    /// (domain randomisation: +/-5 mm, +/-1.5 deg by default).
    /// </summary>
    public class SensorPoseRandomizer : MonoBehaviour
    {
        public float positionJitter = 0.005f;
        public float angleJitter = 1.5f;

        Vector3 m_NominalPosition;
        Quaternion m_NominalRotation;
        bool m_Captured;

        void Awake() => Capture();

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
            transform.localPosition = m_NominalPosition + Random.insideUnitSphere * positionJitter;
            transform.localRotation = m_NominalRotation * Quaternion.Euler(
                Random.Range(-angleJitter, angleJitter),
                Random.Range(-angleJitter, angleJitter),
                Random.Range(-angleJitter, angleJitter));
        }
    }
}

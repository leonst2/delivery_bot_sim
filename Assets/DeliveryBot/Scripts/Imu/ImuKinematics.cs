using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot.Imu
{
    /// <summary>One IMU reading in the body frame (ROS FLU: x forward, y left, z up).</summary>
    public struct ImuReading
    {
        /// <summary>rad/s</summary>
        public Vector3 angularVelocity;
        /// <summary>Specific force, m/s²: +9.81 on z at rest.</summary>
        public Vector3 linearAcceleration;
    }

    /// <summary>
    /// Ideal IMU at a body frame origin, from the pose of that frame once per physics step. Angular velocity comes from
    /// the rotation change between two steps, linear acceleration from the second difference of the position, minus
    /// gravity, rotated into the body frame. Both are converted to ROS FLU with ROS-TCP-Connector's <c>To&lt;FLU&gt;()</c>
    /// first, so the handedness is handled there. The finite differences lag the true state by about one step.
    /// </summary>
    public sealed class ImuKinematics
    {
        Vector3 m_Position;        // FLU, previous step
        Vector3 m_Velocity;        // FLU, between the last two steps
        Quaternion m_Rotation;     // FLU components (used only as quaternion math), previous step
        bool m_Initialized;

        /// <summary>Restarts from <paramref name="position"/>/<paramref name="rotation"/> (Unity world) at rest.</summary>
        public void Reset(Vector3 position, Quaternion rotation)
        {
            m_Position = Flu(position);
            m_Rotation = Flu(rotation);
            m_Velocity = Vector3.zero;
            m_Initialized = true;
        }

        /// <summary>
        /// Advances by one step of <paramref name="dt"/> seconds to the pose <paramref name="position"/>,
        /// <paramref name="rotation"/> (Unity world) and returns the reading. <paramref name="gravity"/> is the Unity
        /// world gravity vector (<c>Physics.gravity</c>).
        /// </summary>
        public ImuReading Step(Vector3 position, Quaternion rotation, float dt, Vector3 gravity)
        {
            if (!m_Initialized || dt <= 0f)
            {
                if (!m_Initialized) Reset(position, rotation);
                return Reading(Vector3.zero, Vector3.zero, Flu(rotation), gravity);
            }

            Vector3 p = Flu(position);
            Quaternion q = Flu(rotation);

            Vector3 velocity = (p - m_Position) / dt;
            Vector3 acceleration = (velocity - m_Velocity) / dt;

            // Rotation from the previous body frame to the current one, expressed in the previous body frame.
            Quaternion delta = Quaternion.Inverse(m_Rotation) * q;
            if (delta.w < 0f) delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
            var axis = new Vector3(delta.x, delta.y, delta.z);
            float sinHalf = axis.magnitude;
            Vector3 angularVelocity = sinHalf < 1e-9f
                ? 2f * axis / dt
                : axis / sinHalf * (2f * Mathf.Atan2(sinHalf, delta.w) / dt);

            m_Position = p;
            m_Velocity = velocity;
            m_Rotation = q;
            return Reading(angularVelocity, acceleration, q, gravity);
        }

        static ImuReading Reading(Vector3 angularVelocity, Vector3 worldAcceleration, Quaternion rotation, Vector3 gravity)
        {
            Vector3 specificForce = worldAcceleration - Flu(gravity);
            return new ImuReading
            {
                angularVelocity = angularVelocity,
                linearAcceleration = Quaternion.Inverse(rotation) * specificForce,
            };
        }

        static Vector3 Flu(Vector3 unity)
        {
            Vector3<FLU> v = unity.To<FLU>();
            return new Vector3(v.x, v.y, v.z);
        }

        static Quaternion Flu(Quaternion unity)
        {
            Quaternion<FLU> q = unity.To<FLU>();
            return new Quaternion(q.x, q.y, q.z, q.w);
        }
    }
}

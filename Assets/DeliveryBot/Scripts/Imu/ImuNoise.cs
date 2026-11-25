using System;
using UnityEngine;

namespace DeliveryBot.Imu
{
    /// <summary>Noise of one three-axis sensor (gyroscope or accelerometer), per axis.</summary>
    [Serializable]
    public struct ImuNoiseSettings
    {
        [Tooltip("Standard deviation of the white noise added to every sample.")]
        public Vector3 white;
        [Tooltip("Constant bias the sensor starts with (after every reseed).")]
        public Vector3 bias;
        [Tooltip("Bias random walk: the bias changes by N(0, σ²·dt) per step, σ in unit/√s.")]
        public Vector3 biasRandomWalk;
    }

    /// <summary>
    /// White noise, constant bias and bias random walk for one three-axis sensor. Draws exactly two standard normal
    /// samples per axis and step, in a fixed order (x, y, z; walk before white noise), whatever the settings, so a
    /// given random stream always gives the same sequence.
    /// </summary>
    public sealed class ImuNoise
    {
        readonly Vector3 m_White, m_InitialBias, m_Walk;
        Vector3 m_Bias;

        public ImuNoise(ImuNoiseSettings settings)
        {
            m_White = settings.white;
            m_InitialBias = settings.bias;
            m_Walk = settings.biasRandomWalk;
            m_Bias = m_InitialBias;
        }

        /// <summary>The current bias.</summary>
        public Vector3 Bias => m_Bias;

        /// <summary>Back to the constant bias (on reseed).</summary>
        public void ResetBias() => m_Bias = m_InitialBias;

        /// <summary>Diagonal of the white-noise covariance (σ²), for the message's covariance fields.</summary>
        public Vector3 WhiteVariance => Vector3.Scale(m_White, m_White);

        /// <summary>
        /// <paramref name="value"/> with noise for a step of <paramref name="dt"/> seconds. <paramref name="standardNormal"/>
        /// returns N(0, 1) samples from the caller's seeded stream.
        /// </summary>
        public Vector3 Apply(Vector3 value, float dt, Func<double> standardNormal)
        {
            float sqrtDt = Mathf.Sqrt(Mathf.Max(0f, dt));
            var result = Vector3.zero;
            for (int axis = 0; axis < 3; axis++)
            {
                float walk = (float)standardNormal();
                float white = (float)standardNormal();
                m_Bias[axis] += m_Walk[axis] * sqrtDt * walk;
                result[axis] = value[axis] + m_Bias[axis] + m_White[axis] * white;
            }
            return result;
        }
    }
}

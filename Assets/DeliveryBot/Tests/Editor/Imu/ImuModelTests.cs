using System;
using DeliveryBot.Imu;
using NUnit.Framework;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot.Imu.Tests
{
    public class ImuModelTests
    {
        const float Dt = 0.02f; // 50 Hz physics
        static readonly Vector3 Gravity = new Vector3(0f, -9.81f, 0f); // Unity world

        /// <summary>Unity pose of base_link at map (x, y) with yaw (rad, CCW from map +x, ROS convention).</summary>
        static (Vector3 position, Quaternion rotation) MapPose(float x, float y, float yaw)
        {
            Vector3 position = new Vector3<FLU>(x, y, 0f).toUnity;
            Quaternion rotation = new Quaternion<FLU>(0f, 0f, Mathf.Sin(yaw / 2f), Mathf.Cos(yaw / 2f)).toUnity;
            return (position, rotation);
        }

        static ImuReading StepAt(ImuKinematics imu, (Vector3 position, Quaternion rotation) pose) =>
            imu.Step(pose.position, pose.rotation, Dt, Gravity);

        static void AssertVector(Vector3 expected, Vector3 actual, float tolerance, string what)
        {
            Assert.That(Vector3.Distance(expected, actual), Is.LessThan(tolerance), $"{what}: expected {expected:F4}, got {actual:F4}");
        }

        [Test]
        public void AtRest_GivesGravityOnZ_AndNoRotation()
        {
            var imu = new ImuKinematics();
            var pose = MapPose(3f, -2f, 0.7f);
            ImuReading reading = default;
            for (int i = 0; i < 5; i++) reading = StepAt(imu, pose);
            AssertVector(new Vector3(0f, 0f, 9.81f), reading.linearAcceleration, 1e-3f, "specific force");
            AssertVector(Vector3.zero, reading.angularVelocity, 1e-5f, "angular velocity");
        }

        [Test]
        public void Tilted_RotatesGravityIntoTheBodyFrame()
        {
            // Nose up by 10° (pitch about FLU y; negative pitch = nose up). At rest the specific force points straight
            // up, so part of it now lies along the raised forward axis: x = -g·sin(pitch) > 0, z = g·cos(pitch).
            float pitch = -10f * Mathf.Deg2Rad;
            var rotation = new Quaternion<FLU>(0f, Mathf.Sin(pitch / 2f), 0f, Mathf.Cos(pitch / 2f)).toUnity;
            var imu = new ImuKinematics();
            ImuReading reading = default;
            for (int i = 0; i < 3; i++) reading = imu.Step(Vector3.zero, rotation, Dt, Gravity);
            Assert.That(reading.linearAcceleration.magnitude, Is.EqualTo(9.81f).Within(1e-3f));
            Assert.That(reading.linearAcceleration.z, Is.EqualTo(9.81f * Mathf.Cos(pitch)).Within(1e-3f));
            Assert.That(reading.linearAcceleration.x, Is.EqualTo(-9.81f * Mathf.Sin(pitch)).Within(1e-3f));
        }

        [Test]
        public void TurningLeft_GivesPositiveYawRate()
        {
            // Rotating on the spot, counter-clockwise seen from above (a left turn), at 0.5 rad/s.
            const float rate = 0.5f;
            var imu = new ImuKinematics();
            ImuReading reading = default;
            for (int i = 0; i <= 10; i++) reading = StepAt(imu, MapPose(0f, 0f, rate * Dt * i));
            AssertVector(new Vector3(0f, 0f, rate), reading.angularVelocity, 1e-3f, "angular velocity");
        }

        [Test]
        public void CircleToTheLeft_GivesVOverR_AndCentripetalForceToTheLeft()
        {
            const float v = 1.2f, radius = 2.5f;
            float omega = v / radius;
            var imu = new ImuKinematics();
            ImuReading reading = default;
            for (int i = 0; i <= 100; i++)
            {
                // Centre at map (0, radius), start at the origin heading +x: a left-hand circle.
                float yaw = omega * Dt * i;
                reading = StepAt(imu, MapPose(radius * Mathf.Sin(yaw), radius * (1f - Mathf.Cos(yaw)), yaw));
            }
            Assert.That(reading.angularVelocity.z, Is.EqualTo(omega).Within(omega * 0.005f), "yaw rate v/R");
            Assert.That(reading.linearAcceleration.y, Is.EqualTo(v * v / radius).Within(0.02f), "centripetal, to the left");
            Assert.That(reading.linearAcceleration.x, Is.EqualTo(0f).Within(0.03f), "along the path");
            Assert.That(reading.linearAcceleration.z, Is.EqualTo(9.81f).Within(1e-3f), "gravity");
        }

        [Test]
        public void Reset_AvoidsASpikeAfterATeleport()
        {
            var imu = new ImuKinematics();
            for (int i = 0; i < 10; i++) StepAt(imu, MapPose(2f * Dt * i, 0f, 0f)); // 2 m/s along x
            var far = MapPose(40f, -30f, 1.2f);
            imu.Reset(far.position, far.rotation);
            ImuReading reading = StepAt(imu, far);
            AssertVector(new Vector3(0f, 0f, 9.81f), reading.linearAcceleration, 1e-3f, "specific force after reset");
            AssertVector(Vector3.zero, reading.angularVelocity, 1e-5f, "angular velocity after reset");
        }

        static Func<double> Normal(int seed)
        {
            var rng = new System.Random(seed);
            return () =>
            {
                double u1 = 1.0 - rng.NextDouble(), u2 = rng.NextDouble();
                return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
            };
        }

        static readonly ImuNoiseSettings Noisy = new ImuNoiseSettings
        {
            white = new Vector3(0.05f, 0.05f, 0.05f),
            bias = new Vector3(0.1f, -0.2f, 0.3f),
            biasRandomWalk = new Vector3(0.001f, 0.001f, 0.001f),
        };

        [Test]
        public void Noise_IsDeterministicPerSeed()
        {
            Vector3[] Run(int seed)
            {
                var noise = new ImuNoise(Noisy);
                var normal = Normal(seed);
                var samples = new Vector3[200];
                for (int i = 0; i < samples.Length; i++) samples[i] = noise.Apply(Vector3.up, Dt, normal);
                return samples;
            }
            Assert.That(Run(42), Is.EqualTo(Run(42)));
            Assert.That(Run(43), Is.Not.EqualTo(Run(42)));
        }

        [Test]
        public void ZeroSettings_LeaveTheValueExact()
        {
            var noise = new ImuNoise(default);
            var normal = Normal(1);
            var value = new Vector3(0.3f, -1.2f, 9.81f);
            for (int i = 0; i < 50; i++) Assert.That(noise.Apply(value, Dt, normal), Is.EqualTo(value));
        }

        [Test]
        public void Bias_StartsAtTheConstantAndWalks()
        {
            var settings = new ImuNoiseSettings { bias = new Vector3(0.1f, 0f, 0f), biasRandomWalk = new Vector3(0.01f, 0f, 0f) };
            const int runs = 400, steps = 250; // 5 s each
            double sum = 0, sumSquares = 0;
            for (int r = 0; r < runs; r++)
            {
                var noise = new ImuNoise(settings);
                Assert.That(noise.Bias.x, Is.EqualTo(0.1f));
                var normal = Normal(1000 + r);
                for (int i = 0; i < steps; i++) noise.Apply(Vector3.zero, Dt, normal);
                double drift = noise.Bias.x - 0.1;
                sum += drift;
                sumSquares += drift * drift;
                noise.ResetBias();
                Assert.That(noise.Bias.x, Is.EqualTo(0.1f));
            }
            double variance = sumSquares / runs - (sum / runs) * (sum / runs);
            double expected = 0.01 * 0.01 * steps * Dt; // σ²·t
            Assert.That(variance, Is.EqualTo(expected).Within(expected * 0.25), "bias random walk variance");
        }

        [Test]
        public void WhiteNoise_HasTheConfiguredSpread()
        {
            var noise = new ImuNoise(new ImuNoiseSettings { white = new Vector3(0.05f, 0.1f, 0.2f) });
            var normal = Normal(7);
            const int n = 20000;
            var sumSquares = Vector3.zero;
            for (int i = 0; i < n; i++)
            {
                Vector3 s = noise.Apply(Vector3.zero, Dt, normal);
                sumSquares += Vector3.Scale(s, s);
            }
            Vector3 std = new Vector3(Mathf.Sqrt(sumSquares.x / n), Mathf.Sqrt(sumSquares.y / n), Mathf.Sqrt(sumSquares.z / n));
            AssertVector(new Vector3(0.05f, 0.1f, 0.2f), std, 0.005f, "white noise std");
            Assert.That(noise.WhiteVariance, Is.EqualTo(new Vector3(0.0025f, 0.01f, 0.04f)).Using(Vector3EqualityComparerWithin(1e-6f)));
        }

        static System.Collections.Generic.IEqualityComparer<Vector3> Vector3EqualityComparerWithin(float tolerance) =>
            new Vector3Comparer(tolerance);

        sealed class Vector3Comparer : System.Collections.Generic.IEqualityComparer<Vector3>
        {
            readonly float m_Tolerance;
            public Vector3Comparer(float tolerance) => m_Tolerance = tolerance;
            public bool Equals(Vector3 a, Vector3 b) => Vector3.Distance(a, b) <= m_Tolerance;
            public int GetHashCode(Vector3 v) => 0;
        }
    }
}

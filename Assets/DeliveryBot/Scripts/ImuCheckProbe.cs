using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RosMessageTypes.DeliveryBotInterfaces;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Test helper for Tools > Delivery Bot > Check IMU: resets the robot through <see cref="SimControl"/>, records the
    /// noise-free IMU at rest, then drives a constant command and records base_link positions (map) and the IMU yaw rate
    /// every physics step. Everything happens in FixedUpdate, keyed on the physics step.
    /// </summary>
    public class ImuCheckProbe : MonoBehaviour
    {
        public const int MaxResetSteps = 250;   // 5 s for the reset to settle
        public const int RestWaitSteps = 25;    // after the reset, before the rest samples
        public const int RestSteps = 100;       // 2 s at rest
        public const int DriveSettleSteps = 150; // 3 s to reach a steady circle
        public const int DriveSteps = 450;      // 9 s on the circle

        public class Recording
        {
            public bool resetOk;
            public string resetMessage;
            public Vector3 restAccelerationSum;
            public float restMaxAngularRate;
            public int restSamples;
            public readonly List<Vector2> drivePositions = new List<Vector2>(); // map, m
            public readonly List<float> driveYawRates = new List<float>();      // IMU angular velocity z, rad/s
            public long driveReadings;                                          // IMU readings during the drive
            public float speed, steering;
        }

        enum Phase { Resetting, Rest, DriveSettle, Drive }

        SimControl m_Sim;
        ImuPublisher m_Imu;
        Transform m_BaseLink;
        Action<Recording> m_Done;
        Recording m_Recording;
        Task<ResetSimResponse> m_Reset;
        Phase m_Phase;
        int m_Step;
        bool m_WasNoiseFree;
        long m_ReadingsAtDriveStart;

        /// <summary>Starts the check; <paramref name="done"/> gets the recording.</summary>
        public void Begin(SimControl sim, Vector3 position, Quaternion rotation, float speed, float steering, Action<Recording> done)
        {
            var robot = sim.Robot;
            m_Sim = sim;
            m_Imu = robot.GetComponent<ImuPublisher>();
            m_BaseLink = robot.Root.transform;
            m_Done = done;
            m_Recording = new Recording { speed = speed, steering = steering };
            m_WasNoiseFree = m_Imu.noiseFree;
            m_Imu.noiseFree = true;
            m_Reset = sim.ResetAsync(position, rotation, 1);
            m_Phase = Phase.Resetting;
            m_Step = 0;
        }

        void FixedUpdate()
        {
            if (m_Done == null) return;
            m_Step++;
            switch (m_Phase)
            {
                case Phase.Resetting:
                    if (m_Reset.IsCompleted)
                    {
                        m_Recording.resetOk = m_Reset.Result.success;
                        m_Recording.resetMessage = m_Reset.Result.message;
                        if (!m_Recording.resetOk) { Finish(); return; }
                        Next(Phase.Rest);
                    }
                    else if (m_Step >= MaxResetSteps)
                    {
                        m_Recording.resetMessage = "reset not finished after 5 s";
                        Finish();
                    }
                    return;

                case Phase.Rest:
                    if (m_Step <= RestWaitSteps) return;
                    m_Recording.restAccelerationSum += m_Imu.Latest.linearAcceleration;
                    m_Recording.restMaxAngularRate = Mathf.Max(m_Recording.restMaxAngularRate, m_Imu.Latest.angularVelocity.magnitude);
                    m_Recording.restSamples++;
                    if (m_Step >= RestWaitSteps + RestSteps) Next(Phase.DriveSettle);
                    return;

                case Phase.DriveSettle:
                    m_Sim.Robot.SetCommand(m_Recording.speed, m_Recording.steering);
                    if (m_Step >= DriveSettleSteps)
                    {
                        m_ReadingsAtDriveStart = m_Imu.ReadingCount;
                        Next(Phase.Drive);
                    }
                    return;

                case Phase.Drive:
                    m_Sim.Robot.SetCommand(m_Recording.speed, m_Recording.steering);
                    Vector3<FLU> p = m_BaseLink.position.To<FLU>();
                    m_Recording.drivePositions.Add(new Vector2(p.x, p.y));
                    m_Recording.driveYawRates.Add(m_Imu.Latest.angularVelocity.z);
                    if (m_Step >= DriveSteps)
                    {
                        m_Recording.driveReadings = m_Imu.ReadingCount - m_ReadingsAtDriveStart;
                        Finish();
                    }
                    return;
            }
        }

        void Next(Phase phase)
        {
            m_Phase = phase;
            m_Step = 0;
        }

        void Finish()
        {
            if (m_Sim != null && m_Sim.Robot != null) m_Sim.Robot.SetCommand(0f, 0f);
            if (m_Imu != null) m_Imu.noiseFree = m_WasNoiseFree;
            var done = m_Done;
            m_Done = null;
            done?.Invoke(m_Recording);
        }

        void OnDestroy()
        {
            if (m_Done != null && m_Imu != null) m_Imu.noiseFree = m_WasNoiseFree;
        }
    }
}

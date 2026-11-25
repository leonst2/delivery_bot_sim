using DeliveryBot.Imu;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Simulated IMU at base_link: publishes /imu (sensor_msgs/Imu, frame base_link, ROS FLU, sim time) once per physics
    /// step, with angular velocity and linear acceleration (specific force, +9.81 m/s² on z at rest) and no orientation
    /// estimate (orientation_covariance[0] = -1). Lives on the robot root next to <see cref="DeliveryBotDrive"/>; the
    /// setup window adds it.
    ///
    /// The values come from the base_link pose of consecutive steps (<see cref="ImuKinematics"/>), so they lag the true
    /// state by about one step (20 ms). Noise is deterministic per scenario seed (stream "imu").
    /// </summary>
    [RequireComponent(typeof(DeliveryBotDrive))]
    public class ImuPublisher : MonoBehaviour
    {
        const string k_RandomStream = "imu";

        [Header("ROS")]
        public string topic = "/imu";
        public string frameId = "base_link";

        [Header("Noise (deterministic per SimRandom seed, stream \"imu\")")]
        [Tooltip("Publish exact values: no white noise, bias or bias random walk.")]
        public bool noiseFree;
        [Tooltip("Gyroscope, rad/s (white, bias) and rad/s/√s (bias random walk), per axis (FLU x, y, z). "
                 + "Placeholder MEMS values until the robot's IMU is chosen.")]
        public ImuNoiseSettings gyroNoise = new ImuNoiseSettings
        {
            white = new Vector3(0.005f, 0.005f, 0.005f),
            bias = Vector3.zero,
            biasRandomWalk = new Vector3(0.0005f, 0.0005f, 0.0005f),
        };
        [Tooltip("Accelerometer, m/s² (white, bias) and m/s²/√s (bias random walk), per axis (FLU x, y, z). "
                 + "Placeholder MEMS values until the robot's IMU is chosen.")]
        public ImuNoiseSettings accelNoise = new ImuNoiseSettings
        {
            white = new Vector3(0.05f, 0.05f, 0.05f),
            bias = Vector3.zero,
            biasRandomWalk = new Vector3(0.001f, 0.001f, 0.001f),
        };

        /// <summary>The latest reading as published (with noise unless <see cref="noiseFree"/>).</summary>
        public ImuReading Latest { get; private set; }

        /// <summary>Number of readings computed so far (one per physics step), for rate checks.</summary>
        public long ReadingCount { get; private set; }

        DeliveryBotDrive m_Drive;
        ROSConnection m_Ros;
        readonly ImuKinematics m_Kinematics = new ImuKinematics();
        ImuNoise m_Gyro, m_Accel;
        System.Random m_Rng;
        System.Func<double> m_Normal;

        void Awake()
        {
            m_Drive = GetComponent<DeliveryBotDrive>();
            m_Normal = () => SimRandom.Gaussian(m_Rng);
            Reseed();
        }

        void OnEnable()
        {
            SimRandom.Reseeded += Reseed;
            if (m_Drive != null) m_Drive.PoseReset += OnPoseReset;
        }

        void OnDisable()
        {
            SimRandom.Reseeded -= Reseed;
            if (m_Drive != null) m_Drive.PoseReset -= OnPoseReset;
        }

        void OnValidate() => Reseed();

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<ImuMsg>(topic);
        }

        /// <summary>New noise stream from the simulation seed; biases start again from their constant values.</summary>
        void Reseed()
        {
            m_Rng = SimRandom.Stream(k_RandomStream);
            m_Gyro = new ImuNoise(gyroNoise);
            m_Accel = new ImuNoise(accelNoise);
        }

        void OnPoseReset(Vector3 position, Quaternion rotation) => m_Kinematics.Reset(position, rotation);

        void FixedUpdate()
        {
            if (m_Drive == null || !m_Drive.IsInitialized) return;

            // Every step, also while disconnected, so the finite differences always span exactly one step.
            Transform baseLink = m_Drive.Root.transform;
            float dt = Time.fixedDeltaTime;
            ImuReading reading = m_Kinematics.Step(baseLink.position, baseLink.rotation, dt, Physics.gravity);
            if (!noiseFree)
            {
                reading.angularVelocity = m_Gyro.Apply(reading.angularVelocity, dt, m_Normal);
                reading.linearAcceleration = m_Accel.Apply(reading.linearAcceleration, dt, m_Normal);
            }
            Latest = reading;
            ReadingCount++;

            // The connector queues the publisher registration behind already queued messages, so anything published
            // while disconnected reaches the endpoint first and is rejected with "Not registered to publish topic".
            if (!RosBridgeStatus.IsConnected) return;
            m_Ros.Publish(topic, Message(reading, Time.fixedTimeAsDouble));
        }

        ImuMsg Message(ImuReading reading, double simTime)
        {
            int sec = (int)System.Math.Floor(simTime);
            uint nanosec = (uint)((simTime - sec) * 1e9);
            Vector3 gyroVariance = noiseFree ? Vector3.zero : m_Gyro.WhiteVariance;
            Vector3 accelVariance = noiseFree ? Vector3.zero : m_Accel.WhiteVariance;
            return new ImuMsg
            {
                header = new HeaderMsg { stamp = new TimeMsg { sec = sec, nanosec = nanosec }, frame_id = frameId },
                orientation = new QuaternionMsg(0, 0, 0, 0),
                orientation_covariance = new double[] { -1, 0, 0, 0, 0, 0, 0, 0, 0 },
                angular_velocity = new Vector3Msg(reading.angularVelocity.x, reading.angularVelocity.y, reading.angularVelocity.z),
                angular_velocity_covariance = Diagonal(gyroVariance),
                linear_acceleration = new Vector3Msg(reading.linearAcceleration.x, reading.linearAcceleration.y, reading.linearAcceleration.z),
                linear_acceleration_covariance = Diagonal(accelVariance),
            };
        }

        static double[] Diagonal(Vector3 d) => new double[] { d.x, 0, 0, 0, d.y, 0, 0, 0, d.z };
    }
}

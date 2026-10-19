using System.Collections.Generic;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Low-level drive for the URDF-imported delivery bot (ArticulationBody chain).
    /// Configures the joint drives from the README values and converts a
    /// (forward speed, steering angle) command into Ackermann steering targets
    /// and per-wheel velocity targets.
    ///
    /// Frames: the URDF importer maps ROS x (forward) to Unity +Z and ROS y (left) to Unity -X.
    /// Joint signs follow the URDF: positive steering = left turn, positive wheel velocity = forward.
    /// </summary>
    [DisallowMultipleComponent]
    public class DeliveryBotDrive : MonoBehaviour
    {
        [Header("URDF joint names")]
        public string frontLeftSteering = "front_left_steering_joint";
        public string frontRightSteering = "front_right_steering_joint";
        public string frontLeftWheel = "front_left_wheel_joint";
        public string frontRightWheel = "front_right_wheel_joint";
        public string rearLeftWheel = "rear_left_wheel_joint";
        public string rearRightWheel = "rear_right_wheel_joint";
        public string[] frontSuspension = { "front_left_suspension_joint", "front_right_suspension_joint" };
        public string[] rearSuspension = { "rear_left_suspension_joint", "rear_right_suspension_joint" };

        [Header("Steering servo (DS5160)")]
        public float steeringStiffness = 1000f;
        public float steeringDamping = 50f;
        public float steeringForceLimit = 5.9f;
        [Tooltip("Servo slew rate, deg/s (7 rad/s).")]
        public float steeringRate = 401f;
        [Tooltip("Max angle of the inner wheel, deg.")]
        public float innerSteeringLimit = 26f;
        [Tooltip("Max angle of the outer wheel, deg.")]
        public float outerSteeringLimit = 20f;

        [Header("Hub motors")]
        public float wheelDamping = 10f;
        public float wheelForceLimit = 10f;
        [Tooltip("Max wheel speed, rad/s.")]
        public float maxWheelSpeed = 35f;

        [Header("Suspension (passive)")]
        public float frontSuspensionStiffness = 333f;
        public float rearSuspensionStiffness = 210f;
        public float suspensionDamping = 10f;

        [Header("Geometry (0 = measure from the imported model)")]
        public float wheelbase;
        public float frontTrack;
        public float rearTrack;
        public float wheelRadius;

        [Header("Sign fixes, in case the import flips an axis")]
        public bool invertSteering;
        public bool invertWheels;

        [Tooltip("Multiplier on hub-motor and servo force limits (domain randomisation).")]
        public float torqueScale = 1f;

        public ArticulationBody Root { get; private set; }
        public bool IsInitialized { get; private set; }

        /// <summary>Largest bicycle-model steering angle that keeps both front wheels inside their limits, deg.</summary>
        public float MaxSteeringAngle { get; private set; }

        /// <summary>Top speed allowed by the hub motors, m/s.</summary>
        public float MaxSpeed => maxWheelSpeed * wheelRadius;

        public float ForwardSpeed => Vector3.Dot(Root.linearVelocity, Root.transform.forward);
        public float CommandedSteeringAngle => m_SteerCommand;

        ArticulationBody m_SteerL, m_SteerR, m_WheelFL, m_WheelFR, m_WheelRL, m_WheelRR;
        readonly List<ArticulationBody> m_FrontSusp = new List<ArticulationBody>();
        readonly List<ArticulationBody> m_RearSusp = new List<ArticulationBody>();
        ArticulationBody[] m_AllBodies;

        float m_SpeedCommand, m_SteerCommand;
        float m_SteerTargetL, m_SteerTargetR;

        void Awake()
        {
            Initialize();
        }

        public void Initialize()
        {
            if (IsInitialized)
                return;

            m_AllBodies = GetComponentsInChildren<ArticulationBody>();
            if (m_AllBodies.Length == 0)
            {
                Debug.LogError($"{name}: no ArticulationBody found. Import the URDF first.", this);
                return;
            }
            Root = System.Array.Find(m_AllBodies, b => b.isRoot);

            var byJoint = new Dictionary<string, ArticulationBody>();
            foreach (var joint in GetComponentsInChildren<UrdfJoint>())
            {
                if (!string.IsNullOrEmpty(joint.jointName) && joint.TryGetComponent(out ArticulationBody body))
                    byJoint[joint.jointName] = body;
            }

            ArticulationBody Find(string jointName)
            {
                if (byJoint.TryGetValue(jointName, out var body))
                    return body;
                Debug.LogError($"{name}: joint '{jointName}' not found in the imported robot.", this);
                return null;
            }

            m_SteerL = Find(frontLeftSteering);
            m_SteerR = Find(frontRightSteering);
            m_WheelFL = Find(frontLeftWheel);
            m_WheelFR = Find(frontRightWheel);
            m_WheelRL = Find(rearLeftWheel);
            m_WheelRR = Find(rearRightWheel);
            foreach (var j in frontSuspension) if (byJoint.TryGetValue(j, out var b)) m_FrontSusp.Add(b);
            foreach (var j in rearSuspension) if (byJoint.TryGetValue(j, out var b)) m_RearSusp.Add(b);

            if (Root == null || m_SteerL == null || m_SteerR == null || m_WheelFL == null || m_WheelFR == null
                || m_WheelRL == null || m_WheelRR == null)
                return;

            MeasureGeometry();
            ConfigureDrives();
            IsInitialized = true;
        }

        void MeasureGeometry()
        {
            Transform frame = Root.transform;
            Vector3 Local(ArticulationBody b) => frame.InverseTransformPoint(b.transform.position);

            if (wheelbase <= 0f)
                wheelbase = Mathf.Abs(Local(m_WheelFL).z + Local(m_WheelFR).z - Local(m_WheelRL).z - Local(m_WheelRR).z) * 0.5f;
            if (frontTrack <= 0f)
                frontTrack = Mathf.Abs(Local(m_WheelFL).x - Local(m_WheelFR).x);
            if (rearTrack <= 0f)
                rearTrack = Mathf.Abs(Local(m_WheelRL).x - Local(m_WheelRR).x);
            if (wheelRadius <= 0f)
            {
                // Wheel axis is horizontal, so the vertical half-extent of the collider is the radius.
                var colliders = m_WheelRL.GetComponentsInChildren<Collider>();
                if (colliders.Length > 0)
                {
                    Bounds bounds = colliders[0].bounds;
                    foreach (var c in colliders) bounds.Encapsulate(c.bounds);
                    wheelRadius = bounds.extents.y;
                }
                else
                {
                    wheelRadius = 0.1f;
                    Debug.LogWarning($"{name}: no wheel collider found, wheel radius defaults to {wheelRadius} m.", this);
                }
            }

            // Smallest turning radius (rear-axle centre) that keeps inner <= innerLimit and outer <= outerLimit.
            float half = frontTrack * 0.5f;
            float rInner = wheelbase / Mathf.Tan(innerSteeringLimit * Mathf.Deg2Rad) + half;
            float rOuter = wheelbase / Mathf.Tan(outerSteeringLimit * Mathf.Deg2Rad) - half;
            MaxSteeringAngle = Mathf.Atan(wheelbase / Mathf.Max(rInner, rOuter)) * Mathf.Rad2Deg;
        }

        void ConfigureDrives()
        {
            Root.immovable = false;

            foreach (var steer in new[] { m_SteerL, m_SteerR })
                SetDrive(steer, steeringStiffness, steeringDamping, steeringForceLimit * torqueScale);

            foreach (var wheel in new[] { m_WheelFL, m_WheelFR, m_WheelRL, m_WheelRR })
            {
                SetDrive(wheel, 0f, wheelDamping, wheelForceLimit * torqueScale);
                wheel.jointFriction = 0f;
                wheel.angularDamping = 0f;
                wheel.maxAngularVelocity = Mathf.Max(wheel.maxAngularVelocity, maxWheelSpeed * 1.2f);
            }

            foreach (var s in m_FrontSusp) SetDrive(s, frontSuspensionStiffness, suspensionDamping, float.MaxValue);
            foreach (var s in m_RearSusp) SetDrive(s, rearSuspensionStiffness, suspensionDamping, float.MaxValue);
        }

        static void SetDrive(ArticulationBody body, float stiffness, float damping, float forceLimit)
        {
            ArticulationDrive drive = body.xDrive;
            drive.driveType = ArticulationDriveType.Force;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            drive.target = 0f;
            drive.targetVelocity = 0f;
            body.xDrive = drive;
        }

        /// <summary>Re-applies force limits after <see cref="torqueScale"/> changed.</summary>
        public void ApplyTorqueScale()
        {
            if (!IsInitialized) return;
            foreach (var steer in new[] { m_SteerL, m_SteerR })
                SetForceLimit(steer, steeringForceLimit * torqueScale);
            foreach (var wheel in new[] { m_WheelFL, m_WheelFR, m_WheelRL, m_WheelRR })
                SetForceLimit(wheel, wheelForceLimit * torqueScale);
        }

        static void SetForceLimit(ArticulationBody body, float forceLimit)
        {
            ArticulationDrive drive = body.xDrive;
            drive.forceLimit = forceLimit;
            body.xDrive = drive;
        }

        /// <summary>
        /// Sets the drive command.
        /// </summary>
        /// <param name="speed">Forward speed of the rear-axle centre, m/s.</param>
        /// <param name="steeringAngle">Bicycle-model steering angle, deg, positive = left. Clamped to <see cref="MaxSteeringAngle"/>.</param>
        public void SetCommand(float speed, float steeringAngle)
        {
            m_SpeedCommand = Mathf.Clamp(speed, -MaxSpeed, MaxSpeed);
            m_SteerCommand = Mathf.Clamp(steeringAngle, -MaxSteeringAngle, MaxSteeringAngle);
        }

        void FixedUpdate()
        {
            if (!IsInitialized) return;

            float delta = m_SteerCommand * Mathf.Deg2Rad;
            float steerL, steerR;
            float sRL, sRR, sFL, sFR; // wheel path radius / centre radius

            if (Mathf.Abs(delta) < 1e-4f)
            {
                steerL = steerR = 0f;
                sRL = sRR = sFL = sFR = 1f;
            }
            else
            {
                float radius = wheelbase / Mathf.Tan(delta); // signed, + = turning left
                float hf = frontTrack * 0.5f, hr = rearTrack * 0.5f;
                steerL = Mathf.Atan(wheelbase / (radius - hf)) * Mathf.Rad2Deg;
                steerR = Mathf.Atan(wheelbase / (radius + hf)) * Mathf.Rad2Deg;
                float centre = Mathf.Abs(radius);
                sRL = (radius - hr) / radius;
                sRR = (radius + hr) / radius;
                sFL = Mathf.Sqrt(wheelbase * wheelbase + (radius - hf) * (radius - hf)) / centre;
                sFR = Mathf.Sqrt(wheelbase * wheelbase + (radius + hf) * (radius + hf)) / centre;
            }

            float maxStep = steeringRate * Time.fixedDeltaTime;
            m_SteerTargetL = Mathf.MoveTowards(m_SteerTargetL, steerL, maxStep);
            m_SteerTargetR = Mathf.MoveTowards(m_SteerTargetR, steerR, maxStep);
            float steerSign = invertSteering ? -1f : 1f;
            SetTarget(m_SteerL, steerSign * m_SteerTargetL);
            SetTarget(m_SteerR, steerSign * m_SteerTargetR);

            // Wheel angular speed, deg/s, capped at the motor limit while keeping the wheel speed ratios.
            float omega = m_SpeedCommand / wheelRadius;
            float maxRatio = Mathf.Max(Mathf.Max(sRL, sRR), Mathf.Max(sFL, sFR));
            if (Mathf.Abs(omega) * maxRatio > maxWheelSpeed)
                omega = Mathf.Sign(omega) * maxWheelSpeed / maxRatio;
            float w = omega * Mathf.Rad2Deg * (invertWheels ? -1f : 1f);
            SetTargetVelocity(m_WheelRL, w * sRL);
            SetTargetVelocity(m_WheelRR, w * sRR);
            SetTargetVelocity(m_WheelFL, w * sFL);
            SetTargetVelocity(m_WheelFR, w * sFR);
        }

        static void SetTarget(ArticulationBody body, float target)
        {
            ArticulationDrive drive = body.xDrive;
            drive.target = target;
            body.xDrive = drive;
        }

        static void SetTargetVelocity(ArticulationBody body, float targetVelocity)
        {
            ArticulationDrive drive = body.xDrive;
            drive.targetVelocity = targetVelocity;
            body.xDrive = drive;
        }

        /// <summary>Teleports the robot and zeros all joint states and velocities.</summary>
        public void ResetPose(Vector3 position, Quaternion rotation)
        {
            if (!IsInitialized) return;

            m_SpeedCommand = m_SteerCommand = 0f;
            m_SteerTargetL = m_SteerTargetR = 0f;

            Root.TeleportRoot(position, rotation);
            Root.linearVelocity = Vector3.zero;
            Root.angularVelocity = Vector3.zero;

            foreach (var body in m_AllBodies)
            {
                if (body.isRoot) continue;
                int dofs = body.dofCount;
                if (dofs == 0) continue;
                var zero = dofs == 1 ? new ArticulationReducedSpace(0f)
                    : dofs == 2 ? new ArticulationReducedSpace(0f, 0f)
                    : new ArticulationReducedSpace(0f, 0f, 0f);
                body.jointPosition = zero;
                body.jointVelocity = zero;
                body.jointForce = zero;
                ArticulationDrive drive = body.xDrive;
                drive.target = 0f;
                drive.targetVelocity = 0f;
                body.xDrive = drive;
            }
        }

        public IReadOnlyList<ArticulationBody> Bodies => m_AllBodies;
    }
}

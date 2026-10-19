using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

namespace DeliveryBot
{
    /// <summary>
    /// Point-to-point navigation agent: drive to the goal without hitting walls or obstacles.
    ///
    /// Actions (continuous): [0] throttle -1..1, [1] steering -1..1 (positive = left).
    /// Vector observations (10): goal direction/distance in the robot frame, velocities, steering, tilt.
    /// Lidar and radar rays come from RayPerceptionSensorComponent3D children.
    /// </summary>
    [RequireComponent(typeof(DeliveryBotDrive))]
    public class DeliveryBotAgent : Agent
    {
        public const int VectorObservationSize = 10;
        public const int ContinuousActionSize = 2;

        public TrainingArena arena;

        [Header("Driving")]
        [Tooltip("Speed at full throttle, m/s. Hub motors allow ~2.9 m/s.")]
        public float maxSpeed = 2f;
        [Tooltip("Fraction of maxSpeed allowed in reverse.")]
        public float reverseFraction = 0.3f;

        [Header("Task")]
        public float goalRadius = 0.6f;
        [Tooltip("Episode fails if the robot's up vector drops below this (cos of tilt).")]
        public float minUprightness = 0.6f;

        [Header("Rewards")]
        public float goalReward = 1f;
        public float collisionPenalty = -1f;
        public float flipPenalty = -1f;
        [Tooltip("Reward per metre of progress towards the goal.")]
        public float progressReward = 0.1f;

        [Header("Domain randomisation (+/- fraction)")]
        public float massRandomization = 0.15f;
        public float torqueRandomization = 0.15f;

        DeliveryBotDrive m_Drive;
        float m_PreviousDistance;
        bool m_Collided;
        readonly List<float> m_BaseMasses = new List<float>();
        SensorPoseRandomizer[] m_SensorRandomizers;

        public override void Initialize()
        {
            m_Drive = GetComponent<DeliveryBotDrive>();
            m_Drive.Initialize();
            m_SensorRandomizers = GetComponentsInChildren<SensorPoseRandomizer>();

            foreach (var body in m_Drive.Bodies)
            {
                m_BaseMasses.Add(body.mass);
                if (!body.TryGetComponent(out LinkCollisionReporter reporter))
                    reporter = body.gameObject.AddComponent<LinkCollisionReporter>();
                reporter.agent = this;
            }

            if (arena == null)
                arena = GetComponentInParent<TrainingArena>();
        }

        public override void OnEpisodeBegin()
        {
            RandomizeDynamics();

            if (arena != null)
                arena.ResetEpisode(this);

            m_Collided = false;
            m_PreviousDistance = DistanceToGoal();
        }

        void RandomizeDynamics()
        {
            var bodies = m_Drive.Bodies;
            for (int i = 0; i < bodies.Count; i++)
                if (m_BaseMasses[i] > 0.05f) // leave the near-massless sensor frames alone
                    bodies[i].mass = m_BaseMasses[i] * (1f + Random.Range(-massRandomization, massRandomization));

            m_Drive.torqueScale = 1f + Random.Range(-torqueRandomization, torqueRandomization);
            m_Drive.ApplyTorqueScale();

            foreach (var randomizer in m_SensorRandomizers)
                randomizer.Randomize();
        }

        public void PlaceAt(Vector3 position, Quaternion rotation)
        {
            m_Drive.ResetPose(position, rotation);
        }

        Transform Frame => m_Drive.Root.transform;

        float DistanceToGoal()
        {
            if (arena == null || arena.goal == null) return 0f;
            Vector3 d = arena.goal.position - Frame.position;
            d.y = 0f;
            return d.magnitude;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            Transform frame = Frame;
            float arenaScale = arena != null ? arena.size : 20f;

            Vector3 toGoal = arena != null && arena.goal != null ? arena.goal.position - frame.position : Vector3.zero;
            toGoal.y = 0f;
            Vector3 localGoal = frame.InverseTransformDirection(toGoal);
            float distance = toGoal.magnitude;
            Vector3 heading = distance > 1e-3f ? localGoal / distance : Vector3.forward;

            Vector3 localVel = frame.InverseTransformDirection(m_Drive.Root.linearVelocity);
            Vector3 localAngVel = frame.InverseTransformDirection(m_Drive.Root.angularVelocity);

            sensor.AddObservation(localGoal.x / arenaScale);                                           // 1
            sensor.AddObservation(localGoal.z / arenaScale);                                           // 2
            sensor.AddObservation(Mathf.Clamp01(distance / arenaScale));                               // 3
            sensor.AddObservation(heading.x);                                                          // 4
            sensor.AddObservation(heading.z);                                                          // 5
            sensor.AddObservation(localVel.x / maxSpeed);                                              // 6
            sensor.AddObservation(localVel.z / maxSpeed);                                              // 7
            sensor.AddObservation(localAngVel.y / Mathf.PI);                                           // 8
            sensor.AddObservation(m_Drive.CommandedSteeringAngle / Mathf.Max(1f, m_Drive.MaxSteeringAngle)); // 9
            sensor.AddObservation(frame.up.y);                                                         // 10
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            var a = actions.ContinuousActions;
            float throttle = Mathf.Clamp(a[0], -reverseFraction, 1f);
            float steer = Mathf.Clamp(a[1], -1f, 1f);
            m_Drive.SetCommand(throttle * maxSpeed, steer * m_Drive.MaxSteeringAngle);

            // Dense shaping: progress towards the goal, small per-step time penalty.
            float distance = DistanceToGoal();
            AddReward(progressReward * (m_PreviousDistance - distance));
            m_PreviousDistance = distance;
            if (MaxStep > 0)
                AddReward(-1f / MaxStep);

            if (m_Collided)
            {
                AddReward(collisionPenalty);
                EndEpisode();
            }
            else if (Frame.up.y < minUprightness || (arena != null && Frame.position.y < arena.transform.position.y - 1f))
            {
                AddReward(flipPenalty);
                EndEpisode();
            }
            else if (distance < goalRadius)
            {
                AddReward(goalReward);
                EndEpisode();
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            var a = actionsOut.ContinuousActions;
            var kb = Keyboard.current;
            if (kb == null) return;
            a[0] = (kb.wKey.isPressed || kb.upArrowKey.isPressed ? 1f : 0f) - (kb.sKey.isPressed || kb.downArrowKey.isPressed ? 1f : 0f);
            a[1] = (kb.aKey.isPressed || kb.leftArrowKey.isPressed ? 1f : 0f) - (kb.dKey.isPressed || kb.rightArrowKey.isPressed ? 1f : 0f);
        }

        internal void OnLinkCollision(Collision collision)
        {
            if (collision.collider.CompareTag(TrainingArena.ObstacleTag) || collision.collider.CompareTag(TrainingArena.WallTag))
                m_Collided = true;
        }
    }
}

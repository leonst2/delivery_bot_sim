using System.Collections.Generic;
using RosMessageTypes.DeliveryBotInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Spawns the cyclists of a scenario for /sim/load_scenario (<see cref="SimControl"/>) and removes them on
    /// /sim/reset. Publishes their true state on /ground_truth/dynamic_objects (TrackedObstacleArray, map) at
    /// <see cref="groundTruthRateHz"/> for evaluation only (delivery_bot_core#23): the stack must not subscribe to it.
    /// One per scene.
    /// </summary>
    public class DynamicObjectSpawner : MonoBehaviour, IDynamicObjectSpawner
    {
        [Header("Cyclists")]
        [Tooltip("Speeds outside this range (m/s) are clamped.")]
        public float minSpeed = 1f;
        public float maxSpeed = 8f;
        public Material cyclistMaterial;

        [Header("Ground truth (evaluation only)")]
        public string groundTruthTopic = "/ground_truth/dynamic_objects";
        public float groundTruthRateHz = 20f;

        readonly List<CyclistAgent> m_Agents = new List<CyclistAgent>();
        ROSConnection m_Ros;
        SensorCaptureClock m_Clock;

        /// <summary>The cyclists of the current scenario, including those that finished their route (inactive).</summary>
        public IReadOnlyList<CyclistAgent> Agents => m_Agents;

        void OnEnable() => DynamicObjectSpawners.Register(this);
        void OnDisable() => DynamicObjectSpawners.Unregister(this);

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<TrackedObstacleArrayMsg>(groundTruthTopic);
        }

        public void Spawn(SimScenario scenario, System.Random rng)
        {
            Clear();
            uint id = 1;
            foreach (var c in scenario.cyclists)
            {
                // Always two draws per cyclist, so one cyclist's jitter doesn't shift the others' numbers.
                double speedDraw = rng.NextDouble(), delayDraw = rng.NextDouble();
                float speed = Mathf.Clamp(c.speed + (float)(speedDraw * 2.0 - 1.0) * c.speedJitter, minSpeed, maxSpeed);
                float delay = Mathf.Max(0f, c.startDelay + (float)(delayDraw * 2.0 - 1.0) * c.startDelayJitter);
                m_Agents.Add(CyclistAgent.Create(transform, id++, c.name, c.route, speed, delay, c.loop, cyclistMaterial));
            }
        }

        public void Clear()
        {
            foreach (var agent in m_Agents)
                if (agent != null) Destroy(agent.gameObject);
            m_Agents.Clear();
        }

        void FixedUpdate()
        {
            // Ground truth first: it describes the current physics poses, which Advance then moves on.
            if (m_Clock.IsDue(Time.fixedTimeAsDouble, groundTruthRateHz))
                PublishGroundTruth(Time.fixedTimeAsDouble);

            float dt = Time.fixedDeltaTime;
            foreach (var agent in m_Agents)
            {
                if (agent == null || !agent.gameObject.activeSelf) continue;
                if (!agent.Advance(dt)) agent.gameObject.SetActive(false); // end of route
            }
        }

        void PublishGroundTruth(double stamp)
        {
            // Skipped while disconnected: messages queued before the publisher is registered are rejected.
            if (m_Ros == null || !RosBridgeStatus.IsConnected) return;
            var obstacles = new List<TrackedObstacleMsg>(m_Agents.Count);
            foreach (var agent in m_Agents)
            {
                if (agent == null || !agent.gameObject.activeSelf) continue;
                var p = agent.Position.To<FLU>();
                var v = agent.Velocity.To<FLU>();
                obstacles.Add(new TrackedObstacleMsg
                {
                    id = agent.Id,
                    position = new PointMsg(p.x, p.y, 0.0),
                    velocity = new Vector3Msg(v.x, v.y, 0.0),
                    radius = CyclistAgent.Radius,
                    position_covariance = new float[4],
                    predicted_position = new PointMsg(p.x, p.y, 0.0),
                    prediction_horizon = 0f,
                    age = agent.Age,
                });
            }
            m_Ros.Publish(groundTruthTopic, new TrackedObstacleArrayMsg
            {
                header = new HeaderMsg { stamp = SimCameraChannel.ToTimeMsg(stamp), frame_id = "map" },
                obstacles = obstacles.ToArray(),
            });
        }
    }
}

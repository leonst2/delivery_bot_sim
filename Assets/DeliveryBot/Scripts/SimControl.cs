using System;
using System.Threading.Tasks;
using RosMessageTypes.DeliveryBotInterfaces;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Remote control of the simulation for the scenario runner (delivery_bot_core#24). One per scene.
    /// - /sim/reset (ResetSim): removes dynamic objects, sets the seed, places the robot at the requested map pose and
    ///   answers once the robot is at rest.
    /// - /sim/load_scenario (LoadScenario): replaces the scenario objects with the named scenario's and sets the seed.
    /// - /sim/collisions (CollisionEvent): every contact between the robot and another object, except the wheels
    ///   rolling on the ground.
    /// A runner calls load_scenario once per scenario, then reset before every run. Both are idempotent.
    /// </summary>
    public class SimControl : MonoBehaviour
    {
        const string k_ResetService = "/sim/reset";
        const string k_LoadScenarioService = "/sim/load_scenario";
        const string k_CollisionsTopic = "/sim/collisions";
        const string k_DynamicStream = "dynamic_objects";

        [Header("Reset")]
        [Tooltip("Height above the requested pose the robot is placed at, so the wheels don't start inside the ground (m).")]
        public float dropHeight = 0.02f;
        [Tooltip("At rest: base_link moves slower than this (m/s) ...")]
        public float restLinearSpeed = 1e-3f;
        [Tooltip("... and turns slower than this (rad/s) ...")]
        public float restAngularSpeed = 1e-3f;
        [Tooltip("... for this many consecutive physics steps.")]
        public int restSteps = 10;
        [Tooltip("Sim seconds to wait for the robot to come to rest before the reset reports failure.")]
        public float restTimeout = 5f;

        [Header("Scenario")]
        public Material obstacleMaterial;

        [Header("Collisions")]
        [Tooltip("Contacts of wheel links whose normals are this close to vertical count as driving, not collisions.")]
        [Range(0f, 1f)] public float groundNormalUp = 0.7f;

        /// <summary>Every reported collision, also when ROS is disconnected (for checks and the UI).</summary>
        public event Action<CollisionEventMsg> CollisionReported;

        public SimScenario CurrentScenario { get; private set; }
        public DeliveryBotDrive Robot => FindRobot();

        ROSConnection m_Ros;
        DeliveryBotDrive m_Robot;
        Transform m_ScenarioRoot;

        TaskCompletionSource<ResetSimResponse> m_PendingReset;
        int m_RestStepCount, m_WaitSteps;
        Vector3 m_LastPosition;
        Quaternion m_LastRotation;

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.ImplementService<ResetSimRequest, ResetSimResponse>(k_ResetService, OnResetRequest);
            m_Ros.ImplementService<LoadScenarioRequest, LoadScenarioResponse>(k_LoadScenarioService, OnLoadScenarioRequest);
            m_Ros.RegisterPublisher<CollisionEventMsg>(k_CollisionsTopic);
            FindRobot();
        }

        void OnDestroy()
        {
            if (m_Robot != null) m_Robot.Collided -= OnCollided;
            m_PendingReset?.TrySetResult(new ResetSimResponse(false, "simulation stopped"));
        }

        DeliveryBotDrive FindRobot()
        {
            if (m_Robot != null) return m_Robot;
            m_Robot = FindFirstObjectByType<DeliveryBotDrive>();
            if (m_Robot != null) m_Robot.Collided += OnCollided;
            return m_Robot;
        }

        // ---------------------------------------------------------------- reset

        Task<ResetSimResponse> OnResetRequest(ResetSimRequest request)
        {
            var p = request.robot_pose.position;
            var q = request.robot_pose.orientation;
            Vector3 position = new Vector3<FLU>((float)p.x, (float)p.y, (float)p.z).toUnity;

            string note = "";
            double norm = Math.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            Quaternion rotation = Quaternion.identity;
            if (norm < 1e-6)
                note = " (orientation was all zero; used yaw 0)";
            else
                rotation = new Quaternion<FLU>((float)(q.x / norm), (float)(q.y / norm), (float)(q.z / norm), (float)(q.w / norm)).toUnity;

            var task = ResetAsync(position, rotation, request.seed);
            if (note.Length == 0) return task;
            return task.ContinueWith(t => new ResetSimResponse(t.Result.success, t.Result.message + note),
                TaskContinuationOptions.ExecuteSynchronously);
        }

        /// <summary>
        /// Resets the simulation: removes dynamic objects, sets the seed and places the robot (base_link) at
        /// <paramref name="position"/> with the yaw of <paramref name="rotation"/> (roll and pitch are dropped).
        /// Completes once the robot is at rest, or with success = false after <see cref="restTimeout"/>.
        /// </summary>
        public Task<ResetSimResponse> ResetAsync(Vector3 position, Quaternion rotation, uint seed)
        {
            var robot = FindRobot();
            if (robot == null || !robot.IsInitialized)
                return Task.FromResult(new ResetSimResponse(false, "no robot (DeliveryBotDrive) in the scene"));

            // A newer reset replaces one that is still settling.
            m_PendingReset?.TrySetResult(new ResetSimResponse(false, "superseded by a newer reset"));

            DynamicObjectSpawners.Current?.Clear();
            SimRandom.SetSeed(seed);

            float yaw = rotation.eulerAngles.y;
            robot.ResetPose(position + Vector3.up * dropHeight, Quaternion.Euler(0f, yaw, 0f));

            m_PendingReset = new TaskCompletionSource<ResetSimResponse>();
            m_RestStepCount = 0;
            m_WaitSteps = 0;
            m_LastPosition = robot.Root.transform.position;
            m_LastRotation = robot.Root.transform.rotation;
            return m_PendingReset.Task;
        }

        void FixedUpdate()
        {
            if (m_PendingReset == null) return;
            var root = m_Robot != null ? m_Robot.Root : null;
            if (root == null)
            {
                Finish(false, "robot destroyed during reset");
                return;
            }

            // From the pose change per step: the articulation root reports a small residual velocity even when its
            // pose doesn't move at all.
            var t = root.transform;
            float linear = Vector3.Distance(t.position, m_LastPosition) / Time.fixedDeltaTime;
            float angular = Quaternion.Angle(t.rotation, m_LastRotation) * Mathf.Deg2Rad / Time.fixedDeltaTime;
            m_LastPosition = t.position;
            m_LastRotation = t.rotation;
            m_RestStepCount = linear < restLinearSpeed && angular < restAngularSpeed ? m_RestStepCount + 1 : 0;
            m_WaitSteps++;
            float waited = m_WaitSteps * Time.fixedDeltaTime;
            if (m_RestStepCount >= restSteps)
            {
                var p = root.transform.position.To<FLU>();
                Finish(true, $"robot at rest at map ({p.x:F3}, {p.y:F3}) after {waited:F2} s, seed {SimRandom.Seed}");
            }
            else if (waited >= restTimeout)
            {
                Finish(false, $"robot not at rest after {restTimeout:F1} s (|v| = {linear:F4} m/s, |w| = {angular:F4} rad/s)");
            }
        }

        void Finish(bool success, string message)
        {
            var pending = m_PendingReset;
            m_PendingReset = null;
            pending.TrySetResult(new ResetSimResponse(success, message));
        }

        // ---------------------------------------------------------------- scenarios

        Task<LoadScenarioResponse> OnLoadScenarioRequest(LoadScenarioRequest request)
        {
            bool ok = LoadScenario(request.scenario_name, request.seed, out string message);
            return Task.FromResult(new LoadScenarioResponse(ok, message));
        }

        /// <summary>
        /// Replaces the scenario objects with those of <paramref name="scenarioName"/> and sets the seed. On failure
        /// nothing changes.
        /// </summary>
        public bool LoadScenario(string scenarioName, uint seed, out string message)
        {
            var scenario = SimScenario.Load(scenarioName);
            if (scenario == null)
            {
                message = $"unknown scenario '{scenarioName}'; known: {string.Join(", ", SimScenario.AvailableNames())}";
                return false;
            }
            var spawner = DynamicObjectSpawners.Current;
            if (scenario.NeedsDynamicObjects && spawner == null)
            {
                message = $"scenario '{scenario.name}' has {scenario.cyclists.Length} cyclist(s) and needs the dynamic " +
                          "object spawner (delivery_bot_sim#16), which isn't in this simulation yet";
                return false;
            }

            ClearScenario();
            SimRandom.SetSeed(seed);
            m_ScenarioRoot = new GameObject("Scenario: " + scenario.name).transform;
            m_ScenarioRoot.SetParent(transform, false);
            foreach (var box in scenario.staticObstacles)
                CreateObstacle(box);
            if (scenario.NeedsDynamicObjects)
                spawner.Spawn(scenario, SimRandom.Stream(k_DynamicStream));

            CurrentScenario = scenario;
            message = $"loaded '{scenario.name}': {scenario.staticObstacles.Length} static obstacle(s), " +
                      $"{scenario.cyclists.Length} cyclist(s), seed {seed}";
            return true;
        }

        void ClearScenario()
        {
            DynamicObjectSpawners.Current?.Clear();
            if (m_ScenarioRoot != null) Destroy(m_ScenarioRoot.gameObject);
            m_ScenarioRoot = null;
            CurrentScenario = null;
        }

        void CreateObstacle(SimScenario.StaticObstacle box)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = string.IsNullOrEmpty(box.name) ? "obstacle" : box.name;
            go.tag = "Obstacle";
            go.transform.SetParent(m_ScenarioRoot, false);
            // Map (FLU) → Unity: the box's own x (length) becomes Unity z, its y (width) Unity x, its height Unity y.
            go.transform.SetPositionAndRotation(
                new Vector3<FLU>(box.x, box.y, 0.5f * box.height).toUnity,
                new Quaternion<FLU>(0f, 0f, Mathf.Sin(0.5f * box.yaw), Mathf.Cos(0.5f * box.yaw)).toUnity);
            go.transform.localScale = new Vector3(box.width, box.height, box.length);
            if (obstacleMaterial != null)
                go.GetComponent<Renderer>().sharedMaterial = obstacleMaterial;
        }

        // ---------------------------------------------------------------- collisions

        void OnCollided(Collision collision)
        {
            if (collision.contactCount == 0) return;
            var contact = collision.GetContact(0);
            var link = contact.thisCollider != null ? contact.thisCollider.GetComponentInParent<ArticulationBody>() : null;
            string linkName = link != null ? link.name : "unknown";
            if (IsDriving(collision, linkName)) return;

            var msg = new CollisionEventMsg
            {
                header = new HeaderMsg { stamp = SimCameraChannel.ToTimeMsg(Time.fixedTimeAsDouble), frame_id = "map" },
                robot_link = linkName,
                other_object = collision.gameObject.name,
                other_tag = collision.gameObject.tag,
                impulse = collision.impulse.magnitude,
            };
            CollisionReported?.Invoke(msg);
            if (m_Ros != null && RosBridgeStatus.IsConnected)
                m_Ros.Publish(k_CollisionsTopic, msg);
        }

        /// <summary>A wheel touching a surface from above: the robot driving, not a collision.</summary>
        bool IsDriving(Collision collision, string linkName)
        {
            if (!linkName.EndsWith("_wheel", StringComparison.Ordinal)) return false;
            for (int i = 0; i < collision.contactCount; i++)
                if (Mathf.Abs(Vector3.Dot(collision.GetContact(i).normal, Vector3.up)) < groundNormalUp)
                    return false;
            return true;
        }
    }
}

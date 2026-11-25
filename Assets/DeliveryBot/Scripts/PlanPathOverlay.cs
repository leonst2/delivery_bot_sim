using System.Collections.Generic;
using RosMessageTypes.DeliveryBotInterfaces;
using RosMessageTypes.Nav;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.InputSystem;

namespace DeliveryBot
{
    /// <summary>
    /// Draws the planner's /plan (nav_msgs/Path, frame map) as flat lines just above the ground in the 3D scene and
    /// highlights the current step (the next <see cref="stepSize"/> metres from the robot) in a second colour.
    /// Reverse segments use the dashed material. The line is replaced on every new plan and cleared when
    /// /grid_step/state reports DONE or FAILED. <see cref="testPathKey"/> toggles a hard-coded test path in front of
    /// the robot, so the overlay can be checked without ROS. Display only: nothing here plans or decides.
    /// </summary>
    public class PlanPathOverlay : MonoBehaviour
    {
        const string k_PlanTopic = "/plan";
        const string k_GridStepTopic = "/grid_step/state";
        const float k_SearchInterval = 1f;     // s between searches for the robot
        const float k_RebuildDistance = 0.01f; // m the projection must move before the runs are rebuilt

        [Header("Path")]
        [Tooltip("Length of one grid step (m); the stack's step_size.")]
        public float stepSize = 0.5f;
        [Tooltip("Height of the line above the path's z (m).")]
        public float heightAboveGround = 0.05f;
        public float lineWidth = 0.1f;

        [Header("Style")]
        public Color pathColor = new Color(0.1f, 0.85f, 1f, 1f);
        public Color stepColor = new Color(1f, 0.55f, 0.05f, 1f);
        public Material solidMaterial;
        [Tooltip("Used for reverse segments; tiled along the line.")]
        public Material dashedMaterial;
        [Tooltip("Length of one dash plus gap (m).")]
        public float dashPeriod = 0.3f;

        [Header("Test path (no ROS)")]
        public bool useTestPath;
        public Key testPathKey = Key.P;

        readonly PlanPathGeometry m_RosPlan = new PlanPathGeometry();
        readonly PlanPathGeometry m_TestPlan = new PlanPathGeometry();
        readonly List<PlanPathGeometry.Run> m_Runs = new List<PlanPathGeometry.Run>();
        readonly List<LineRenderer> m_Lines = new List<LineRenderer>();
        NativeArray<Vector3> m_Positions;

        // Scratch buffers for converting messages; grown, never shrunk.
        Vector2[] m_Xy = new Vector2[0];
        float[] m_Heights = new float[0];
        float[] m_Yaw = new float[0];

        bool m_HasRosPlan;
        bool m_TestPlanBuilt;
        bool m_Dirty;
        PlanPathGeometry m_Shown;
        float m_ShownStepStart = float.NaN;

        DeliveryBotDrive m_Robot;
        Transform m_BaseLink;
        float m_NextSearch;

        void Start()
        {
            var ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<PathMsg>(k_PlanTopic, OnPlan);
            ros.Subscribe<GridStepStateMsg>(k_GridStepTopic, OnGridStepState);
            m_Positions = new NativeArray<Vector3>(256, Allocator.Persistent);
        }

        void OnDestroy()
        {
            if (m_Positions.IsCreated) m_Positions.Dispose();
        }

        void OnValidate()
        {
            stepSize = Mathf.Max(0f, stepSize);
            lineWidth = Mathf.Max(0.001f, lineWidth);
            dashPeriod = Mathf.Max(0.01f, dashPeriod);
            m_Dirty = true;
        }

        // ---------------------------------------------------------------- ROS

        void OnPlan(PathMsg msg)
        {
            int count = msg.poses?.Length ?? 0;
            Grow(count);
            for (int i = 0; i < count; i++)
            {
                var pose = msg.poses[i].pose;
                m_Xy[i] = new Vector2((float)pose.position.x, (float)pose.position.y);
                m_Heights[i] = (float)pose.position.z;
                var q = pose.orientation;
                m_Yaw[i] = (float)System.Math.Atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
            }
            m_RosPlan.SetPath(m_Xy, m_Heights, m_Yaw, count);
            m_HasRosPlan = count >= 2;
            useTestPath = false; // a real plan replaces the test path
            m_Dirty = true;
        }

        void OnGridStepState(GridStepStateMsg msg)
        {
            if (msg.state != GridStepStateMsg.DONE && msg.state != GridStepStateMsg.FAILED) return;
            m_HasRosPlan = false;
            m_Dirty = true;
        }

        void Grow(int count)
        {
            if (m_Xy.Length >= count) return;
            int size = Mathf.NextPowerOfTwo(Mathf.Max(count, 16));
            m_Xy = new Vector2[size];
            m_Heights = new float[size];
            m_Yaw = new float[size];
        }

        // ---------------------------------------------------------------- update

        void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard[testPathKey].wasPressedThisFrame)
            {
                useTestPath = !useTestPath;
                m_TestPlanBuilt = false; // rebuilt in front of the robot's current pose
                m_Dirty = true;
            }

            FindRobot();
            if (useTestPath && !m_TestPlanBuilt)
                BuildTestPlan();

            var shown = useTestPath && m_TestPlanBuilt ? m_TestPlan : m_HasRosPlan ? m_RosPlan : null;
            if (shown != m_Shown)
            {
                m_Shown = shown;
                m_Dirty = true;
            }
            if (shown == null)
            {
                if (m_Dirty) Draw(0);
                m_Dirty = false;
                return;
            }

            // Current step: from the robot's projection onto the path. Without a robot, from the path's start.
            float stepStart = m_BaseLink != null ? shown.Project(RobotXY()) : 0f;
            if (!m_Dirty && Mathf.Abs(stepStart - m_ShownStepStart) < k_RebuildDistance) return;
            m_ShownStepStart = stepStart;
            m_Dirty = false;
            Draw(shown.BuildRuns(stepStart, stepSize, m_Runs));
        }

        void FindRobot()
        {
            if (m_BaseLink != null || Time.unscaledTime < m_NextSearch) return;
            m_NextSearch = Time.unscaledTime + k_SearchInterval;
            if (m_Robot == null) m_Robot = FindFirstObjectByType<DeliveryBotDrive>();
            if (m_Robot == null) return;
            // The base_link body, not the robot root: the body is what moves under physics.
            foreach (var t in m_Robot.GetComponentsInChildren<Transform>(true))
            {
                if (t.name != "base_link") continue;
                m_BaseLink = t;
                break;
            }
            if (m_BaseLink == null) m_BaseLink = m_Robot.transform;
        }

        Vector2 RobotXY()
        {
            var p = m_BaseLink.position.To<FLU>();
            return new Vector2(p.x, p.y);
        }

        /// <summary>3 m straight, a 90° left arc (R = 1.5 m), then 2 m in reverse, starting at the robot.</summary>
        void BuildTestPlan()
        {
            if (m_BaseLink == null) return;
            const float spacing = 0.1f, straight = 3f, radius = 1.5f, back = 2f;
            Vector2 p = RobotXY();
            var forward = m_BaseLink.forward.To<FLU>();
            float yaw = Mathf.Atan2(forward.y, forward.x);
            float z = m_BaseLink.position.To<FLU>().z;

            int arcPoints = Mathf.CeilToInt(0.5f * Mathf.PI * radius / spacing);
            int count = Mathf.CeilToInt(straight / spacing) + arcPoints + Mathf.CeilToInt(back / spacing) + 1;
            Grow(count);
            int n = 0;
            void Add(Vector2 xy, float heading)
            {
                m_Xy[n] = xy;
                m_Heights[n] = z;
                m_Yaw[n] = heading;
                n++;
            }

            Vector2 dir = new Vector2(Mathf.Cos(yaw), Mathf.Sin(yaw));
            int straightPoints = Mathf.CeilToInt(straight / spacing);
            for (int i = 0; i < straightPoints; i++)
                Add(p + dir * (i * spacing), yaw);
            Vector2 arcStart = p + dir * straight;
            Vector2 centre = arcStart + radius * new Vector2(-dir.y, dir.x); // left of the heading
            for (int i = 0; i < arcPoints; i++)
            {
                float a = 0.5f * Mathf.PI * i / arcPoints;
                Add(centre + radius * new Vector2(Mathf.Sin(yaw + a), -Mathf.Cos(yaw + a)), yaw + a);
            }
            float endYaw = yaw + 0.5f * Mathf.PI;
            Vector2 arcEnd = centre + radius * new Vector2(Mathf.Sin(endYaw), -Mathf.Cos(endYaw));
            Vector2 endDir = new Vector2(Mathf.Cos(endYaw), Mathf.Sin(endYaw));
            int backPoints = Mathf.CeilToInt(back / spacing);
            for (int i = 0; i <= backPoints; i++)
                Add(arcEnd - endDir * (i * spacing), endYaw); // reversing: moving against the heading

            m_TestPlan.SetPath(m_Xy, m_Heights, m_Yaw, n);
            m_TestPlanBuilt = true;
            m_Dirty = true;
        }

        // ---------------------------------------------------------------- drawing

        void Draw(int runCount)
        {
            for (int r = 0; r < runCount; r++)
            {
                var run = m_Runs[r];
                var line = GetLine(r);
                line.gameObject.SetActive(true);
                int n = run.points.Count;
                if (m_Positions.Length < n)
                {
                    m_Positions.Dispose();
                    m_Positions = new NativeArray<Vector3>(Mathf.NextPowerOfTwo(n), Allocator.Persistent);
                }
                for (int i = 0; i < n; i++)
                {
                    var p = run.points[i];
                    m_Positions[i] = new Vector3<FLU>(p.x, p.y, run.heights[i] + heightAboveGround).toUnity;
                }
                line.positionCount = n;
                line.SetPositions(m_Positions.GetSubArray(0, n));
                Color color = run.inStep ? stepColor : pathColor;
                line.startColor = line.endColor = color;
                line.widthMultiplier = lineWidth;
                line.sharedMaterial = run.reverse && dashedMaterial != null ? dashedMaterial : solidMaterial;
                line.textureMode = run.reverse ? LineTextureMode.Tile : LineTextureMode.Stretch;
                line.textureScale = new Vector2(1f / dashPeriod, 1f);
            }
            for (int r = runCount; r < m_Lines.Count; r++)
                m_Lines[r].gameObject.SetActive(false);
        }

        LineRenderer GetLine(int index)
        {
            while (m_Lines.Count <= index)
            {
                var go = new GameObject($"PlanRun{m_Lines.Count}") { hideFlags = HideFlags.DontSave };
                go.transform.SetParent(transform, false);
                // Flat on the ground: TransformZ alignment with the line object's z pointing up.
                go.transform.rotation = Quaternion.LookRotation(Vector3.up, Vector3.forward);
                var line = go.AddComponent<LineRenderer>();
                line.useWorldSpace = true;
                line.alignment = LineAlignment.TransformZ;
                line.numCornerVertices = 2;
                line.numCapVertices = 0;
                line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                line.receiveShadows = false;
                m_Lines.Add(line);
            }
            return m_Lines[index];
        }
    }
}

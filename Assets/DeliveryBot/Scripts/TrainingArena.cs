using System.Collections.Generic;
using Unity.MLAgents;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Square arena with walls, randomly placed box obstacles and a goal marker.
    /// Duplicate the arena prefab (offset in x/z) to train several robots in parallel.
    ///
    /// Curriculum: the environment parameters "goal_distance" and "obstacle_count"
    /// (see config/delivery_bot_ppo.yaml) override the max goal distance and obstacle count.
    /// </summary>
    public class TrainingArena : MonoBehaviour
    {
        public const string ObstacleTag = "Obstacle";
        public const string WallTag = "Wall";

        [Tooltip("Inner side length of the square arena, m.")]
        public float size = 20f;
        public Transform goal;
        public Transform obstacleRoot;

        [Header("Obstacles")]
        public int maxObstacles = 12;
        public Vector2 obstacleSize = new Vector2(0.3f, 1.5f);
        public float obstacleHeight = 1f;

        [Header("Spawning")]
        public float minGoalDistance = 2f;
        public float maxGoalDistance = 15f;
        [Tooltip("Free radius kept around the robot spawn and the goal, m.")]
        public float clearance = 1.2f;
        [Tooltip("Spawn height above the floor (base_link origin is the ground point under the rear axle).")]
        public float spawnHeight = 0.02f;

        readonly List<Transform> m_Obstacles = new List<Transform>();
        readonly List<(Vector2 centre, float radius)> m_Occupied = new List<(Vector2, float)>();

        void Awake()
        {
            if (obstacleRoot == null)
            {
                obstacleRoot = new GameObject("Obstacles").transform;
                obstacleRoot.SetParent(transform, false);
            }
            for (int i = 0; i < maxObstacles; i++)
            {
                var obstacle = GameObject.CreatePrimitive(PrimitiveType.Cube);
                obstacle.name = $"Obstacle_{i}";
                obstacle.tag = ObstacleTag;
                obstacle.transform.SetParent(obstacleRoot, false);
                obstacle.SetActive(false);
                m_Obstacles.Add(obstacle.transform);
            }
        }

        public void ResetEpisode(DeliveryBotAgent agent)
        {
            var parameters = Academy.Instance.EnvironmentParameters;
            float goalDistance = parameters.GetWithDefault("goal_distance", maxGoalDistance);
            int obstacleCount = Mathf.Clamp(Mathf.RoundToInt(parameters.GetWithDefault("obstacle_count", maxObstacles)), 0, maxObstacles);

            m_Occupied.Clear();
            float half = size * 0.5f;

            // Robot.
            Vector2 spawn = RandomPoint(half - clearance);
            m_Occupied.Add((spawn, clearance));
            Quaternion yaw = transform.rotation * Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
            agent.PlaceAt(ToWorld(spawn, spawnHeight), yaw);

            // Goal.
            Vector2 goalPos = spawn;
            float maxDist = Mathf.Max(minGoalDistance, goalDistance);
            for (int attempt = 0; attempt < 50; attempt++)
            {
                float angle = Random.Range(0f, 2f * Mathf.PI);
                float dist = Random.Range(minGoalDistance, maxDist);
                Vector2 candidate = spawn + new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * dist;
                if (Mathf.Abs(candidate.x) < half - 0.5f && Mathf.Abs(candidate.y) < half - 0.5f)
                {
                    goalPos = candidate;
                    break;
                }
            }
            m_Occupied.Add((goalPos, clearance));
            if (goal != null)
                goal.position = ToWorld(goalPos, 0.01f);

            // Obstacles.
            for (int i = 0; i < m_Obstacles.Count; i++)
            {
                Transform obstacle = m_Obstacles[i];
                if (i >= obstacleCount)
                {
                    obstacle.gameObject.SetActive(false);
                    continue;
                }

                Vector2 footprint = new Vector2(Random.Range(obstacleSize.x, obstacleSize.y), Random.Range(obstacleSize.x, obstacleSize.y));
                float radius = footprint.magnitude * 0.5f;
                bool placed = false;
                for (int attempt = 0; attempt < 30 && !placed; attempt++)
                {
                    Vector2 p = RandomPoint(half - radius);
                    if (IsFree(p, radius))
                    {
                        m_Occupied.Add((p, radius));
                        obstacle.localScale = new Vector3(footprint.x, obstacleHeight, footprint.y);
                        obstacle.position = ToWorld(p, obstacleHeight * 0.5f);
                        obstacle.rotation = transform.rotation * Quaternion.Euler(0f, Random.Range(0f, 90f), 0f);
                        placed = true;
                    }
                }
                obstacle.gameObject.SetActive(placed);
            }
        }

        bool IsFree(Vector2 p, float radius)
        {
            foreach (var (centre, r) in m_Occupied)
                if ((p - centre).sqrMagnitude < (r + radius) * (r + radius))
                    return false;
            return true;
        }

        static Vector2 RandomPoint(float extent)
        {
            extent = Mathf.Max(0f, extent);
            return new Vector2(Random.Range(-extent, extent), Random.Range(-extent, extent));
        }

        Vector3 ToWorld(Vector2 p, float height) => transform.TransformPoint(new Vector3(p.x, height, p.y));
    }
}

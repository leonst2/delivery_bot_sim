using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// A cyclist riding a polyline route at constant speed: a kinematic Rigidbody on the Dynamic layer, tagged Obstacle,
    /// so the LiDAR sees it, the static map export skips it and robot contacts are reported. Its motion is a pure
    /// function of the physics steps since it was spawned (deterministic), and it doesn't react to the robot: it keeps
    /// riding while the robot is halted. Only a crash stops it: after touching the robot it stays where it is, instead
    /// of shoving the robot along its route with infinite mass.
    ///
    /// Driven by <see cref="DynamicObjectSpawner"/>, which calls <see cref="Advance"/> once per physics step after it
    /// has read the ground truth, so <see cref="Position"/> and <see cref="Velocity"/> always describe the current
    /// physics pose.
    /// </summary>
    public class CyclistAgent : MonoBehaviour
    {
        public const string DynamicLayer = "Dynamic";
        public const float Length = 1.8f, Width = 0.6f, Height = 1.7f; // m, bike plus rider

        /// <summary>Radius of the circle around the footprint (m), as reported in the ground truth.</summary>
        public static float Radius => 0.5f * Mathf.Sqrt(Length * Length + Width * Width);

        public uint Id { get; private set; }
        /// <summary>Physics position (Unity world, ground point under the centre).</summary>
        public Vector3 Position => m_Body.position;
        /// <summary>Velocity of the motion that produced <see cref="Position"/> (Unity world, m/s); 0 while waiting.</summary>
        public Vector3 Velocity { get; private set; }
        /// <summary>Sim seconds since the cyclist started riding (0 while waiting for its start delay).</summary>
        public float Age { get; private set; }
        public float Speed => m_Speed;
        /// <summary>True after the cyclist has touched the robot; it no longer moves.</summary>
        public bool Crashed { get; private set; }

        Rigidbody m_Body;
        Vector3[] m_Route;  // Unity world, y = 0
        float[] m_Arc;      // cumulative arc length per waypoint
        float m_Speed, m_StartDelay;
        bool m_Loop;
        int m_Steps;

        /// <summary>Creates a cyclist at the start of <paramref name="mapRoute"/> (map frame, m).</summary>
        public static CyclistAgent Create(Transform parent, uint id, string name, SimScenario.Waypoint[] mapRoute,
            float speed, float startDelay, bool loop, Material material)
        {
            var go = new GameObject(string.IsNullOrEmpty(name) ? $"cyclist_{id}" : name);
            go.transform.SetParent(parent, false);
            go.tag = "Obstacle";
            int layer = LayerMask.NameToLayer(DynamicLayer);
            if (layer < 0) Debug.LogError($"[Cyclist] Layer '{DynamicLayer}' is missing (ProjectSettings > Tags and Layers).");
            else go.layer = layer;

            var agent = go.AddComponent<CyclistAgent>();
            agent.Id = id;
            agent.m_Speed = speed;
            agent.m_StartDelay = startDelay;
            agent.m_Loop = loop;
            agent.SetRoute(mapRoute);

            var collider = go.AddComponent<BoxCollider>();
            collider.size = new Vector3(Width, Height, Length);
            collider.center = new Vector3(0f, 0.5f * Height, 0f);
            agent.m_Body = go.AddComponent<Rigidbody>();
            agent.m_Body.isKinematic = true;
            agent.m_Body.interpolation = RigidbodyInterpolation.None;
            agent.m_Body.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

            BuildVisual(go.transform, material, layer);

            agent.Pose(0f, out Vector3 position, out Quaternion rotation, out _);
            go.transform.SetPositionAndRotation(position, rotation);
            agent.m_Body.position = position;
            agent.m_Body.rotation = rotation;
            return agent;
        }

        void SetRoute(SimScenario.Waypoint[] mapRoute)
        {
            int n = mapRoute != null ? mapRoute.Length : 0;
            if (n == 0)
            {
                mapRoute = new[] { new SimScenario.Waypoint() };
                n = 1;
            }
            m_Route = new Vector3[n];
            m_Arc = new float[n];
            for (int i = 0; i < n; i++)
            {
                m_Route[i] = new Vector3<FLU>(mapRoute[i].x, mapRoute[i].y, 0f).toUnity;
                m_Arc[i] = i == 0 ? 0f : m_Arc[i - 1] + Vector3.Distance(m_Route[i - 1], m_Route[i]);
            }
        }

        float RouteLength => m_Arc[m_Arc.Length - 1];

        /// <summary>
        /// One physics step: moves the body to its pose at the end of this step. Returns false once the cyclist has
        /// reached the end of a non-looping route (the spawner then deactivates it).
        /// </summary>
        public bool Advance(float dt)
        {
            m_Steps++;
            if (Crashed)
            {
                Velocity = Vector3.zero;
                return true;
            }
            float t = m_Steps * dt - m_StartDelay;
            float s = Mathf.Max(0f, t) * m_Speed;
            if (s > RouteLength && !m_Loop) return false;
            if (m_Loop && RouteLength > 0f) s %= RouteLength;

            Pose(s, out Vector3 position, out Quaternion rotation, out Vector3 direction);
            m_Body.MovePosition(position);
            m_Body.MoveRotation(rotation);
            Velocity = t > 0f ? direction * m_Speed : Vector3.zero;
            Age = Mathf.Max(0f, t);
            return true;
        }

        void OnCollisionEnter(Collision collision)
        {
            if (collision.gameObject.layer == LayerMask.NameToLayer("Robot")) Crashed = true;
        }

        /// <summary>Pose at arc length <paramref name="s"/>: position on the route, heading along the segment.</summary>
        void Pose(float s, out Vector3 position, out Quaternion rotation, out Vector3 direction)
        {
            int last = m_Route.Length - 1;
            if (last == 0)
            {
                position = m_Route[0];
                direction = Vector3.forward;
                rotation = Quaternion.identity;
                return;
            }
            int i = 0;
            while (i < last - 1 && s > m_Arc[i + 1]) i++;
            float segment = m_Arc[i + 1] - m_Arc[i];
            float u = segment > 1e-6f ? Mathf.Clamp01((s - m_Arc[i]) / segment) : 0f;
            position = Vector3.Lerp(m_Route[i], m_Route[i + 1], u);
            direction = segment > 1e-6f ? (m_Route[i + 1] - m_Route[i]) / segment : Vector3.forward;
            rotation = Quaternion.LookRotation(direction, Vector3.up);
        }

        /// <summary>A simple bike and rider from primitives, without colliders (the root's box is the collider).</summary>
        static void BuildVisual(Transform root, Material material, int layer)
        {
            void Part(PrimitiveType type, Vector3 position, Vector3 scale, Vector3 euler)
            {
                var part = GameObject.CreatePrimitive(type);
                Object.DestroyImmediate(part.GetComponent<Collider>()); // before it joins the body as a compound collider
                part.layer = layer >= 0 ? layer : 0;
                part.transform.SetParent(root, false);
                part.transform.SetLocalPositionAndRotation(position, Quaternion.Euler(euler));
                part.transform.localScale = scale;
                if (material != null) part.GetComponent<Renderer>().sharedMaterial = material;
            }

            const float wheel = 0.68f; // m diameter
            Part(PrimitiveType.Cylinder, new Vector3(0f, 0.5f * wheel, 0.55f), new Vector3(wheel, 0.02f, wheel), new Vector3(0f, 0f, 90f));
            Part(PrimitiveType.Cylinder, new Vector3(0f, 0.5f * wheel, -0.55f), new Vector3(wheel, 0.02f, wheel), new Vector3(0f, 0f, 90f));
            Part(PrimitiveType.Cube, new Vector3(0f, 0.6f, 0f), new Vector3(0.06f, 0.06f, 1.1f), Vector3.zero);
            Part(PrimitiveType.Capsule, new Vector3(0f, 1.25f, -0.1f), new Vector3(0.4f, 0.45f, 0.3f), new Vector3(20f, 0f, 0f));
            Part(PrimitiveType.Sphere, new Vector3(0f, 1.6f, 0.05f), Vector3.one * 0.22f, Vector3.zero);
        }
    }
}

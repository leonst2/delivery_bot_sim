using System;
using RosMessageTypes.DeliveryBotInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.UI;

namespace DeliveryBot
{
    /// <summary>
    /// HUD LiDAR panel (LidarSlot): the latest 2D scan as a robot-centred polar plot (forward up), switchable between
    /// the raw scan, the stack's filtered scan and a synthetic mock, with the tracked obstacles and the safety stop
    /// zone on top. Display only: no filtering, detection or safety logic happens here.
    /// Built by Tools > Delivery Bot > Build LiDAR Panel.
    /// </summary>
    public class LidarPanel : MonoBehaviour
    {
        public enum Source { Raw, Filtered, Mock }

        const string k_ScanTopic = "/scan";
        const string k_FilteredTopic = "/scan/filtered";
        const string k_ObstaclesTopic = "/tracked_obstacles";
        const string k_SafetyStateTopic = "/safety/state";
        const string k_StopZoneTopic = "/safety/stop_zone";
        const string k_MapFrame = "map";
        const string k_BaseFrame = "base_link";
        const int k_MaxObstacles = 64;
        const int k_MaxZonePoints = 64;
        const float k_SearchInterval = 1f; // s between searches for the lidar and the robot
        const float k_MockPredictionHorizon = 1.5f; // s, like the tracker's prediction_horizon (one grid step)
        static readonly Vector2 k_UrdfLidarOffset = new Vector2(0.455f, 0f); // base_link → lidar_link (URDF)

        [Header("View")]
        public Source source = Source.Raw;
        public Key cycleKey = Key.L;
        public bool showObstacles = true;
        public bool showStopZone = true;
        public LidarPlotGraphic plot;
        public Button rawButton, filteredButton, mockButton, obstaclesButton, stopZoneButton;
        public Text title, caption, statusText;

        [Header("Safety state colours")]
        public Color zoneUnknown = new Color(1f, 1f, 1f, 0.6f);
        public Color zoneOk = new Color(0.3f, 0.9f, 0.4f, 1f);
        public Color zoneSlow = new Color(1f, 0.75f, 0.2f, 1f);
        public Color zoneStop = new Color(1f, 0.2f, 0.2f, 1f);

        ILaserScanSource m_Unity;      // LidarScanPublisher in the scene
        ILaserScanSource m_RawRos;     // /scan, created only when there is no LidarScanPublisher
        ILaserScanSource m_Filtered;   // /scan/filtered
        MockLaserScanSource m_Mock;
        ILaserScanSource m_Shown;
        int m_ShownVersion = -1;
        RateEstimator m_Rate;
        float m_NextSearch;

        DeliveryBotDrive m_Robot;
        Vector2 m_LidarOffset = k_UrdfLidarOffset;

        TrackedObstacleArrayMsg m_ObstacleMsg;
        bool m_ObstaclesDirty;
        readonly LidarPlotGraphic.Obstacle[] m_Obstacles = new LidarPlotGraphic.Obstacle[k_MaxObstacles];
        int m_ObstacleCount;

        readonly Vector2[] m_Zone = new Vector2[k_MaxZonePoints];
        int m_ZoneCount;
        string m_SafetyState;
        bool m_ZoneDirty;
        bool m_WarnedFrame;

        int m_ShownTextKey = -1;

        void Awake()
        {
            HudPanelUtil.HideSlotPlaceholder(transform);
            HudPanelUtil.EnsureEventSystem();
            if (rawButton != null) rawButton.onClick.AddListener(() => source = Source.Raw);
            if (filteredButton != null) filteredButton.onClick.AddListener(() => source = Source.Filtered);
            if (mockButton != null) mockButton.onClick.AddListener(() => source = Source.Mock);
            if (obstaclesButton != null) obstaclesButton.onClick.AddListener(() =>
            {
                showObstacles = !showObstacles;
                m_ObstaclesDirty = true;
            });
            if (stopZoneButton != null) stopZoneButton.onClick.AddListener(() =>
            {
                showStopZone = !showStopZone;
                m_ZoneDirty = true;
            });
        }

        void Start()
        {
            var ros = ROSConnection.GetOrCreateInstance();
            m_Filtered = new RosLaserScanSource(k_FilteredTopic);
            m_Mock = new MockLaserScanSource();
            ros.Subscribe<TrackedObstacleArrayMsg>(k_ObstaclesTopic, msg =>
            {
                m_ObstacleMsg = msg;
                m_ObstaclesDirty = true;
            });
            ros.Subscribe<StringMsg>(k_SafetyStateTopic, msg =>
            {
                m_SafetyState = msg.data;
                m_ZoneDirty = true;
            });
            ros.Subscribe<PolygonStampedMsg>(k_StopZoneTopic, OnStopZone);
        }

        void OnDestroy()
        {
            m_Unity?.Dispose();
            m_RawRos?.Dispose();
            m_Filtered?.Dispose();
            m_Mock?.Dispose();
        }

        void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard[cycleKey].wasPressedThisFrame)
                source = (Source)(((int)source + 1) % 3);

            FindSceneObjects();
            var shown = CurrentSource();
            shown.Poll();
            if (shown != m_Shown)
            {
                m_Shown = shown;
                m_ShownVersion = -1;
                m_Rate.Reset();
                m_ObstaclesDirty = m_ZoneDirty = true;
                m_ShownTextKey = -1;
            }

            if (shown.Version != m_ShownVersion)
            {
                m_ShownVersion = shown.Version;
                if (shown.Available) m_Rate.Add(shown.Stamp);
                plot.SetScan(shown.Angles, shown.Ranges, shown.Count, m_LidarOffset);
                // Obstacles are given in map, so they move in the plot whenever the robot moves: refresh with the scan.
                m_ObstaclesDirty = true;
            }

            if (m_ObstaclesDirty) UpdateObstacles();
            if (m_ZoneDirty) UpdateZone();
            UpdateTexts();
        }

        // ---------------------------------------------------------------- sources

        ILaserScanSource CurrentSource()
        {
            switch (source)
            {
                case Source.Filtered: return m_Filtered;
                case Source.Mock: return m_Mock;
                default:
                    if (m_Unity != null) return m_Unity;
                    // Without a simulated lidar in the scene, show /scan from ROS (e.g. a replayed bag).
                    return m_RawRos ??= new RosLaserScanSource(k_ScanTopic);
            }
        }

        void FindSceneObjects()
        {
            if (m_Unity is UnityLidarSource unity && !unity.Alive)
            {
                unity.Dispose();
                m_Unity = null;
            }
            if (m_Robot != null && m_Unity != null) return;
            if (Time.unscaledTime < m_NextSearch) return;
            m_NextSearch = Time.unscaledTime + k_SearchInterval;

            if (m_Robot == null) m_Robot = FindFirstObjectByType<DeliveryBotDrive>();
            if (m_Unity == null)
            {
                var lidar = FindFirstObjectByType<LidarScanPublisher>();
                if (lidar != null && lidar.isActiveAndEnabled) m_Unity = new UnityLidarSource(lidar);
            }

            // base_link → lidar mount, measured in the scene (falls back to the URDF value).
            var lidarSource = m_Unity as UnityLidarSource;
            if (m_Robot != null && m_Robot.Root != null && lidarSource != null)
            {
                Vector3 local = m_Robot.Root.transform.InverseTransformPoint(lidarSource.Lidar.transform.position);
                var flu = local.To<FLU>();
                m_LidarOffset = new Vector2(flu.x, flu.y);
            }
        }

        // ---------------------------------------------------------------- overlays

        void UpdateObstacles()
        {
            m_ObstaclesDirty = false;
            m_ObstacleCount = 0;
            if (showObstacles)
            {
                if (source == Source.Mock)
                    AddMockObstacle();
                else
                    AddTrackedObstacles();
            }
            plot.SetObstacles(m_Obstacles, m_ObstacleCount);
        }

        void AddMockObstacle()
        {
            Vector2 position = m_LidarOffset + m_Mock.DiscPosition;
            m_Obstacles[m_ObstacleCount++] = new LidarPlotGraphic.Obstacle
            {
                position = position,
                predicted = position + m_Mock.DiscVelocity * k_MockPredictionHorizon,
                radius = MockLaserScanSource.DiscRadius,
            };
        }

        void AddTrackedObstacles()
        {
            var msg = m_ObstacleMsg;
            if (msg == null || m_Robot == null || m_Robot.Root == null) return;
            if (!CheckFrame(msg.header.frame_id, k_MapFrame, k_ObstaclesTopic)) return;

            // map = Unity world origin with ROS axes (#15), so map → base_link is the robot's world pose.
            Transform baseLink = m_Robot.Root.transform;
            foreach (var o in msg.obstacles)
            {
                if (m_ObstacleCount >= k_MaxObstacles) break;
                m_Obstacles[m_ObstacleCount++] = new LidarPlotGraphic.Obstacle
                {
                    position = MapToBaseLink(baseLink, o.position),
                    predicted = MapToBaseLink(baseLink, o.predicted_position),
                    radius = o.radius,
                };
            }
        }

        /// <summary>A point in map (ROS FLU, origin = Unity world origin) as plot coordinates in base_link.</summary>
        static Vector2 MapToBaseLink(Transform baseLink, PointMsg point)
        {
            Vector3 world = new Vector3<FLU>((float)point.x, (float)point.y, (float)point.z).toUnity;
            var p = baseLink.InverseTransformPoint(world).To<FLU>();
            return new Vector2(p.x, p.y);
        }

        void OnStopZone(PolygonStampedMsg msg)
        {
            if (!CheckFrame(msg.header.frame_id, k_BaseFrame, k_StopZoneTopic)) return;
            var points = msg.polygon.points;
            m_ZoneCount = Mathf.Min(points.Length, k_MaxZonePoints);
            for (int i = 0; i < m_ZoneCount; i++)
                m_Zone[i] = new Vector2(points[i].x, points[i].y); // base_link is the plot frame (x forward, y left)
            m_ZoneDirty = true;
        }

        void UpdateZone()
        {
            m_ZoneDirty = false;
            plot.SetStopZone(m_Zone, showStopZone ? m_ZoneCount : 0, SafetyColor());
        }

        Color SafetyColor()
        {
            if (string.IsNullOrEmpty(m_SafetyState)) return zoneUnknown;
            if (m_SafetyState.IndexOf("STOP", StringComparison.OrdinalIgnoreCase) >= 0) return zoneStop;
            if (m_SafetyState.IndexOf("SLOW", StringComparison.OrdinalIgnoreCase) >= 0) return zoneSlow;
            return zoneOk;
        }

        bool CheckFrame(string frame, string expected, string topic)
        {
            if (string.IsNullOrEmpty(frame) || frame == expected) return true;
            if (!m_WarnedFrame)
                Debug.LogWarning($"[LidarPanel] Ignoring {topic}: expected frame '{expected}', got '{frame}'.");
            m_WarnedFrame = true;
            return false;
        }

        // ---------------------------------------------------------------- texts

        void UpdateTexts()
        {
            int key = (int)source * 4 + (showObstacles ? 2 : 0) + (showStopZone ? 1 : 0);
            if (m_ShownTextKey == key && Time.frameCount % 30 != 0) return;
            m_ShownTextKey = key;

            var shown = m_Shown;
            string name = source == Source.Raw ? "Raw" : source == Source.Filtered ? "Filtered" : "Mock";
            HudPanelUtil.SetText(title, $"LiDAR · {name} ({shown.Name})");
            if (shown.Available)
            {
                string rate = m_Rate.Hz > 0 ? $" · {m_Rate.Hz:F1} Hz" : "";
                HudPanelUtil.SetText(caption, $"{shown.Count} pts{rate} · t = {shown.Stamp:F1} s · rings {plot.ringStep:F0} m");
            }
            else
            {
                string hint = source == Source.Filtered ? " (delivery_bot_core#9)" : "";
                HudPanelUtil.SetText(caption, $"waiting for {shown.Name}{hint}");
            }

            HudPanelUtil.SetText(statusText, $"{ObstacleStatus()} · {SafetyStatus()}");

            HudPanelUtil.HighlightButton(rawButton, source == Source.Raw);
            HudPanelUtil.HighlightButton(filteredButton, source == Source.Filtered);
            HudPanelUtil.HighlightButton(mockButton, source == Source.Mock);
            HudPanelUtil.HighlightButton(obstaclesButton, showObstacles);
            HudPanelUtil.HighlightButton(stopZoneButton, showStopZone);
        }

        string ObstacleStatus()
        {
            if (!showObstacles) return "obstacles off";
            if (source == Source.Mock) return "1 mock obstacle";
            if (m_ObstacleMsg == null) return $"no {k_ObstaclesTopic}";
            if (m_Robot == null) return "no robot for obstacles";
            return m_ObstacleCount == 1 ? "1 obstacle" : $"{m_ObstacleCount} obstacles";
        }

        string SafetyStatus()
        {
            string state = string.IsNullOrEmpty(m_SafetyState) ? $"no {k_SafetyStateTopic}" : $"safety: {m_SafetyState}";
            if (!showStopZone) return state;
            return m_ZoneCount > 0 ? state : $"{state} · no stop zone";
        }
    }
}

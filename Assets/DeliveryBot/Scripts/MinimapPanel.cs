using System;
using System.Globalization;
using DeliveryBot.Minimap;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.DeliveryBotInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.Rendering.Universal;
using UnityEngine.UI;

namespace DeliveryBot
{
    /// <summary>
    /// HUD minimap (MinimapSlot, issue #18): a top-down view of the whole city with the robot's true pose, the stack's
    /// estimated pose (/pose), the goal, the cyclists and the plan (/plan), plus the global costmap (/costmap/global)
    /// as a toggleable overlay. Clicking the map publishes /goal_pose (drag to set the heading); without ROS the goal
    /// is only logged. Display and input only: no localization, mapping or planning happens here.
    ///
    /// The city is rendered by an orthographic camera into a RenderTexture only on demand (at start, when a scenario
    /// is loaded and with the Refresh button), so the per-frame GPU cost is one textured quad plus the marker mesh.
    /// The costmap is converted to a texture only when a new grid arrives. Built by Tools > Delivery Bot > Build
    /// Minimap Panel.
    /// </summary>
    public class MinimapPanel : MonoBehaviour, IPointerDownHandler, IDragHandler, IPointerUpHandler
    {
        const string k_MapFrame = "map";
        const float k_SearchInterval = 1f; // s between searches for the robot, the spawner and SimControl
        const int k_MaxCyclists = 64;
        const int k_FirstRenderDelayFrames = 5;

        [Header("Topics")]
        public string poseTopic = "/pose";
        public string planTopic = "/plan";
        public string costmapTopic = "/costmap/global";
        [Tooltip("Shown in the overlay while no costmap has arrived yet (latched static map from the map server).")]
        public string staticMapTopic = "/map";
        public string goalTopic = "/goal_pose";
        public string gridStepTopic = "/grid_step/state";

        [Header("View")]
        [Tooltip("Grow the view to the static colliders (everything outside the Robot/Dynamic layers that reaches "
                 + "into the obstacle band), in addition to the map bounds.")]
        public bool autoFitBounds = true;
        [Tooltip("Always shown: the extent of the exported city map (map frame, m), so the global costmap fits.")]
        public Vector2 mapBoundsMin = new Vector2(-50f, -50f);
        public Vector2 mapBoundsMax = new Vector2(50f, 50f);
        [Tooltip("Auto-fit uses only colliders that reach into this band above Unity Y = 0 (the map export's band).")]
        public float minObstacleHeight = 0.08f;
        public float maxObstacleHeight = 0.6f;
        public float padding = 2f;
        [Tooltip("Longer side of the city RenderTexture in pixels.")]
        public int textureSize = 1024;
        public Color backgroundColor = new Color(0.05f, 0.06f, 0.08f, 1f);

        [Header("Height colours (city image)")]
        public Color groundColor = new Color(0.16f, 0.18f, 0.21f, 1f);
        public Color curbColor = new Color(0.26f, 0.28f, 0.31f, 1f);
        public Color obstacleColor = new Color(0.85f, 0.62f, 0.3f, 1f);
        public Color buildingLowColor = new Color(0.45f, 0.48f, 0.55f, 1f);
        public Color buildingHighColor = new Color(0.78f, 0.81f, 0.88f, 1f);
        public Color edgeColor = new Color(0.02f, 0.02f, 0.03f, 0.9f);
        [Tooltip("Height (m) that gets the brightest building colour.")]
        public float tallestBuilding = 30f;
        public LayerMask excludedLayers;

        [Header("Overlays")]
        public bool showCostmap;
        public bool showPlan = true;
        public Key costmapKey = Key.M;
        [Range(0f, 1f)] public float costmapOpacity = 0.8f;

        [Header("Goal input")]
        [Tooltip("A drag shorter than this (canvas units) keeps the default heading: from the robot towards the goal.")]
        public float minDragForHeading = 8f;

        [Header("References (set by the prefab builder)")]
        public RectTransform mapArea;
        public RawImage cityImage;
        public RawImage costmapImage;
        public MinimapMarkerGraphic markers;
        public AspectRatioFitter aspectFitter;
        public Button costmapButton, planButton, refreshButton;
        public Text title, caption, statusText;

        public MinimapView View => m_View;

        MinimapView m_View;
        Camera m_Camera;
        RenderTexture m_CityTexture;
        RenderTexture m_DepthBuffer;
        Material m_HeightMaterial;
        bool m_RenderRequested = true;
        int m_FirstRenderFrame = -1;
        int m_ViewLayers;

        ROSConnection m_Ros;
        DeliveryBotDrive m_Robot;
        DynamicObjectSpawner m_Spawner;
        SimControl m_SimControl;
        SimScenario m_LastScenario;
        float m_NextSearch;

        // Costmap overlay
        Color32[] m_CostLut;
        Texture2D m_CostTexture;
        OccupancyGridMsg m_PendingGrid;
        bool m_PendingIsCostmap;
        bool m_HasCostmap, m_HasStaticMap;
        string m_OverlayName = "";
        Vector2 m_GridOrigin;
        Vector2 m_GridSize;
        float m_GridYaw;
        Vector2 m_LastAreaSize;
        bool m_OverlayLayoutDirty;

        // Markers
        Vector2[] m_Plan = new Vector2[256];
        int m_PlanCount;
        readonly MinimapMarkerGraphic.Cyclist[] m_Cyclists = new MinimapMarkerGraphic.Cyclist[k_MaxCyclists];
        int m_CyclistCount;
        bool m_HasPose;
        MinimapMarkerGraphic.Pose2D m_Pose;
        float m_PoseSigma;
        bool m_HasGoal;
        MinimapMarkerGraphic.Pose2D m_Goal;
        string m_GoalSource = "";
        bool m_MarkersDirty = true;
        MinimapMarkerGraphic.Pose2D m_LastTruePose;
        bool m_LastHasTruePose;

        // Goal input
        bool m_Placing;
        Vector2 m_PlaceStart;  // map
        Vector2 m_PlaceScreenStart;
        Vector2? m_PlaceDrag;  // map
        int m_GoalCount;

        int m_TextKey = -1;

        // ---------------------------------------------------------------- lifecycle

        void Awake()
        {
            HudPanelUtil.HideSlotPlaceholder(transform);
            HudPanelUtil.EnsureEventSystem();
            m_CostLut = CostmapPalette.BuildLut();
            if (costmapButton != null) costmapButton.onClick.AddListener(ToggleCostmap);
            if (planButton != null) planButton.onClick.AddListener(() =>
            {
                showPlan = !showPlan;
                m_MarkersDirty = true;
            });
            if (refreshButton != null) refreshButton.onClick.AddListener(RequestCityRender);
            if (costmapImage != null) costmapImage.enabled = false;
        }

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.Subscribe<PoseWithCovarianceStampedMsg>(poseTopic, OnPose);
            m_Ros.Subscribe<PathMsg>(planTopic, OnPlan);
            m_Ros.Subscribe<OccupancyGridMsg>(costmapTopic, msg => OnGrid(msg, true));
            if (!string.IsNullOrEmpty(staticMapTopic))
                m_Ros.Subscribe<OccupancyGridMsg>(staticMapTopic, msg => OnGrid(msg, false));
            m_Ros.Subscribe<GridStepStateMsg>(gridStepTopic, OnGridStepState);
            m_Ros.RegisterPublisher<PoseStampedMsg>(goalTopic);
        }

        void OnDestroy()
        {
            if (m_Camera != null) Destroy(m_Camera.gameObject);
            if (m_CityTexture != null)
            {
                m_CityTexture.Release();
                Destroy(m_CityTexture);
            }
            if (m_DepthBuffer != null)
            {
                m_DepthBuffer.Release();
                Destroy(m_DepthBuffer);
            }
            if (m_HeightMaterial != null) Destroy(m_HeightMaterial);
            if (m_CostTexture != null) Destroy(m_CostTexture);
        }

        void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard[costmapKey].wasPressedThisFrame) ToggleCostmap();

            FindSceneObjects();
            // A render request in the very first frames of Play mode comes back empty (URP isn't set up yet), so the
            // first render waits a few frames.
            if (m_FirstRenderFrame < 0) m_FirstRenderFrame = Time.frameCount + k_FirstRenderDelayFrames;
            if (m_RenderRequested && Time.frameCount >= m_FirstRenderFrame) RenderCity();
            if (m_PendingGrid != null) ApplyGrid();
            UpdateOverlayLayout();
            UpdateMarkers();
            UpdateTexts();
        }

        // ---------------------------------------------------------------- scene

        void FindSceneObjects()
        {
            // A newly loaded scenario adds or removes static boxes: render the city again.
            if (m_SimControl != null && m_SimControl.CurrentScenario != m_LastScenario)
            {
                m_LastScenario = m_SimControl.CurrentScenario;
                RequestCityRender();
            }

            if (m_Robot != null && m_Robot.Root != null && m_Spawner != null && m_SimControl != null) return;
            if (Time.unscaledTime < m_NextSearch) return;
            m_NextSearch = Time.unscaledTime + k_SearchInterval;
            if (m_Robot == null || m_Robot.Root == null) m_Robot = FindFirstObjectByType<DeliveryBotDrive>();
            if (m_Spawner == null) m_Spawner = FindFirstObjectByType<DynamicObjectSpawner>();
            if (m_SimControl == null) m_SimControl = FindFirstObjectByType<SimControl>();
        }

        public void RequestCityRender() => m_RenderRequested = true;

        void RenderCity()
        {
            m_RenderRequested = false;
            m_ViewLayers = ~(LayerMaskOf("Robot") | LayerMaskOf("Dynamic") | LayerMaskOf("UI") | excludedLayers.value);
            ComputeView(out float topY, out float bottomY);

            int size = Mathf.Clamp(textureSize, 64, 4096);
            int width = m_View.Aspect >= 1f ? size : Mathf.Max(16, Mathf.RoundToInt(size * m_View.Aspect));
            int height = m_View.Aspect >= 1f ? Mathf.Max(16, Mathf.RoundToInt(size / m_View.Aspect)) : size;
            if (m_CityTexture == null || m_CityTexture.width != width || m_CityTexture.height != height)
            {
                if (m_CityTexture != null)
                {
                    m_CityTexture.Release();
                    Destroy(m_CityTexture);
                }
                m_CityTexture = new RenderTexture(width, height, 0, RenderTextureFormat.ARGB32)
                {
                    name = "MinimapCity",
                    filterMode = FilterMode.Bilinear,
                    wrapMode = TextureWrapMode.Clamp,
                };
                m_CityTexture.Create();
                if (m_DepthBuffer != null)
                {
                    m_DepthBuffer.Release();
                    Destroy(m_DepthBuffer);
                }
                m_DepthBuffer = new RenderTexture(width, height, 32, RenderTextureFormat.Depth) { name = "MinimapDepth" };
                m_DepthBuffer.Create();
            }
            // Widen the view to the texture's exact (whole-pixel) aspect, so the camera's image, the markers and the
            // costmap overlay all use the same view transform.
            m_View = MinimapView.Fit(m_View.min, m_View.Max, 0f, (float)width / height);
            if (aspectFitter != null) aspectFitter.aspectRatio = m_View.Aspect;
            if (markers != null)
            {
                markers.View = m_View;
                m_MarkersDirty = true;
            }
            m_OverlayLayoutDirty = true;

            var cam = EnsureCamera();
            MinimapGeometry.PlaceTopDownCamera(cam, m_View, topY, bottomY);
            cam.cullingMask = m_ViewLayers;
            cam.backgroundColor = backgroundColor;
            RenderHeightMap(cam);

            if (cityImage != null)
            {
                cityImage.texture = m_CityTexture;
                cityImage.color = Color.white;
            }
            m_TextKey = -1;
        }

        /// <summary>
        /// Renders only depth from above and colours it by height: from straight above, roofs and streets have the
        /// same lit colour, so the plain colour image would be one flat grey. Falls back to the colour image if the
        /// shader is missing.
        /// </summary>
        void RenderHeightMap(Camera cam)
        {
            if (m_HeightMaterial == null)
            {
                var shader = Resources.Load<Shader>("DeliveryBot/MinimapHeight");
                if (shader != null) m_HeightMaterial = new Material(shader);
            }
            if (m_HeightMaterial == null)
            {
                Debug.LogWarning("[Minimap] Resources/DeliveryBot/MinimapHeight.shader missing; showing the colour image.");
                var colour = RenderTexture.GetTemporary(m_CityTexture.width, m_CityTexture.height, 24);
                SimCameraChannel.Render(cam, colour);
                Graphics.Blit(colour, m_CityTexture);
                RenderTexture.ReleaseTemporary(colour);
                return;
            }

            SimCameraChannel.Render(cam, m_DepthBuffer);
            var m = m_HeightMaterial;
            m.SetTexture("_DepthTex", m_DepthBuffer, UnityEngine.Rendering.RenderTextureSubElement.Depth);
            m.SetVector("_DepthTex_TexelSize", new Vector4(1f / m_DepthBuffer.width, 1f / m_DepthBuffer.height,
                m_DepthBuffer.width, m_DepthBuffer.height));
            m.SetVector("_ClipPlanes", new Vector4(cam.nearClipPlane, cam.farClipPlane, cam.transform.position.y, 0f));
            m.SetVector("_Bands", new Vector4(minObstacleHeight, maxObstacleHeight, tallestBuilding, 0f));
            m.SetColor("_Background", backgroundColor);
            m.SetColor("_Ground", groundColor);
            m.SetColor("_Curb", curbColor);
            m.SetColor("_Obstacle", obstacleColor);
            m.SetColor("_BuildingLow", buildingLowColor);
            m.SetColor("_BuildingHigh", buildingHighColor);
            m.SetColor("_Edge", edgeColor);
            Graphics.Blit(null, m_CityTexture, m);
        }

        Camera EnsureCamera()
        {
            if (m_Camera != null) return m_Camera;
            var go = new GameObject("MinimapCamera") { hideFlags = HideFlags.DontSave };
            m_Camera = go.AddComponent<Camera>();
            m_Camera.enabled = false; // rendered on demand only
            m_Camera.orthographic = true;
            m_Camera.clearFlags = CameraClearFlags.SolidColor;
            m_Camera.allowHDR = false;
            m_Camera.allowMSAA = false;
            m_Camera.useOcclusionCulling = false;
            var data = m_Camera.GetUniversalAdditionalCameraData();
            data.renderShadows = false;
            data.renderPostProcessing = false;
            data.requiresDepthTexture = false;
            data.requiresColorTexture = false;
            return m_Camera;
        }

        /// <summary>The map-frame view: the map bounds, grown to the static collider bounds (like the map export).</summary>
        void ComputeView(out float topY, out float bottomY)
        {
            bool found = false;
            var bounds = new Bounds();
            if (autoFitBounds)
            {
                foreach (var c in FindObjectsByType<Collider>(FindObjectsSortMode.None))
                {
                    if (!c.enabled || c.isTrigger || !c.gameObject.activeInHierarchy) continue;
                    if ((m_ViewLayers & (1 << c.gameObject.layer)) == 0) continue;
                    // Only what reaches into the map export's obstacle band, so that a large ground plane (top at
                    // Y ≈ 0) doesn't zoom the view out beyond the city.
                    Bounds b = c.bounds;
                    if (b.max.y < minObstacleHeight || b.min.y > maxObstacleHeight) continue;
                    if (found) bounds.Encapsulate(b);
                    else
                    {
                        bounds = b;
                        found = true;
                    }
                }
            }

            Vector2 min = Vector2.Min(mapBoundsMin, mapBoundsMax);
            Vector2 max = Vector2.Max(mapBoundsMin, mapBoundsMax);
            topY = 50f;
            bottomY = -10f;
            if (found)
            {
                // The Unity bounds box is axis-aligned; its corners map onto an axis-aligned box in the map frame.
                Vector2 a = MinimapGeometry.UnityToMap(bounds.min);
                Vector2 b = MinimapGeometry.UnityToMap(bounds.max);
                min = Vector2.Min(min, Vector2.Min(a, b));
                max = Vector2.Max(max, Vector2.Max(a, b));
                topY = bounds.max.y;
                bottomY = Mathf.Min(bounds.min.y, -1f);
            }
            m_View = MinimapView.Fit(min, max, padding);
        }

        static int LayerMaskOf(string layer)
        {
            int index = LayerMask.NameToLayer(layer);
            return index >= 0 ? 1 << index : 0;
        }

        // ---------------------------------------------------------------- ROS input

        void OnPose(PoseWithCovarianceStampedMsg msg)
        {
            if (!IsMapFrame(msg.header.frame_id, poseTopic)) return;
            var p = msg.pose.pose;
            m_Pose = new MinimapMarkerGraphic.Pose2D
            {
                position = new Vector2((float)p.position.x, (float)p.position.y),
                yaw = MinimapGeometry.YawFromQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            };
            m_PoseSigma = MinimapGeometry.PositionSigma(msg.pose.covariance);
            m_HasPose = true;
            m_MarkersDirty = true;
        }

        void OnPlan(PathMsg msg)
        {
            if (!IsMapFrame(msg.header.frame_id, planTopic)) return;
            int count = msg.poses?.Length ?? 0;
            if (m_Plan.Length < count) m_Plan = new Vector2[Mathf.NextPowerOfTwo(count)];
            for (int i = 0; i < count; i++)
            {
                var position = msg.poses[i].pose.position;
                m_Plan[i] = new Vector2((float)position.x, (float)position.y);
            }
            m_PlanCount = count;
            m_MarkersDirty = true;
        }

        void OnGridStepState(GridStepStateMsg msg)
        {
            if (msg.state == GridStepStateMsg.DONE || msg.state == GridStepStateMsg.FAILED)
            {
                // Like the 3D path overlay: the run is over, so its plan and goal go away.
                m_PlanCount = 0;
                m_HasGoal = false;
                m_MarkersDirty = true;
                return;
            }

            // The executive's goal also covers goals sent from the ROS CLI or the scenario runner.
            if (msg.state == GridStepStateMsg.IDLE) return;
            var goal = msg.goal;
            if (goal == null || goal.header == null || goal.header.frame_id != k_MapFrame) return;
            var p = goal.pose;
            m_Goal = new MinimapMarkerGraphic.Pose2D
            {
                position = new Vector2((float)p.position.x, (float)p.position.y),
                yaw = MinimapGeometry.YawFromQuaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
            };
            m_HasGoal = true;
            m_GoalSource = "executive";
            m_MarkersDirty = true;
        }

        void OnGrid(OccupancyGridMsg msg, bool isCostmap)
        {
            string topic = isCostmap ? costmapTopic : staticMapTopic;
            if (!IsMapFrame(msg.header.frame_id, topic)) return;
            if (isCostmap) m_HasCostmap = true;
            else
            {
                m_HasStaticMap = true;
                if (m_HasCostmap) return; // the costmap wins once it exists
            }
            // Converted in Update, at most once per frame, however many grids arrive.
            m_PendingGrid = msg;
            m_PendingIsCostmap = isCostmap;
        }

        bool m_WarnedFrame;

        bool IsMapFrame(string frame, string topic)
        {
            if (string.IsNullOrEmpty(frame) || frame == k_MapFrame) return true;
            if (!m_WarnedFrame)
                Debug.LogWarning($"[Minimap] Ignoring {topic}: expected frame '{k_MapFrame}', got '{frame}'.");
            m_WarnedFrame = true;
            return false;
        }

        // ---------------------------------------------------------------- costmap overlay

        void ToggleCostmap()
        {
            showCostmap = !showCostmap;
            m_OverlayLayoutDirty = true;
            m_TextKey = -1;
        }

        void ApplyGrid()
        {
            var msg = m_PendingGrid;
            m_PendingGrid = null;
            int width = (int)msg.info.width, height = (int)msg.info.height;
            if (width <= 0 || height <= 0 || msg.info.resolution <= 0f || msg.data == null) return;
            if (width > SystemInfo.maxTextureSize || height > SystemInfo.maxTextureSize)
            {
                Debug.LogWarning($"[Minimap] {width}×{height} grid exceeds the maximum texture size; not shown.");
                return;
            }

            if (m_CostTexture == null || m_CostTexture.width != width || m_CostTexture.height != height)
            {
                if (m_CostTexture != null) Destroy(m_CostTexture);
                m_CostTexture = new Texture2D(width, height, TextureFormat.RGBA32, false, true)
                {
                    name = "MinimapCostmap",
                    filterMode = FilterMode.Point,
                    wrapMode = TextureWrapMode.Clamp,
                };
            }
            CostmapPalette.Fill(msg.data, m_CostLut, m_CostTexture.GetPixelData<Color32>(0));
            m_CostTexture.Apply(false, false);

            var origin = msg.info.origin;
            m_GridOrigin = new Vector2((float)origin.position.x, (float)origin.position.y);
            m_GridYaw = MinimapGeometry.YawFromQuaternion(origin.orientation.x, origin.orientation.y,
                origin.orientation.z, origin.orientation.w);
            m_GridSize = new Vector2(width * msg.info.resolution, height * msg.info.resolution);
            m_OverlayName = (m_PendingIsCostmap ? costmapTopic : staticMapTopic)
                            + $" {width}×{height} @ {msg.info.resolution.ToString("0.###", CultureInfo.InvariantCulture)} m";
            if (costmapImage != null) costmapImage.texture = m_CostTexture;
            m_OverlayLayoutDirty = true;
            m_TextKey = -1;
        }

        /// <summary>
        /// Places the grid image over the city: its pivot is the grid origin (cell (0, 0)'s outer corner) and it is
        /// rotated by the origin's yaw, in the same view transform as the city camera, so both line up exactly.
        /// </summary>
        void UpdateOverlayLayout()
        {
            if (costmapImage == null || mapArea == null) return;
            Vector2 areaSize = mapArea.rect.size;
            if (!m_OverlayLayoutDirty && areaSize == m_LastAreaSize) return;
            m_OverlayLayoutDirty = false;
            m_LastAreaSize = areaSize;

            bool visible = showCostmap && m_CostTexture != null && costmapImage.texture != null;
            costmapImage.enabled = visible;
            if (!visible) return;

            var rect = costmapImage.rectTransform;
            rect.anchorMin = rect.anchorMax = Vector2.zero;
            rect.pivot = Vector2.zero;
            Vector2 scale = new Vector2(areaSize.x / m_View.size.x, areaSize.y / m_View.size.y);
            rect.anchoredPosition = Vector2.Scale(m_GridOrigin - m_View.min, scale);
            rect.sizeDelta = Vector2.Scale(m_GridSize, scale);
            rect.localRotation = Quaternion.Euler(0f, 0f, m_GridYaw * Mathf.Rad2Deg);
            costmapImage.color = new Color(1f, 1f, 1f, costmapOpacity);
        }

        // ---------------------------------------------------------------- markers

        void UpdateMarkers()
        {
            if (markers == null) return;

            bool hasTrue = m_Robot != null && m_Robot.Root != null;
            var truePose = new MinimapMarkerGraphic.Pose2D();
            if (hasTrue)
            {
                Transform t = m_Robot.Root.transform;
                truePose.position = MinimapGeometry.UnityToMap(t.position);
                truePose.yaw = MinimapGeometry.UnityDirectionToYaw(t.forward);
            }
            if (hasTrue != m_LastHasTruePose ||
                (truePose.position - m_LastTruePose.position).sqrMagnitude > 1e-4f ||
                Mathf.Abs(Mathf.DeltaAngle(truePose.yaw * Mathf.Rad2Deg, m_LastTruePose.yaw * Mathf.Rad2Deg)) > 0.5f)
            {
                m_LastHasTruePose = hasTrue;
                m_LastTruePose = truePose;
                m_MarkersDirty = true;
            }

            int cyclists = 0;
            if (m_Spawner != null)
            {
                var agents = m_Spawner.Agents;
                for (int i = 0; i < agents.Count && cyclists < k_MaxCyclists; i++)
                {
                    var agent = agents[i];
                    if (agent == null) continue;
                    Vector3 v = agent.Velocity;
                    Vector3 heading = v.sqrMagnitude > 1e-6f ? v : agent.transform.forward;
                    m_Cyclists[cyclists++] = new MinimapMarkerGraphic.Cyclist
                    {
                        position = MinimapGeometry.UnityToMap(agent.Position),
                        yaw = MinimapGeometry.UnityDirectionToYaw(heading),
                        radius = CyclistAgent.Radius,
                    };
                }
            }
            if (cyclists > 0 || cyclists != m_CyclistCount) m_MarkersDirty = true; // cyclists keep moving
            m_CyclistCount = cyclists;

            if (!m_MarkersDirty) return;
            m_MarkersDirty = false;

            markers.HasTruePose = hasTrue;
            markers.TruePose = truePose;
            markers.HasEstimatedPose = m_HasPose;
            markers.EstimatedPose = m_Pose;
            markers.EstimatedSigma = m_PoseSigma;
            markers.HasGoal = m_HasGoal;
            markers.Goal = m_Goal;
            markers.HasPendingGoal = m_Placing;
            if (m_Placing)
                markers.PendingGoal = new MinimapMarkerGraphic.Pose2D
                {
                    position = m_PlaceStart,
                    yaw = PlacingYaw(),
                };
            markers.SetPlan(m_Plan, showPlan ? m_PlanCount : 0);
            markers.SetCyclists(m_Cyclists, m_CyclistCount);
            markers.SetVerticesDirty();
        }

        // ---------------------------------------------------------------- goal input

        bool TryScreenToMap(PointerEventData eventData, out Vector2 map)
        {
            map = default;
            if (mapArea == null) return false;
            if (!RectTransformUtility.ScreenPointToLocalPointInRectangle(mapArea, eventData.position,
                    eventData.pressEventCamera, out Vector2 local))
                return false;
            Rect r = mapArea.rect;
            Vector2 normalized = new Vector2((local.x - r.xMin) / r.width, (local.y - r.yMin) / r.height);
            map = m_View.NormalizedToMap(normalized);
            return normalized.x >= 0f && normalized.y >= 0f && normalized.x <= 1f && normalized.y <= 1f;
        }

        public void OnPointerDown(PointerEventData eventData)
        {
            if (eventData.button != PointerEventData.InputButton.Left) return;
            if (!TryScreenToMap(eventData, out Vector2 map)) return;
            m_Placing = true;
            m_PlaceStart = map;
            m_PlaceScreenStart = eventData.position;
            m_PlaceDrag = null;
            m_MarkersDirty = true;
        }

        public void OnDrag(PointerEventData eventData)
        {
            if (!m_Placing) return;
            TryScreenToMap(eventData, out Vector2 map); // outside the map still gives a usable direction
            m_PlaceDrag = map;
            m_MarkersDirty = true;
        }

        public void OnPointerUp(PointerEventData eventData)
        {
            if (!m_Placing || eventData.button != PointerEventData.InputButton.Left) return;
            m_Placing = false;
            m_Goal = new MinimapMarkerGraphic.Pose2D { position = m_PlaceStart, yaw = PlacingYaw() };
            m_HasGoal = true;
            m_GoalSource = "minimap";
            m_MarkersDirty = true;
            PublishGoal(m_Goal);
        }

        float PlacingYaw()
        {
            // The drag threshold is in canvas units; convert it into metres for the shared heading rule.
            float metresPerUnit = mapArea != null && mapArea.rect.width > 0f ? m_View.size.x / mapArea.rect.width : 0f;
            Vector2? robot = m_LastHasTruePose ? m_LastTruePose.position : m_HasPose ? m_Pose.position : (Vector2?)null;
            return MinimapGeometry.GoalYaw(m_PlaceStart, m_PlaceDrag, robot, minDragForHeading * metresPerUnit);
        }

        void PublishGoal(MinimapMarkerGraphic.Pose2D goal)
        {
            m_GoalCount++;
            double time = Time.fixedTimeAsDouble; // sim time, the same clock as /clock
            int sec = (int)Math.Floor(time);
            uint nanosec = (uint)((time - sec) * 1e9);
            var q = MinimapGeometry.QuaternionFromYaw(goal.yaw);
            var msg = new PoseStampedMsg
            {
                header = new HeaderMsg { stamp = new TimeMsg { sec = sec, nanosec = nanosec }, frame_id = k_MapFrame },
                pose = new PoseMsg
                {
                    position = new PointMsg(goal.position.x, goal.position.y, 0.0),
                    orientation = new QuaternionMsg(q.x, q.y, q.z, q.w),
                },
            };

            string text = string.Format(CultureInfo.InvariantCulture,
                "goal #{0}: map ({1:F2}, {2:F2}) m, yaw {3:F1}°", m_GoalCount, goal.position.x, goal.position.y,
                goal.yaw * Mathf.Rad2Deg);
            // Publishing while disconnected is rejected by the endpoint ("Not registered"), so only log then.
            if (RosBridgeStatus.IsConnected)
            {
                m_Ros.Publish(goalTopic, msg);
                Debug.Log($"[Minimap] Published {goalTopic} {text}");
            }
            else
            {
                Debug.Log($"[Minimap] ROS not connected, not published: {goalTopic} {text}");
            }
            m_TextKey = -1;
        }

        // ---------------------------------------------------------------- texts

        void UpdateTexts()
        {
            int key = (showCostmap ? 1 : 0) + (showPlan ? 2 : 0) + (m_HasGoal ? 4 : 0) + (m_HasPose ? 8 : 0);
            if (m_TextKey == key && Time.frameCount % 30 != 0) return;
            m_TextKey = key;

            HudPanelUtil.SetText(title, "Minimap · click = goal, drag = heading");
            string view = string.Format(CultureInfo.InvariantCulture, "{0:F0}×{1:F0} m", m_View.size.x, m_View.size.y);
            string goal = m_HasGoal
                ? string.Format(CultureInfo.InvariantCulture, "goal ({0:F1}, {1:F1}) from {2}", m_Goal.position.x,
                    m_Goal.position.y, m_GoalSource)
                : "no goal";
            HudPanelUtil.SetText(caption, $"{view} · {goal}{(RosBridgeStatus.IsConnected ? "" : " · ROS offline (log only)")}");

            string pose = m_HasPose
                ? (m_LastHasTruePose
                    ? string.Format(CultureInfo.InvariantCulture, "pose error {0:F2} m",
                        (m_Pose.position - m_LastTruePose.position).magnitude)
                    : "pose ok")
                : $"no {poseTopic}";
            string overlay;
            if (!showCostmap) overlay = "costmap off (M)";
            else if (m_HasCostmap) overlay = m_OverlayName;
            else if (m_HasStaticMap) overlay = $"no {costmapTopic}, showing {m_OverlayName}";
            else overlay = $"waiting for {costmapTopic}";
            string plan = !showPlan ? "plan off" : m_PlanCount > 1 ? $"plan {m_PlanCount} pts" : $"no {planTopic}";
            HudPanelUtil.SetText(statusText, $"{pose} · {plan} · {overlay}");

            HudPanelUtil.HighlightButton(costmapButton, showCostmap);
            HudPanelUtil.HighlightButton(planButton, showPlan);
        }
    }
}

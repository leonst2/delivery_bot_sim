using System;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.UI;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;
using UnityEngine.UI;

namespace DeliveryBot
{
    /// <summary>
    /// HUD camera panel (CameraSlot): shows the robot's stereo feeds, the side camera feeds and, from the own stereo
    /// depth node, a false-colour disparity view. Display only: the images come straight from the cameras'
    /// RenderTextures (no ROS round trip); only the disparity arrives over ROS.
    ///
    /// Without a <see cref="StereoCameraPublisher"/> in the scene, a stand-in scene camera (default Camera.main) is
    /// rendered instead, so the panel can be laid out and tested before the robot exists. Without registered
    /// <see cref="SideCameraFeeds"/>, stand-in cameras are rendered on the robot's mono camera mounts (or, without a
    /// robot, yawed outwards from the stand-in scene camera).
    /// Built by Tools > Delivery Bot > Build Camera Panel.
    /// </summary>
    public class CameraPanel : MonoBehaviour
    {
        public enum View { Stereo, Left, Right, Disparity, Side }

        /// <summary>One image in the panel: a letterboxed RawImage with a title and a caption.</summary>
        [Serializable]
        public class FeedView
        {
            public GameObject root;
            public RawImage image;
            public AspectRatioFitter fitter;
            public Text title;
            public Text caption;

            public bool Visible => root != null && root.activeSelf;

            public void Show(bool visible)
            {
                if (root != null && root.activeSelf != visible) root.SetActive(visible);
            }

            public void SetTexture(Texture texture, Rect uvRect)
            {
                if (image.texture != texture) image.texture = texture;
                image.uvRect = uvRect;
                image.enabled = texture != null;
                if (texture != null && texture.height > 0)
                    fitter.aspectRatio = (float)texture.width / texture.height;
            }
        }

        /// <summary>Capture rate from consecutive sim-time stamps, smoothed.</summary>
        struct RateEstimator
        {
            double m_Last;
            bool m_HasLast;
            public double Hz;

            public void Reset()
            {
                m_HasLast = false;
                Hz = 0;
            }

            public void Add(double stamp)
            {
                if (m_HasLast && stamp > m_Last)
                {
                    double rate = 1.0 / (stamp - m_Last);
                    Hz = Hz <= 0 ? rate : 0.8 * Hz + 0.2 * rate;
                }
                m_Last = stamp;
                m_HasLast = true;
            }
        }

        /// <summary>Where one side feed comes from: a registered side camera, or a stand-in rendered by the panel.</summary>
        class SideSource
        {
            public readonly CameraSide side;
            public ISideCameraFeed feed;
            public RateEstimator rate;
            public Camera standIn;
            public Transform standInParent;
            public bool standInOnRobot;
            public RenderTexture standInTexture;
            readonly Action<double> m_OnCaptured;

            public SideSource(CameraSide side)
            {
                this.side = side;
                m_OnCaptured = stamp => rate.Add(stamp);
            }

            public bool HasFeed => feed != null && feed.Texture != null;
            public Texture Texture => HasFeed ? feed.Texture : standInTexture;
            public Camera Camera => HasFeed ? feed.Camera : standIn;

            public void Track(ISideCameraFeed current)
            {
                if (current == feed) return;
                if (feed != null) feed.Captured -= m_OnCaptured;
                feed = current;
                rate.Reset();
                if (feed != null) feed.Captured += m_OnCaptured;
            }
        }

        const string k_DisparityTopic = "/stereo/disparity";
        const string k_SideMount = "CameraSensor";
        const float k_SearchInterval = 1f; // s between searches for the stereo cameras and the robot
        static readonly Rect k_Upright = new Rect(0f, 0f, 1f, 1f);
        static readonly Rect k_FlipY = new Rect(0f, 1f, 1f, -1f); // ROS images are top row first
        static readonly string[] k_SideLinks = { "mono_left_camera", "mono_right_camera" };

        [Header("Views")]
        public View view = View.Stereo;
        public Key cycleKey = Key.V;
        public FeedView left = new FeedView();
        public FeedView right = new FeedView();
        public FeedView disparity = new FeedView();
        public FeedView sideLeft = new FeedView();
        public FeedView sideRight = new FeedView();
        public Button stereoButton, leftButton, rightButton, disparityButton, sideButton;
        public Text statusText;

        [Header("Disparity")]
        [Tooltip("Disparity (px) shown at the top of the colour map. Values <= 0, NaN and Inf are drawn black.")]
        public float maxDisparity = 96f;

        [Header("Stand-in until the stereo and side cameras exist")]
        [Tooltip("Defaults to Camera.main.")]
        public Camera standInCamera;
        public int standInWidth = 640;
        public int standInHeight = 360;
        public float standInRateHz = 10f;
        [Tooltip("Horizontal FOV of the side stand-ins (URDF placeholder: 1.2 rad).")]
        public float standInSideHorizontalFovDeg = 68.75f;
        [Tooltip("Outward yaw of the side stand-ins when there is no robot to mount them on (URDF: 21.7°).")]
        public float standInSideYawDeg = 21.7f;

        StereoCameraPublisher m_Stereo;
        float m_NextSearch;
        RenderTexture m_StandInTexture;
        float m_NextStandInRender;

        readonly SideSource[] m_Sides = { new SideSource(CameraSide.Left), new SideSource(CameraSide.Right) };
        DeliveryBotDrive m_Robot;
        float m_NextRobotSearch;
        float m_NextSideStandInRender;
        int m_NextSideStandIn;

        Texture2D m_DisparityTexture;
        Material m_DisparityMaterial;
        bool m_HasDisparity;
        double m_DisparityStamp;

        RateEstimator m_PairRate; // from PairCaptured (sim time)
        int m_ShownView = -1;

        void Awake()
        {
            HideSlotPlaceholder();
            EnsureEventSystem();
            var shader = Resources.Load<Shader>("DeliveryBot/DisparityColormap");
            if (shader != null)
            {
                m_DisparityMaterial = new Material(shader);
                disparity.image.material = m_DisparityMaterial;
            }
            else
            {
                Debug.LogWarning("[CameraPanel] Resources/DeliveryBot/DisparityColormap.shader missing; disparity shown in grey.");
            }

            if (stereoButton != null) stereoButton.onClick.AddListener(() => view = View.Stereo);
            if (leftButton != null) leftButton.onClick.AddListener(() => view = View.Left);
            if (rightButton != null) rightButton.onClick.AddListener(() => view = View.Right);
            if (disparityButton != null) disparityButton.onClick.AddListener(() => view = View.Disparity);
            if (sideButton != null) sideButton.onClick.AddListener(() => view = View.Side);
        }

        void Start()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(k_DisparityTopic, OnDisparity);
        }

        void OnDisable()
        {
            if (m_Stereo != null) m_Stereo.PairCaptured -= OnPairCaptured;
            m_Stereo = null;
            foreach (var side in m_Sides) side.Track(null);
        }

        void OnDestroy()
        {
            if (m_StandInTexture != null) m_StandInTexture.Release();
            foreach (var side in m_Sides)
            {
                if (side.standInTexture != null) side.standInTexture.Release();
                if (side.standIn != null) Destroy(side.standIn.gameObject);
            }
            if (m_DisparityTexture != null) Destroy(m_DisparityTexture);
            if (m_DisparityMaterial != null) Destroy(m_DisparityMaterial);
        }

        void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard[cycleKey].wasPressedThisFrame)
                view = (View)(((int)view + 1) % 5);

            FindStereo();
            if (m_Stereo == null)
                RenderStandIn();

            foreach (var side in m_Sides) side.Track(SideCameraFeeds.Get(side.side));
            if (view == View.Side)
                RenderSideStandIns();

            UpdateFeeds();
        }

        // ---------------------------------------------------------------- sources

        Camera StandInCamera => standInCamera != null ? standInCamera : Camera.main;

        void FindStereo()
        {
            if (m_Stereo != null && m_Stereo.isActiveAndEnabled && m_Stereo.LeftTexture != null) return;
            if (m_Stereo != null)
            {
                m_Stereo.PairCaptured -= OnPairCaptured;
                m_Stereo = null;
            }
            if (Time.unscaledTime < m_NextSearch) return;
            m_NextSearch = Time.unscaledTime + k_SearchInterval;

            var found = FindFirstObjectByType<StereoCameraPublisher>();
            if (found == null || !found.isActiveAndEnabled || found.LeftTexture == null) return;
            m_Stereo = found;
            m_Stereo.PairCaptured += OnPairCaptured;
            m_PairRate.Reset();
        }

        void OnPairCaptured(double stamp) => m_PairRate.Add(stamp);

        DeliveryBotDrive FindRobot()
        {
            if (m_Robot != null || Time.unscaledTime < m_NextRobotSearch) return m_Robot;
            m_NextRobotSearch = Time.unscaledTime + k_SearchInterval;
            m_Robot = FindFirstObjectByType<DeliveryBotDrive>();
            return m_Robot;
        }

        void RenderStandIn()
        {
            var cam = StandInCamera;
            if (cam == null || Time.unscaledTime < m_NextStandInRender) return;
            m_NextStandInRender = Time.unscaledTime + 1f / Mathf.Max(0.1f, standInRateHz);

            if (m_StandInTexture == null)
                m_StandInTexture = CreateStandInTexture("CameraPanelStandIn");
            Render(cam, m_StandInTexture);
        }

        void RenderSideStandIns()
        {
            // The two stand-ins take turns, so at most one extra camera renders in any frame.
            if (Time.unscaledTime < m_NextSideStandInRender) return;
            m_NextSideStandInRender = Time.unscaledTime + 0.5f / Mathf.Max(0.1f, standInRateHz);
            var side = m_Sides[m_NextSideStandIn];
            m_NextSideStandIn = 1 - m_NextSideStandIn;
            if (side.HasFeed) return;

            if (!EnsureSideStandIn(side)) return;
            if (side.standInTexture == null)
                side.standInTexture = CreateStandInTexture($"CameraPanelSideStandIn{side.side}");
            Render(side.standIn, side.standInTexture);
        }

        bool EnsureSideStandIn(SideSource side)
        {
            // Preferred: the robot's mono camera mount, so the stand-in shows the URDF view. Otherwise the stand-in
            // scene camera, yawed outwards.
            Transform mount = FindSideMount(side.side);
            bool onRobot = mount != null;
            Transform parent = onRobot ? mount : StandInCamera != null ? StandInCamera.transform : null;
            if (parent == null) return false;
            if (side.standIn != null && side.standInParent == parent) return true;

            if (side.standIn != null) Destroy(side.standIn.gameObject);
            var go = new GameObject($"CameraPanelSideStandIn{side.side}") { hideFlags = HideFlags.DontSave };
            var cam = go.AddComponent<Camera>();
            var template = StandInCamera;
            if (template != null) cam.CopyFrom(template); // also copies the template's pose and layer
            go.transform.SetParent(parent, false);
            float yaw = onRobot ? 0f : (side.side == CameraSide.Left ? -standInSideYawDeg : standInSideYawDeg);
            go.transform.SetLocalPositionAndRotation(Vector3.zero, Quaternion.Euler(0f, yaw, 0f));
            cam.enabled = false;
            cam.targetTexture = null;
            cam.usePhysicalProperties = false;
            cam.aspect = (float)standInWidth / standInHeight;
            cam.fieldOfView = Camera.HorizontalToVerticalFieldOfView(standInSideHorizontalFovDeg, cam.aspect);
            if (onRobot) cam.nearClipPlane = 0.05f;

            side.standIn = cam;
            side.standInParent = parent;
            side.standInOnRobot = onRobot;
            return true;
        }

        Transform FindSideMount(CameraSide side)
        {
            var robot = FindRobot();
            if (robot == null) return null;
            var current = m_Sides[(int)side];
            if (current.standInOnRobot && current.standInParent != null) return current.standInParent;
            foreach (var t in robot.GetComponentsInChildren<Transform>(true))
            {
                if (t.name != k_SideLinks[(int)side]) continue;
                var mount = t.Find(k_SideMount);
                return mount != null ? mount : t;
            }
            return null;
        }

        RenderTexture CreateStandInTexture(string textureName)
        {
            var texture = new RenderTexture(standInWidth, standInHeight, 24) { name = textureName };
            texture.Create();
            return texture;
        }

        static void Render(Camera cam, RenderTexture target)
        {
            var request = new UniversalRenderPipeline.SingleCameraRequest { destination = target };
            if (RenderPipeline.SupportsRenderRequest(cam, request))
                RenderPipeline.SubmitRenderRequest(cam, request);
        }

        void OnDisparity(ImageMsg msg)
        {
            if (msg.encoding != "32FC1" || msg.width == 0 || msg.height == 0 || msg.step != msg.width * 4
                || msg.data.Length < msg.step * msg.height)
            {
                Debug.LogWarning($"[CameraPanel] Ignoring {k_DisparityTopic}: expected 32FC1, got {msg.encoding} {msg.width}x{msg.height}.");
                return;
            }
            int width = (int)msg.width, height = (int)msg.height;
            if (m_DisparityTexture == null || m_DisparityTexture.width != width || m_DisparityTexture.height != height)
            {
                if (m_DisparityTexture != null) Destroy(m_DisparityTexture);
                m_DisparityTexture = new Texture2D(width, height, TextureFormat.RFloat, false, true)
                {
                    name = "CameraPanelDisparity",
                    filterMode = FilterMode.Point,
                    wrapMode = TextureWrapMode.Clamp,
                };
            }
            m_DisparityTexture.SetPixelData(msg.data, 0);
            m_DisparityTexture.Apply(false, false);
            m_HasDisparity = true;
            m_DisparityStamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
        }

        // ---------------------------------------------------------------- display

        void UpdateFeeds()
        {
            bool stereo = m_Stereo != null;
            Texture leftTexture = stereo ? m_Stereo.LeftTexture : m_StandInTexture;
            Texture rightTexture = stereo ? m_Stereo.RightTexture : null;

            left.Show(view == View.Stereo || view == View.Left || view == View.Side);
            right.Show(view == View.Stereo || view == View.Right);
            disparity.Show(view == View.Disparity);
            sideLeft.Show(view == View.Side);
            sideRight.Show(view == View.Side);

            if (view == View.Disparity)
            {
                disparity.SetTexture(m_HasDisparity ? m_DisparityTexture : null, k_FlipY);
                if (m_DisparityMaterial != null) m_DisparityMaterial.SetFloat("_MaxDisparity", maxDisparity);
            }
            else
            {
                left.SetTexture(leftTexture, k_Upright);
                right.SetTexture(rightTexture, k_Upright);
                if (view == View.Side)
                {
                    sideLeft.SetTexture(m_Sides[0].Texture, k_Upright);
                    sideRight.SetTexture(m_Sides[1].Texture, k_Upright);
                }
            }

            // Text only changes with the view or about once a second, to keep the per-frame cost near zero.
            if (m_ShownView == (int)view && Time.frameCount % 30 != 0) return;
            m_ShownView = (int)view;
            UpdateTexts(stereo, leftTexture);
        }

        void UpdateTexts(bool stereo, Texture leftTexture)
        {
            string stereoLeftTitle = view == View.Side ? "Stereo" : "Left";
            if (stereo)
            {
                string caption = Caption(leftTexture, m_PairRate.Hz);
                SetText(left.title, stereoLeftTitle);
                SetText(right.title, "Right");
                SetText(left.caption, caption);
                SetText(right.caption, caption);
            }
            else
            {
                var cam = StandInCamera;
                SetText(left.title, cam != null ? $"Stand-in: {cam.name}" : "No camera");
                SetText(left.caption, "stereo cameras not found");
                SetText(right.title, "Right");
                SetText(right.caption, "no stereo camera");
            }

            UpdateSideTexts(sideLeft, m_Sides[0], "Left side");
            UpdateSideTexts(sideRight, m_Sides[1], "Right side");

            bool sidesMissing = !m_Sides[0].HasFeed || !m_Sides[1].HasFeed;
            if (!stereo && view != View.Disparity)
                SetText(statusText, "Stand-in view until the robot's stereo cameras exist");
            else if (sidesMissing && view == View.Side)
                SetText(statusText, "Side stand-ins until the side cameras exist (#40)");
            else
                SetText(statusText, "");

            SetText(disparity.title, "Disparity");
            SetText(disparity.caption, m_HasDisparity
                ? $"{m_DisparityTexture.width}×{m_DisparityTexture.height} · 0–{maxDisparity:F0} px · t = {m_DisparityStamp:F1} s"
                : $"waiting for {k_DisparityTopic} (delivery_bot_core#10)");

            HighlightButton(stereoButton, view == View.Stereo);
            HighlightButton(leftButton, view == View.Left);
            HighlightButton(rightButton, view == View.Right);
            HighlightButton(disparityButton, view == View.Disparity);
            HighlightButton(sideButton, view == View.Side);
        }

        void UpdateSideTexts(FeedView feedView, SideSource source, string title)
        {
            if (source.HasFeed)
            {
                SetText(feedView.title, title);
                SetText(feedView.caption, Caption(source.feed.Texture, source.rate.Hz));
            }
            else
            {
                SetText(feedView.title, $"Stand-in: {title}");
                SetText(feedView.caption, source.standIn == null ? "no camera"
                    : source.standInOnRobot ? "side cameras not found (#40) · on the robot's mount"
                    : "side cameras not found (#40)");
            }
        }

        static string Caption(Texture texture, double rateHz)
        {
            if (texture == null) return "";
            string rate = rateHz > 0 ? $" · {rateHz:F1} Hz" : "";
            return $"{texture.width}×{texture.height}{rate}";
        }

        static void SetText(Text text, string value)
        {
            if (text != null && text.text != value) text.text = value;
        }

        static void HighlightButton(Button button, bool active)
        {
            if (button == null) return;
            var image = button.targetGraphic as Image;
            if (image != null)
                image.color = active ? new Color(0.25f, 0.55f, 0.95f, 0.95f) : new Color(1f, 1f, 1f, 0.15f);
        }

        // ---------------------------------------------------------------- setup helpers

        void HideSlotPlaceholder()
        {
            var placeholder = transform.parent != null ? transform.parent.Find("SlotPlaceholder") : null;
            if (placeholder != null) placeholder.gameObject.SetActive(false);
        }

        static void EnsureEventSystem()
        {
            // The HUD's buttons need an EventSystem; CitySim doesn't have one of its own.
            if (EventSystem.current != null || FindFirstObjectByType<EventSystem>() != null) return;
            var go = new GameObject("EventSystem", typeof(EventSystem), typeof(InputSystemUIInputModule));
            DontDestroyOnLoad(go);
        }
    }
}

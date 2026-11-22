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
    /// HUD camera panel (CameraSlot): shows the robot's stereo feeds and, from the own stereo depth node, a
    /// false-colour disparity view. Display only: the images come straight from the cameras' RenderTextures
    /// (no ROS round trip); only the disparity arrives over ROS.
    ///
    /// Without a <see cref="StereoCameraPublisher"/> in the scene, a stand-in scene camera (default Camera.main) is
    /// rendered instead, so the panel can be laid out and tested before the robot exists.
    /// Built by Tools > Delivery Bot > Build Camera Panel.
    /// </summary>
    public class CameraPanel : MonoBehaviour
    {
        public enum View { Stereo, Left, Right, Disparity }

        /// <summary>One image in the panel: a letterboxed RawImage with a title and a caption.</summary>
        [Serializable]
        public class FeedView
        {
            public GameObject root;
            public RawImage image;
            public AspectRatioFitter fitter;
            public Text title;
            public Text caption;

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

        const string k_DisparityTopic = "/stereo/disparity";
        const float k_SearchInterval = 1f; // s between searches for the stereo cameras
        static readonly Rect k_Upright = new Rect(0f, 0f, 1f, 1f);
        static readonly Rect k_FlipY = new Rect(0f, 1f, 1f, -1f); // ROS images are top row first

        [Header("Views")]
        public View view = View.Stereo;
        public Key cycleKey = Key.V;
        public FeedView left = new FeedView();
        public FeedView right = new FeedView();
        public FeedView disparity = new FeedView();
        public Button stereoButton, leftButton, rightButton, disparityButton;
        public Text statusText;

        [Header("Disparity")]
        [Tooltip("Disparity (px) shown at the top of the colour map. Values <= 0, NaN and Inf are drawn black.")]
        public float maxDisparity = 96f;

        [Header("Stand-in until the stereo cameras exist")]
        [Tooltip("Defaults to Camera.main.")]
        public Camera standInCamera;
        public int standInWidth = 640;
        public int standInHeight = 360;
        public float standInRateHz = 10f;

        StereoCameraPublisher m_Stereo;
        float m_NextSearch;
        RenderTexture m_StandInTexture;
        float m_NextStandInRender;

        Texture2D m_DisparityTexture;
        Material m_DisparityMaterial;
        bool m_HasDisparity;
        double m_DisparityStamp;

        // Rate estimate from PairCaptured (sim time).
        double m_LastPairStamp = double.NaN;
        double m_PairRateHz;
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
        }

        void Start()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(k_DisparityTopic, OnDisparity);
        }

        void OnDisable()
        {
            if (m_Stereo != null) m_Stereo.PairCaptured -= OnPairCaptured;
            m_Stereo = null;
        }

        void OnDestroy()
        {
            if (m_StandInTexture != null) m_StandInTexture.Release();
            if (m_DisparityTexture != null) Destroy(m_DisparityTexture);
            if (m_DisparityMaterial != null) Destroy(m_DisparityMaterial);
        }

        void Update()
        {
            var keyboard = Keyboard.current;
            if (keyboard != null && keyboard[cycleKey].wasPressedThisFrame)
                view = (View)(((int)view + 1) % 4);

            FindStereo();
            if (m_Stereo == null)
                RenderStandIn();

            UpdateFeeds();
        }

        // ---------------------------------------------------------------- sources

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
            m_LastPairStamp = double.NaN;
            m_PairRateHz = 0;
        }

        void OnPairCaptured(double stamp)
        {
            if (!double.IsNaN(m_LastPairStamp) && stamp > m_LastPairStamp)
            {
                double rate = 1.0 / (stamp - m_LastPairStamp);
                m_PairRateHz = m_PairRateHz <= 0 ? rate : 0.8 * m_PairRateHz + 0.2 * rate;
            }
            m_LastPairStamp = stamp;
        }

        void RenderStandIn()
        {
            var cam = standInCamera != null ? standInCamera : Camera.main;
            if (cam == null || Time.unscaledTime < m_NextStandInRender) return;
            m_NextStandInRender = Time.unscaledTime + 1f / Mathf.Max(0.1f, standInRateHz);

            if (m_StandInTexture == null)
            {
                m_StandInTexture = new RenderTexture(standInWidth, standInHeight, 24) { name = "CameraPanelStandIn" };
                m_StandInTexture.Create();
            }
            var request = new UniversalRenderPipeline.SingleCameraRequest { destination = m_StandInTexture };
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

            left.Show(view == View.Stereo || view == View.Left);
            right.Show(view == View.Stereo || view == View.Right);
            disparity.Show(view == View.Disparity);

            if (view != View.Disparity)
            {
                left.SetTexture(leftTexture, k_Upright);
                right.SetTexture(rightTexture, k_Upright);
            }
            else
            {
                disparity.SetTexture(m_HasDisparity ? m_DisparityTexture : null, k_FlipY);
                if (m_DisparityMaterial != null) m_DisparityMaterial.SetFloat("_MaxDisparity", maxDisparity);
            }

            // Text only changes with the view or about once a second, to keep the per-frame cost near zero.
            if (m_ShownView == (int)view && Time.frameCount % 30 != 0) return;
            m_ShownView = (int)view;
            UpdateTexts(stereo, leftTexture);
        }

        void UpdateTexts(bool stereo, Texture leftTexture)
        {
            if (stereo)
            {
                string size = $"{leftTexture.width}×{leftTexture.height}";
                string rate = m_PairRateHz > 0 ? $" · {m_PairRateHz:F1} Hz" : "";
                SetText(left.title, "Left");
                SetText(right.title, "Right");
                SetText(left.caption, size + rate);
                SetText(right.caption, size + rate);
                SetText(statusText, "");
            }
            else
            {
                var cam = standInCamera != null ? standInCamera : Camera.main;
                SetText(left.title, cam != null ? $"Stand-in: {cam.name}" : "No camera");
                SetText(left.caption, "stereo cameras not found");
                SetText(right.title, "Right");
                SetText(right.caption, "no stereo camera");
                SetText(statusText, "Stand-in view until the robot's stereo cameras exist");
            }

            SetText(disparity.title, "Disparity");
            SetText(disparity.caption, m_HasDisparity
                ? $"{m_DisparityTexture.width}×{m_DisparityTexture.height} · 0–{maxDisparity:F0} px · t = {m_DisparityStamp:F1} s"
                : $"waiting for {k_DisparityTopic} (delivery_bot_core#10)");

            HighlightButton(stereoButton, view == View.Stereo);
            HighlightButton(leftButton, view == View.Left);
            HighlightButton(rightButton, view == View.Right);
            HighlightButton(disparityButton, view == View.Disparity);
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

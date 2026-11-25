using System;
using RosMessageTypes.Sensor;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;

namespace DeliveryBot
{
    /// <summary>
    /// Simulated stereo camera pair. Put this on the robot root (the setup window does). It adds a Camera to the
    /// "CameraSensor" mounts under stereo_left_camera and stereo_right_camera, renders both at <see cref="rateHz"/> on
    /// sim time and publishes rgb8 images plus exact camera_info (zero distortion, P[3] = -fx * baseline on the right).
    /// Both images of a pair carry the same stamp. Optionally publishes ground-truth depth of the left camera.
    ///
    /// Other Unity components (UI camera panel) use <see cref="LeftTexture"/> / <see cref="RightTexture"/> directly,
    /// without a ROS round trip, and <see cref="PairCaptured"/> to know when they changed.
    /// </summary>
    public class StereoCameraPublisher : MonoBehaviour
    {
        const string k_LeftLink = "stereo_left_camera";
        const string k_RightLink = "stereo_right_camera";
        const string k_LogTag = "StereoCamera";
        const float k_UrdfBaseline = 0.062f; // m, stereo_left/right_camera joints in the URDF

        [Header("Camera")]
        public int width = 640;
        public int height = 480;
        [Tooltip("Vertical field of view. 50° is a placeholder until the real camera's value is known.")]
        public float verticalFovDeg = 50f;
        public float rateHz = 10f;
        public float nearClip = 0.05f;
        public float farClip = 100f;

        [Header("ROS")]
        public string leftImageTopic = "/stereo/left/image_raw";
        public string rightImageTopic = "/stereo/right/image_raw";
        public string leftInfoTopic = "/stereo/left/camera_info";
        public string rightInfoTopic = "/stereo/right/camera_info";
        public string leftFrameId = "stereo_left_camera_optical_frame";
        public string rightFrameId = "stereo_right_camera_optical_frame";

        [Header("Ground-truth depth (evaluation of the stereo depth node)")]
        public bool publishDepthGroundTruth;
        public string depthTopic = "/stereo/left/depth_gt";

        // ---- For other Unity components ----
        public RenderTexture LeftTexture { get; private set; }
        public RenderTexture RightTexture { get; private set; }
        /// <summary>Linear depth (metres along the optical axis) of the left camera, RFloat. Null unless enabled.</summary>
        public RenderTexture DepthTexture { get; private set; }
        public Camera LeftCamera { get; private set; }
        public Camera RightCamera { get; private set; }
        public StereoIntrinsics LeftIntrinsics { get; private set; }
        public StereoIntrinsics RightIntrinsics { get; private set; }
        /// <summary>Distance between the two camera centres (m), as measured in the scene.</summary>
        public float Baseline { get; private set; }

        /// <summary>Last captured pair as rgb8, top row first (ROS layout). Valid when <see cref="PairCaptured"/> fires.</summary>
        public byte[] LatestLeftRgb { get; private set; }
        public byte[] LatestRightRgb { get; private set; }
        /// <summary>Last ground-truth depth, top row first. +Inf where nothing was hit.</summary>
        public float[] LatestDepth { get; private set; }
        /// <summary>Sim time (s) of the last pair; both images (and depth) share it.</summary>
        public double LatestStamp { get; private set; }
        public event Action<double> PairCaptured;

        ROSConnection m_Ros;
        SimCameraChannel m_Left, m_Right;
        MessagePool<ImageMsg> m_DepthPool;
        Camera m_DepthCamera;
        RenderTexture m_DepthBuffer;
        Material m_LinearDepth;

        double m_NextCaptureTime;
        bool m_CaptureDue;
        double m_DueStamp;

        // One capture in flight at a time: the readbacks of a pair complete within a few frames.
        bool m_InFlight;
        double m_InFlightStamp;
        int m_Pending;
        bool m_InFlightHasDepth;
        bool m_InFlightFailed;
        int m_SkippedCaptures;
        Action<bool> m_OnComplete;
        Action<AsyncGPUReadbackRequest> m_OnDepth;

        void Awake()
        {
            // The right camera's P[3] needs the baseline, measured between the two mounts in the scene.
            var leftMount = SimCameraChannel.FindMount(transform, k_LeftLink);
            var rightMount = SimCameraChannel.FindMount(transform, k_RightLink);
            if (leftMount != null && rightMount != null)
            {
                Baseline = Vector3.Distance(leftMount.position, rightMount.position);
                if (Mathf.Abs(Baseline - k_UrdfBaseline) > 0.001f)
                    Debug.LogWarning($"[{k_LogTag}] Baseline is {Baseline * 1000f:F1} mm, the URDF says {k_UrdfBaseline * 1000f:F0} mm.");
            }

            m_Left = SimCameraChannel.Create(transform, k_LeftLink, new StereoIntrinsics(width, height, verticalFovDeg),
                nearClip, farClip, leftFrameId, "StereoLeft", k_LogTag);
            m_Right = SimCameraChannel.Create(transform, k_RightLink,
                new StereoIntrinsics(width, height, verticalFovDeg, Baseline), nearClip, farClip, rightFrameId,
                "StereoRight", k_LogTag);
            if (m_Left == null || m_Right == null)
            {
                enabled = false;
                return;
            }

            LeftCamera = m_Left.Camera;
            RightCamera = m_Right.Camera;
            LeftIntrinsics = m_Left.Intrinsics;
            RightIntrinsics = m_Right.Intrinsics;
            LeftTexture = m_Left.Texture;
            RightTexture = m_Right.Texture;
            LatestLeftRgb = m_Left.Rgb;
            LatestRightRgb = m_Right.Rgb;

            if (publishDepthGroundTruth)
                SetUpDepth();
        }

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Left.Register(m_Ros, leftImageTopic, leftInfoTopic);
            m_Right.Register(m_Ros, rightImageTopic, rightInfoTopic);
            if (publishDepthGroundTruth)
                m_DepthPool = SimCameraChannel.RegisterImagePublisher(m_Ros, depthTopic);
            m_NextCaptureTime = Time.fixedTimeAsDouble;
        }

        void OnValidate()
        {
            width = Mathf.Clamp(width, 16, 4096);
            height = Mathf.Clamp(height, 16, 4096);
            verticalFovDeg = Mathf.Clamp(verticalFovDeg, 1f, 170f);
            rateHz = Mathf.Max(0.1f, rateHz);
            nearClip = Mathf.Max(0.001f, nearClip);
            farClip = Mathf.Max(nearClip + 0.1f, farClip);
        }

        void OnDestroy()
        {
            m_Left?.Release();
            m_Right?.Release();
            foreach (var texture in new[] { DepthTexture, m_DepthBuffer })
                if (texture != null) texture.Release();
            if (m_LinearDepth != null) Destroy(m_LinearDepth);
        }

        // ---------------------------------------------------------------- setup

        void SetUpDepth()
        {
            var shader = Resources.Load<Shader>("DeliveryBot/LinearDepth");
            if (shader == null)
            {
                Debug.LogError($"[{k_LogTag}] Resources/DeliveryBot/LinearDepth.shader missing; depth_gt disabled.");
                publishDepthGroundTruth = false;
                return;
            }
            m_LinearDepth = new Material(shader);

            // A second camera at the left camera's pose that renders into a depth-only target.
            var go = new GameObject("DepthGroundTruthCamera") { layer = LeftCamera.gameObject.layer };
            go.transform.SetParent(LeftCamera.transform, false);
            m_DepthCamera = go.AddComponent<Camera>();
            SimCameraChannel.Configure(m_DepthCamera, LeftIntrinsics, nearClip, farClip);

            m_DepthBuffer = new RenderTexture(width, height, 32, RenderTextureFormat.Depth) { name = "StereoDepthBuffer" };
            m_DepthBuffer.Create();
            DepthTexture = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat) { name = "StereoDepthGT" };
            DepthTexture.Create();
            LatestDepth = new float[width * height];
        }

        // ---------------------------------------------------------------- capture

        void FixedUpdate()
        {
            double now = Time.fixedTimeAsDouble;
            if (now < m_NextCaptureTime) return;
            m_NextCaptureTime += 1.0 / rateHz;
            if (m_NextCaptureTime < now) m_NextCaptureTime = now + 1.0 / rateHz; // don't catch up after a hitch
            m_CaptureDue = true;
            m_DueStamp = now;
        }

        void LateUpdate()
        {
            // Rendered after the physics step and all transform updates, so the image shows the stamped state.
            if (!m_CaptureDue) return;
            m_CaptureDue = false;
            if (m_InFlight)
            {
                if (++m_SkippedCaptures % 50 == 1)
                    Debug.LogWarning($"[{k_LogTag}] GPU readback too slow, skipped {m_SkippedCaptures} captures.");
                return;
            }

            m_Left.Render();
            m_Right.Render();
            bool withDepth = publishDepthGroundTruth && m_DepthCamera != null;
            if (withDepth)
            {
                SimCameraChannel.Render(m_DepthCamera, m_DepthBuffer);
                m_LinearDepth.SetTexture("_DepthTex", m_DepthBuffer, RenderTextureSubElement.Depth);
                m_LinearDepth.SetVector("_ClipPlanes", new Vector4(nearClip, farClip, 0f, 0f));
                Graphics.Blit(null, DepthTexture, m_LinearDepth);
            }

            m_InFlight = true;
            m_InFlightStamp = m_DueStamp;
            m_InFlightHasDepth = withDepth;
            m_Pending = withDepth ? 3 : 2;
            m_OnComplete ??= Complete; // cached delegates: no allocation per capture
            m_OnDepth ??= OnDepth;
            m_Left.RequestReadback(m_OnComplete);
            m_Right.RequestReadback(m_OnComplete);
            if (withDepth)
                AsyncGPUReadback.Request(DepthTexture, 0, TextureFormat.RFloat, m_OnDepth);
        }

        void OnDepth(AsyncGPUReadbackRequest request)
        {
            if (!request.hasError)
            {
                var source = request.GetData<float>();
                for (int row = 0; row < height; row++)
                    NativeArray<float>.Copy(source, (height - 1 - row) * width, LatestDepth, row * width, width);
            }
            Complete(request.hasError);
        }

        void Complete(bool failed)
        {
            m_InFlightFailed |= failed;
            if (--m_Pending > 0) return;

            bool ok = !m_InFlightFailed;
            m_InFlight = false;
            m_InFlightFailed = false;
            if (!ok)
            {
                Debug.LogWarning($"[{k_LogTag}] GPU readback failed; pair dropped.");
                return;
            }

            LatestStamp = m_InFlightStamp;
            PairCaptured?.Invoke(LatestStamp);
            Publish(LatestStamp, m_InFlightHasDepth);
        }

        // ---------------------------------------------------------------- ROS

        void Publish(double stamp, bool withDepth)
        {
            // The connector queues the publisher registration behind already queued messages, so anything published
            // while disconnected reaches the endpoint first and is rejected with "Not registered to publish topic".
            if (m_Ros == null || !RosBridgeStatus.IsConnected) return;

            var time = SimCameraChannel.ToTimeMsg(stamp);
            m_Left.Publish(m_Ros, time);
            m_Right.Publish(m_Ros, time);

            if (withDepth)
            {
                var msg = SimCameraChannel.FillImage(m_DepthPool, time, leftFrameId, width, height, "32FC1", 4, null);
                Buffer.BlockCopy(LatestDepth, 0, msg.data, 0, msg.data.Length);
                m_Ros.Publish(depthTopic, msg);
            }
        }
    }
}

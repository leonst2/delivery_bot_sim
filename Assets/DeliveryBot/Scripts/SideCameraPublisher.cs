using System;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Simulated outward-facing side cameras (mono_left_camera / mono_right_camera, 21.7° outward in the URDF) for
    /// side obstacle detection. Put this on the robot root (the setup window does). It adds a Camera to the
    /// "CameraSensor" mounts under both links, renders them at <see cref="rateHz"/> on sim time and publishes rgb8
    /// images plus exact camera_info (zero distortion, P without Tx).
    ///
    /// Captures use the same <see cref="SensorCaptureClock"/> grid as the stereo pair, so at equal rates the side
    /// images carry the same stamp as the stereo pair of the same capture. The live images are registered as
    /// <see cref="ISideCameraFeed"/>s for other Unity components (UI camera panel), without a ROS round trip.
    /// </summary>
    public class SideCameraPublisher : MonoBehaviour
    {
        const string k_LeftLink = "mono_left_camera";
        const string k_RightLink = "mono_right_camera";
        const string k_LogTag = "SideCamera";

        [Header("Camera")]
        public int width = 640;
        public int height = 360;
        [Tooltip("Horizontal field of view. 68.755° (1.2 rad) is the URDF placeholder until the real camera's value is known.")]
        public float horizontalFovDeg = 68.755f;
        public float rateHz = 10f;
        public float nearClip = 0.05f;
        public float farClip = 100f;

        [Header("ROS")]
        public string leftImageTopic = "/mono/left/image_raw";
        public string rightImageTopic = "/mono/right/image_raw";
        public string leftInfoTopic = "/mono/left/camera_info";
        public string rightInfoTopic = "/mono/right/camera_info";
        public string leftFrameId = "mono_left_camera_optical_frame";
        public string rightFrameId = "mono_right_camera_optical_frame";

        /// <summary>One side camera as seen by other Unity components.</summary>
        public sealed class Feed : ISideCameraFeed
        {
            internal readonly SimCameraChannel channel;

            internal Feed(CameraSide side, SimCameraChannel channel)
            {
                Side = side;
                this.channel = channel;
            }

            public CameraSide Side { get; }
            public RenderTexture Texture => channel.Texture;
            public Camera Camera => channel.Camera;
            public StereoIntrinsics Intrinsics => channel.Intrinsics;
            /// <summary>Last capture as rgb8, top row first (ROS layout). Valid when <see cref="Captured"/> fires.</summary>
            public byte[] LatestRgb => channel.Rgb;
            public event Action<double> Captured;

            internal void RaiseCaptured(double stamp) => Captured?.Invoke(stamp);
        }

        public Feed Left { get; private set; }
        public Feed Right { get; private set; }
        /// <summary>Sim time (s) of the last capture; both images share it.</summary>
        public double LatestStamp { get; private set; }
        /// <summary>Raised after both images of a capture are read back, before the per-feed events.</summary>
        public event Action<double> Captured;

        ROSConnection m_Ros;
        SensorCaptureClock m_Clock;
        bool m_CaptureDue;
        double m_DueStamp;

        // One capture in flight at a time: both readbacks complete within a few frames.
        bool m_InFlight;
        double m_InFlightStamp;
        int m_Pending;
        bool m_InFlightFailed;
        int m_SkippedCaptures;
        Action<bool> m_OnComplete;

        void Awake()
        {
            var intrinsics = new StereoIntrinsics(width, height,
                StereoIntrinsics.VerticalFromHorizontalFov(horizontalFovDeg, width, height));
            var left = SimCameraChannel.Create(transform, k_LeftLink, intrinsics, nearClip, farClip, leftFrameId,
                "SideLeft", k_LogTag);
            var right = SimCameraChannel.Create(transform, k_RightLink, intrinsics, nearClip, farClip, rightFrameId,
                "SideRight", k_LogTag);
            if (left == null || right == null)
            {
                left?.Release();
                right?.Release();
                enabled = false;
                return;
            }
            Left = new Feed(CameraSide.Left, left);
            Right = new Feed(CameraSide.Right, right);
            m_OnComplete = Complete;
        }

        void OnEnable()
        {
            if (Left == null) return;
            SideCameraFeeds.Register(Left);
            SideCameraFeeds.Register(Right);
        }

        void OnDisable()
        {
            SideCameraFeeds.Unregister(Left);
            SideCameraFeeds.Unregister(Right);
        }

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            Left.channel.Register(m_Ros, leftImageTopic, leftInfoTopic);
            Right.channel.Register(m_Ros, rightImageTopic, rightInfoTopic);
        }

        void OnValidate()
        {
            width = Mathf.Clamp(width, 16, 4096);
            height = Mathf.Clamp(height, 16, 4096);
            horizontalFovDeg = Mathf.Clamp(horizontalFovDeg, 1f, 170f);
            rateHz = Mathf.Max(0.1f, rateHz);
            nearClip = Mathf.Max(0.001f, nearClip);
            farClip = Mathf.Max(nearClip + 0.1f, farClip);
        }

        void OnDestroy()
        {
            Left?.channel.Release();
            Right?.channel.Release();
        }

        // ---------------------------------------------------------------- capture

        void FixedUpdate()
        {
            double now = Time.fixedTimeAsDouble;
            if (!m_Clock.IsDue(now, rateHz)) return;
            m_CaptureDue = true;
            m_DueStamp = now;
        }

        void LateUpdate()
        {
            // Rendered after the physics step and all transform updates, so the images show the stamped state.
            if (!m_CaptureDue) return;
            m_CaptureDue = false;
            if (m_InFlight)
            {
                if (++m_SkippedCaptures % 50 == 1)
                    Debug.LogWarning($"[{k_LogTag}] GPU readback too slow, skipped {m_SkippedCaptures} captures.");
                return;
            }

            Left.channel.Render();
            Right.channel.Render();
            m_InFlight = true;
            m_InFlightStamp = m_DueStamp;
            m_Pending = 2;
            Left.channel.RequestReadback(m_OnComplete);
            Right.channel.RequestReadback(m_OnComplete);
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
                Debug.LogWarning($"[{k_LogTag}] GPU readback failed; capture dropped.");
                return;
            }

            LatestStamp = m_InFlightStamp;
            Captured?.Invoke(LatestStamp);
            Left.RaiseCaptured(LatestStamp);
            Right.RaiseCaptured(LatestStamp);
            Publish(LatestStamp);
        }

        // ---------------------------------------------------------------- ROS

        void Publish(double stamp)
        {
            // The connector queues the publisher registration behind already queued messages, so anything published
            // while disconnected reaches the endpoint first and is rejected with "Not registered to publish topic".
            if (m_Ros == null || !RosBridgeStatus.IsConnected) return;

            var time = SimCameraChannel.ToTimeMsg(stamp);
            Left.channel.Publish(m_Ros, time);
            Right.channel.Publish(m_Ros, time);
        }
    }
}

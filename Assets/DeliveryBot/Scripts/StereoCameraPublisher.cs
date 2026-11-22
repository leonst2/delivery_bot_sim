using System;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.Universal;

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
        const string k_Mount = "CameraSensor";
        const float k_UrdfBaseline = 0.062f; // m, stereo_left/right_camera joints in the URDF
        const int k_ImageQueueSize = 4; // older images are dropped if the bridge falls behind

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
        MessagePool<ImageMsg> m_LeftPool, m_RightPool;
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
        int m_SkippedCaptures;

        void Awake()
        {
            LeftCamera = SetUpCamera(k_LeftLink);
            RightCamera = SetUpCamera(k_RightLink);
            if (LeftCamera == null || RightCamera == null)
            {
                enabled = false;
                return;
            }

            Baseline = Vector3.Distance(LeftCamera.transform.position, RightCamera.transform.position);
            if (Mathf.Abs(Baseline - k_UrdfBaseline) > 0.001f)
                Debug.LogWarning($"[StereoCamera] Baseline is {Baseline * 1000f:F1} mm, the URDF says {k_UrdfBaseline * 1000f:F0} mm.");

            LeftIntrinsics = new StereoIntrinsics(width, height, verticalFovDeg);
            RightIntrinsics = new StereoIntrinsics(width, height, verticalFovDeg, Baseline);

            LeftTexture = CreateColorTexture("StereoLeft");
            RightTexture = CreateColorTexture("StereoRight");
            LatestLeftRgb = new byte[width * height * 3];
            LatestRightRgb = new byte[width * height * 3];

            if (publishDepthGroundTruth)
                SetUpDepth();
        }

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_LeftPool = RegisterImagePublisher(leftImageTopic);
            m_RightPool = RegisterImagePublisher(rightImageTopic);
            m_Ros.RegisterPublisher<CameraInfoMsg>(leftInfoTopic, k_ImageQueueSize);
            m_Ros.RegisterPublisher<CameraInfoMsg>(rightInfoTopic, k_ImageQueueSize);
            if (publishDepthGroundTruth)
                m_DepthPool = RegisterImagePublisher(depthTopic);
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
            foreach (var texture in new[] { LeftTexture, RightTexture, DepthTexture, m_DepthBuffer })
                if (texture != null) texture.Release();
            if (m_LinearDepth != null) Destroy(m_LinearDepth);
        }

        // ---------------------------------------------------------------- setup

        Camera SetUpCamera(string linkName)
        {
            var mount = FindMount(linkName);
            if (mount == null) return null;
            // Not "??": a missing component is a Unity fake-null, which ?? doesn't see.
            if (!mount.TryGetComponent(out Camera cam))
                cam = mount.gameObject.AddComponent<Camera>();
            ConfigureCamera(cam);
            return cam;
        }

        void ConfigureCamera(Camera cam)
        {
            // Rendered on demand at rateHz (SubmitRenderRequest), not every frame.
            cam.enabled = false;
            cam.usePhysicalProperties = false;
            cam.fieldOfView = verticalFovDeg;
            cam.aspect = (float)width / height;
            cam.nearClipPlane = nearClip;
            cam.farClipPlane = farClip;
            cam.allowHDR = false;
            cam.allowMSAA = false;
        }

        Transform FindMount(string linkName)
        {
            foreach (var t in GetComponentsInChildren<Transform>(true))
            {
                if (t.name != linkName) continue;
                var mount = t.Find(k_Mount);
                if (mount != null) return mount;
            }
            Debug.LogError($"[StereoCamera] No {k_Mount} mount under {linkName}. Run Tools > Delivery Bot > Setup > Configure.");
            return null;
        }

        RenderTexture CreateColorTexture(string name)
        {
            var texture = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            {
                name = name,
                antiAliasing = 1,
            };
            texture.Create();
            return texture;
        }

        void SetUpDepth()
        {
            var shader = Resources.Load<Shader>("DeliveryBot/LinearDepth");
            if (shader == null)
            {
                Debug.LogError("[StereoCamera] Resources/DeliveryBot/LinearDepth.shader missing; depth_gt disabled.");
                publishDepthGroundTruth = false;
                return;
            }
            m_LinearDepth = new Material(shader);

            // A second camera at the left camera's pose that renders into a depth-only target.
            var go = new GameObject("DepthGroundTruthCamera") { layer = LeftCamera.gameObject.layer };
            go.transform.SetParent(LeftCamera.transform, false);
            m_DepthCamera = go.AddComponent<Camera>();
            ConfigureCamera(m_DepthCamera);

            m_DepthBuffer = new RenderTexture(width, height, 32, RenderTextureFormat.Depth) { name = "StereoDepthBuffer" };
            m_DepthBuffer.Create();
            DepthTexture = new RenderTexture(width, height, 0, RenderTextureFormat.RFloat) { name = "StereoDepthGT" };
            DepthTexture.Create();
            LatestDepth = new float[width * height];
        }

        MessagePool<ImageMsg> RegisterImagePublisher(string topic)
        {
            // The connector serializes on its sender thread later, so image buffers are recycled through its pool.
            var pool = new MessagePool<ImageMsg>(k_ImageQueueSize + 2);
            m_Ros.RegisterPublisher<ImageMsg>(topic, k_ImageQueueSize).SetMessagePool(pool);
            return pool;
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
                    Debug.LogWarning($"[StereoCamera] GPU readback too slow, skipped {m_SkippedCaptures} captures.");
                return;
            }

            Render(LeftCamera, LeftTexture);
            Render(RightCamera, RightTexture);
            bool withDepth = publishDepthGroundTruth && m_DepthCamera != null;
            if (withDepth)
            {
                Render(m_DepthCamera, m_DepthBuffer);
                m_LinearDepth.SetTexture("_DepthTex", m_DepthBuffer, RenderTextureSubElement.Depth);
                m_LinearDepth.SetVector("_ClipPlanes", new Vector4(nearClip, farClip, 0f, 0f));
                Graphics.Blit(null, DepthTexture, m_LinearDepth);
            }

            m_InFlight = true;
            m_InFlightStamp = m_DueStamp;
            m_InFlightHasDepth = withDepth;
            m_Pending = withDepth ? 3 : 2;
            AsyncGPUReadback.Request(LeftTexture, 0, TextureFormat.RGBA32, r => OnColor(r, LatestLeftRgb));
            AsyncGPUReadback.Request(RightTexture, 0, TextureFormat.RGBA32, r => OnColor(r, LatestRightRgb));
            if (withDepth)
                AsyncGPUReadback.Request(DepthTexture, 0, TextureFormat.RFloat, OnDepth);
        }

        static void Render(Camera cam, RenderTexture target)
        {
            var request = new UniversalRenderPipeline.SingleCameraRequest { destination = target };
            if (RenderPipeline.SupportsRenderRequest(cam, request))
            {
                RenderPipeline.SubmitRenderRequest(cam, request);
                return;
            }
            var previous = cam.targetTexture;
            cam.targetTexture = target;
            cam.Render();
            cam.targetTexture = previous;
        }

        void OnColor(AsyncGPUReadbackRequest request, byte[] rgb)
        {
            if (!request.hasError)
                FlipRgbaToRgb(request.GetData<byte>(), rgb);
            Complete(request.hasError);
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

        /// <summary>Unity reads textures back bottom row first; ROS images are top row first.</summary>
        void FlipRgbaToRgb(NativeArray<byte> rgba, byte[] rgb)
        {
            for (int row = 0; row < height; row++)
            {
                int src = (height - 1 - row) * width * 4;
                int dst = row * width * 3;
                for (int x = 0; x < width; x++, src += 4, dst += 3)
                {
                    rgb[dst] = rgba[src];
                    rgb[dst + 1] = rgba[src + 1];
                    rgb[dst + 2] = rgba[src + 2];
                }
            }
        }

        bool m_InFlightFailed;

        void Complete(bool failed)
        {
            m_InFlightFailed |= failed;
            if (--m_Pending > 0) return;

            bool ok = !m_InFlightFailed;
            m_InFlight = false;
            m_InFlightFailed = false;
            if (!ok)
            {
                Debug.LogWarning("[StereoCamera] GPU readback failed; pair dropped.");
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

            int sec = (int)Math.Floor(stamp);
            var time = new TimeMsg { sec = sec, nanosec = (uint)((stamp - sec) * 1e9) };

            m_Ros.Publish(leftImageTopic, FillImage(m_LeftPool, time, leftFrameId, "rgb8", 3, LatestLeftRgb));
            m_Ros.Publish(rightImageTopic, FillImage(m_RightPool, time, rightFrameId, "rgb8", 3, LatestRightRgb));
            m_Ros.Publish(leftInfoTopic, LeftIntrinsics.ToCameraInfo(new HeaderMsg { stamp = time, frame_id = leftFrameId }));
            m_Ros.Publish(rightInfoTopic, RightIntrinsics.ToCameraInfo(new HeaderMsg { stamp = time, frame_id = rightFrameId }));

            if (withDepth)
            {
                var msg = FillImage(m_DepthPool, time, leftFrameId, "32FC1", 4, null);
                Buffer.BlockCopy(LatestDepth, 0, msg.data, 0, msg.data.Length);
                m_Ros.Publish(depthTopic, msg);
            }
        }

        ImageMsg FillImage(MessagePool<ImageMsg> pool, TimeMsg time, string frameId, string encoding, int bytesPerPixel,
            byte[] pixels)
        {
            var msg = pool.GetOrCreateMessage();
            int size = width * height * bytesPerPixel;
            if (msg.data == null || msg.data.Length != size)
                msg.data = new byte[size];
            if (pixels != null)
                Buffer.BlockCopy(pixels, 0, msg.data, 0, size);
            msg.header = new HeaderMsg { stamp = time, frame_id = frameId };
            msg.height = (uint)height;
            msg.width = (uint)width;
            msg.encoding = encoding;
            msg.is_bigendian = BitConverter.IsLittleEndian ? (byte)0 : (byte)1;
            msg.step = (uint)(width * bytesPerPixel);
            return msg;
        }
    }
}

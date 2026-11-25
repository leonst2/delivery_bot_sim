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
    /// One simulated pinhole camera: a disabled Unity Camera on a link's "CameraSensor" mount, rendered on demand into
    /// its own RenderTexture, read back asynchronously as rgb8 (top row first) and published as sensor_msgs/Image plus
    /// camera_info. Owned by the camera publisher components, which decide when to capture.
    /// </summary>
    public class SimCameraChannel
    {
        public const string MountName = "CameraSensor";
        const int k_ImageQueueSize = 4; // older images are dropped if the bridge falls behind

        public Camera Camera { get; }
        public RenderTexture Texture { get; }
        /// <summary>Last capture as rgb8, top row first (ROS layout).</summary>
        public byte[] Rgb { get; }
        public StereoIntrinsics Intrinsics { get; }
        public string FrameId { get; }

        readonly int m_Width, m_Height;
        MessagePool<ImageMsg> m_Pool;
        string m_ImageTopic, m_InfoTopic;
        Action<AsyncGPUReadbackRequest> m_OnReadback;
        Action<bool> m_ReadbackDone;

        SimCameraChannel(Camera camera, StereoIntrinsics intrinsics, string frameId, string textureName)
        {
            Camera = camera;
            Intrinsics = intrinsics;
            FrameId = frameId;
            m_Width = intrinsics.Width;
            m_Height = intrinsics.Height;
            Texture = new RenderTexture(m_Width, m_Height, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB)
            {
                name = textureName,
                antiAliasing = 1,
            };
            Texture.Create();
            Rgb = new byte[m_Width * m_Height * 3];
        }

        /// <summary>
        /// Sets up the camera on the CameraSensor mount under <paramref name="linkName"/> below <paramref name="robot"/>.
        /// Returns null (and logs) if the mount doesn't exist.
        /// </summary>
        public static SimCameraChannel Create(Transform robot, string linkName, StereoIntrinsics intrinsics,
            float nearClip, float farClip, string frameId, string textureName, string logTag)
        {
            var mount = FindMount(robot, linkName);
            if (mount == null)
            {
                Debug.LogError($"[{logTag}] No {MountName} mount under {linkName}. Run Tools > Delivery Bot > Setup > Configure.");
                return null;
            }
            // Not "??": a missing component is a Unity fake-null, which ?? doesn't see.
            if (!mount.TryGetComponent(out Camera cam))
                cam = mount.gameObject.AddComponent<Camera>();
            Configure(cam, intrinsics, nearClip, farClip);
            return new SimCameraChannel(cam, intrinsics, frameId, textureName);
        }

        /// <summary>Settings for a camera rendered on demand (SubmitRenderRequest) with these intrinsics.</summary>
        public static void Configure(Camera cam, StereoIntrinsics intrinsics, float nearClip, float farClip)
        {
            cam.enabled = false;
            cam.usePhysicalProperties = false;
            cam.fieldOfView = (float)intrinsics.VerticalFovDeg;
            cam.aspect = (float)intrinsics.Width / intrinsics.Height;
            cam.nearClipPlane = nearClip;
            cam.farClipPlane = farClip;
            cam.allowHDR = false;
            cam.allowMSAA = false;
        }

        /// <summary>The CameraSensor mount under the link named <paramref name="linkName"/>, or null.</summary>
        public static Transform FindMount(Transform robot, string linkName)
        {
            foreach (var t in robot.GetComponentsInChildren<Transform>(true))
            {
                if (t.name != linkName) continue;
                var mount = t.Find(MountName);
                if (mount != null) return mount;
            }
            return null;
        }

        public void Release()
        {
            if (Texture != null) Texture.Release();
        }

        // ---------------------------------------------------------------- capture

        public void Render() => Render(Camera, Texture);

        public static void Render(Camera cam, RenderTexture target)
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

        /// <summary>Reads <see cref="Texture"/> back into <see cref="Rgb"/>; <paramref name="done"/> gets "failed".</summary>
        public void RequestReadback(Action<bool> done)
        {
            m_ReadbackDone = done;
            m_OnReadback ??= OnReadback; // cached, so a capture doesn't allocate a delegate
            AsyncGPUReadback.Request(Texture, 0, TextureFormat.RGBA32, m_OnReadback);
        }

        void OnReadback(AsyncGPUReadbackRequest request)
        {
            if (!request.hasError)
                FlipRgbaToRgb(request.GetData<byte>());
            m_ReadbackDone?.Invoke(request.hasError);
        }

        /// <summary>Unity reads textures back bottom row first; ROS images are top row first.</summary>
        void FlipRgbaToRgb(NativeArray<byte> rgba)
        {
            for (int row = 0; row < m_Height; row++)
            {
                int src = (m_Height - 1 - row) * m_Width * 4;
                int dst = row * m_Width * 3;
                for (int x = 0; x < m_Width; x++, src += 4, dst += 3)
                {
                    Rgb[dst] = rgba[src];
                    Rgb[dst + 1] = rgba[src + 1];
                    Rgb[dst + 2] = rgba[src + 2];
                }
            }
        }

        // ---------------------------------------------------------------- ROS

        public void Register(ROSConnection ros, string imageTopic, string infoTopic)
        {
            m_ImageTopic = imageTopic;
            m_InfoTopic = infoTopic;
            m_Pool = RegisterImagePublisher(ros, imageTopic);
            ros.RegisterPublisher<CameraInfoMsg>(infoTopic, k_ImageQueueSize);
        }

        /// <summary>
        /// Registers an image publisher whose messages are recycled through a pool: the connector serializes on its
        /// sender thread later, so a buffer must not be reused before that.
        /// </summary>
        public static MessagePool<ImageMsg> RegisterImagePublisher(ROSConnection ros, string topic)
        {
            var pool = new MessagePool<ImageMsg>(k_ImageQueueSize + 2);
            ros.RegisterPublisher<ImageMsg>(topic, k_ImageQueueSize).SetMessagePool(pool);
            return pool;
        }

        /// <summary>Publishes the last capture and its camera_info. The caller checks the connection first.</summary>
        public void Publish(ROSConnection ros, TimeMsg time)
        {
            ros.Publish(m_ImageTopic, FillImage(m_Pool, time, FrameId, m_Width, m_Height, "rgb8", 3, Rgb));
            ros.Publish(m_InfoTopic, Intrinsics.ToCameraInfo(new HeaderMsg { stamp = time, frame_id = FrameId }));
        }

        public static TimeMsg ToTimeMsg(double stamp)
        {
            int sec = (int)Math.Floor(stamp);
            return new TimeMsg { sec = sec, nanosec = (uint)((stamp - sec) * 1e9) };
        }

        public static ImageMsg FillImage(MessagePool<ImageMsg> pool, TimeMsg time, string frameId, int width, int height,
            string encoding, int bytesPerPixel, byte[] pixels)
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

using System;
using UnityEngine;

namespace DeliveryBot
{
    public enum CameraSide { Left, Right }

    /// <summary>
    /// One live side camera image (mono_left_camera / mono_right_camera) for display in Unity, without a ROS round
    /// trip. Implemented by the side camera component (#40), consumed by the HUD camera panel (#41).
    /// </summary>
    public interface ISideCameraFeed
    {
        CameraSide Side { get; }

        /// <summary>The camera's RenderTexture, upright as Unity renders it. Null until the first capture.</summary>
        RenderTexture Texture { get; }

        /// <summary>The Unity camera that renders <see cref="Texture"/>; used to project points into the image.</summary>
        Camera Camera { get; }

        /// <summary>Raised after each capture with its sim-time stamp (s).</summary>
        event Action<double> Captured;
    }

    /// <summary>
    /// Registry of the active side camera feeds. A feed calls <see cref="Register"/> in OnEnable and
    /// <see cref="Unregister"/> in OnDisable; consumers read <see cref="Get"/> every frame (no scene search).
    /// </summary>
    public static class SideCameraFeeds
    {
        static readonly ISideCameraFeed[] s_Feeds = new ISideCameraFeed[2];

        public static void Register(ISideCameraFeed feed)
        {
            if (feed != null) s_Feeds[(int)feed.Side] = feed;
        }

        public static void Unregister(ISideCameraFeed feed)
        {
            if (feed != null && s_Feeds[(int)feed.Side] == feed) s_Feeds[(int)feed.Side] = null;
        }

        public static ISideCameraFeed Get(CameraSide side)
        {
            var feed = s_Feeds[(int)side];
            // A destroyed MonoBehaviour that forgot to unregister compares equal to null through UnityEngine.Object.
            if (feed is UnityEngine.Object obj && obj == null) s_Feeds[(int)side] = feed = null;
            return feed;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetOnPlay() => Array.Clear(s_Feeds, 0, s_Feeds.Length);
    }
}

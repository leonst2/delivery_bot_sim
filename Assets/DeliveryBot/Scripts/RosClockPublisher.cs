using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Publishes Unity sim time on /clock every physics step, so all ROS nodes can run with use_sim_time:=true.
    /// FixedUpdate doesn't run while paused, so /clock stops with the simulation.
    /// One instance is created automatically in every scene, because /clock is global and not tied to a robot.
    /// </summary>
    public class RosClockPublisher : MonoBehaviour
    {
        const string k_Topic = "/clock";
        const double k_NanosecondsPerSecond = 1e9;

        ROSConnection m_Ros;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void CreateInstance()
        {
            var bridge = new GameObject("RosBridge");
            bridge.AddComponent<RosClockPublisher>();
            DontDestroyOnLoad(bridge);
        }

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.RegisterPublisher<ClockMsg>(k_Topic);
        }

        void FixedUpdate()
        {
            // The connector queues the publisher registration behind already queued messages, so anything published
            // while disconnected reaches the endpoint first and is rejected with "Not registered to publish topic".
            if (!RosBridgeStatus.IsConnected) return;

            double time = Time.fixedTimeAsDouble;
            int seconds = (int)System.Math.Floor(time);
            uint nanoseconds = (uint)((time - seconds) * k_NanosecondsPerSecond);
            m_Ros.Publish(k_Topic, new ClockMsg(new TimeMsg(seconds, nanoseconds)));
        }
    }
}

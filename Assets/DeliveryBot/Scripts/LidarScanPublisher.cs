using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// Simulated 2D LiDAR. Put this on the "LidarSensor" mount object under lidar_link
/// (NOT on the ArticulationBody). The mount's forward axis (+Z) must point along the
/// robot's forward direction.
///
/// Publishes /scan (sensor_msgs/LaserScan, frame lidar_link, sim time) and exposes the
/// latest scan to other Unity components (UI panel) via LatestRanges / Angles / Timestamp.
/// </summary>
namespace DeliveryBot
{
public class LidarScanPublisher : MonoBehaviour
{
    [Header("ROS")]
    public string topic = "/scan";
    public string frameId = "lidar_link";

    [Header("Lidar Settings")]
    public float fieldOfViewDeg = 360f;
    public float resolutionDeg = 1f;
    public float rangeMin = 0.1f;
    public float rangeMax = 12f;
    public float rateHz = 10f;

    [Header("Noise (deterministic per seed)")]
    public float rangeNoiseSigma = 0.01f; //metres Gaussian
    [Range(0f, 1f)] public float dropoutProbability = 0f;
    public int seed = 0; // needs to be connected to the scenario seed (sim #27)

    [Header("Raycast Mask")]
    [Tooltip("Layers the LiDAR can hit. The 'Robot' layer is removed automatically.")]
    public LayerMask hitMask = ~0;

    [Header("Debug")]
    public bool drawDebugRays = false;

    // ---- Latest scan for other Unity components (UI) ----
    public float[] LatestRanges { get; private set; }
    public float[] Angles { get; private set; }     // radians, in the lidar frame (ROS convention: CCW from forward)
    public double Timestamp { get; private set; }   // sim time, seconds
    public event Action<float[], double> ScanPublished;

    ROSConnection ros; // ROS connection instance
    System.Random rng;
    int beamCount;
    float angleMin, angleIncrement;
    float[] ranges;
    Vector3[] localDirs; // precomputed beam directions in the local frame
    double nextScanTime;

    void Awake()
    {
        // Never hit the robot itself
        int robotLayer = LayerMask.NameToLayer("Robot");
        if (robotLayer >= 0) hitMask &= ~(1 << robotLayer);
        else Debug.LogWarning("Robot layer not found. LiDAR may hit the robot itself.");

        Rebuild();
    }

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topic);
        nextScanTime = Time.fixedTimeAsDouble;
    }

    void OnValidate()
    {
        fieldOfViewDeg = Mathf.Clamp(fieldOfViewDeg, 1f, 360f);
        resolutionDeg = Mathf.Max(0.05f, resolutionDeg);
        rateHz = Mathf.Max(0.1f, rateHz);
        rangeMax = Mathf.Max(rangeMin + 0.01f, rangeMax);
    }

    void Rebuild()
    {
        rng = new System.Random(seed);
        beamCount = Mathf.Max(1, Mathf.RoundToInt(fieldOfViewDeg / resolutionDeg));
        angleIncrement = resolutionDeg * Mathf.Deg2Rad; // convert to radians
        angleMin = -fieldOfViewDeg * 0.5f * Mathf.Deg2Rad;
        
        ranges = new float[beamCount];
        Angles = new float[beamCount];
        localDirs = new Vector3[beamCount];
        for (int i = 0; i < beamCount; i++)
        {
            float angle = angleMin + i * angleIncrement;
            Angles[i] = angle;
            localDirs[i] = new Vector3<FLU>(Mathf.Cos(angle), Mathf.Sin(angle), 0f).toUnity;
        }
        LatestRanges = new float[beamCount];
    }

    void FixedUpdate()
    {
        double now = Time.fixedTimeAsDouble;
        if (now < nextScanTime) return;
        nextScanTime += 1.0 / rateHz;
        if (nextScanTime < now) nextScanTime = now + 1.0 / rateHz; // don't try to catch up after a hitch

        Scan(now);
    }

    void Scan(double simTime)
    {
        Vector3 origin = transform.position;

        for (int i = 0; i < beamCount; i++)
        {
            Vector3 dir = transform.TransformDirection(localDirs[i]);
            float r = float.PositiveInfinity; // no return

            if (Physics.Raycast(origin, dir, out RaycastHit hit, rangeMax, hitMask, QueryTriggerInteraction.Ignore))
            {
                r = hit.distance;
                if (rangeNoiseSigma > 0f) r += (float)(Gaussian() * rangeNoiseSigma);
                if (r < rangeMin || r > rangeMax) r = float.PositiveInfinity;
            }

            if (dropoutProbability > 0f && rng.NextDouble() < dropoutProbability)
                r = float.PositiveInfinity;

            ranges[i] = r;

            if (drawDebugRays)
                Debug.DrawRay(origin, dir * (float.IsInfinity(r) ? rangeMax : r), float.IsInfinity(r) ? Color.gray : Color.red, 1f / rateHz);
        }

        Array.Copy(ranges, LatestRanges, beamCount);
        Timestamp = simTime;
        ScanPublished?.Invoke(LatestRanges, simTime);

        // The connector queues the publisher registration behind already queued messages, so anything published
        // while disconnected reaches the endpoint first and is rejected with "Not registered to publish topic".
        if (!RosBridgeStatus.IsConnected) return;

        int sec = (int)Math.Floor(simTime);
        uint nanosec = (uint)((simTime - sec) * 1e9);

        var msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg { sec = sec, nanosec = nanosec },
                frame_id = frameId
            },
            angle_min = angleMin,
            angle_max = angleMin + (beamCount - 1) * angleIncrement,
            angle_increment = angleIncrement,
            time_increment = 0f,
            scan_time = 1f / rateHz,
            range_min = rangeMin,
            range_max = rangeMax,
            ranges = (float[])ranges.Clone(),
            intensities = new float[0]
        };
        ros.Publish(topic, msg);
    }

    // Box-Muller, seeded RNG => reproducible noise.
    double Gaussian()
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }
}
}

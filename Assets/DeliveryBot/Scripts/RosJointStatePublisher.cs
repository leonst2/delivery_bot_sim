using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Nav;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

/// <summary>
/// Raw proprioceptive data for the own odometry stack (delivery_bot_core#7):
///   /joint_states        sensor_msgs/JointState, 50 Hz, rad and rad/s, URDF joint names
///   /ground_truth/odom   nav_msgs/Odometry, 50 Hz, frame map -> base_link (EVALUATION ONLY)
///
/// Put this on the robot root (the object that holds the URDF joints).
/// Publishes once per physics step (FixedUpdate); Fixed Timestep must be 0.02 s.
/// </summary>
public class ProprioceptionPublisher : MonoBehaviour
{
    [Header("Topics / frames")]
    public string jointStatesTopic = "/joint_states";
    public string groundTruthTopic = "/ground_truth/odom";
    public string mapFrame = "map";
    public string baseFrame = "base_link";

    [Tooltip("Transform whose pose is base_link (origin = ground point under the rear-axle centre). " +
             "If empty, a child named 'base_link' is used.")]
    public Transform baseLink;

    [Header("Sign conventions (flip only if the sign test fails)")]
    public bool invertWheelSigns = false;     // want: positive = forward
    public bool invertSteeringSigns = false;  // want: positive = left

    [Header("Noise (deterministic per seed)")]
    public bool noiseFree = false;
    [Tooltip("Gaussian noise on wheel velocity, rad/s")]
    public float wheelVelocitySigma = 0.05f;
    [Tooltip("Encoder ticks per wheel revolution; wheel position is quantised to this. 0 = off")]
    public int encoderTicksPerRev = 1024;
    [Tooltip("Gaussian noise on steering angle, rad")]
    public float steeringSigma = 0.002f;
    public int seed = 0;                      // hook up to the scenario seed (sim #27)

    enum Kind { Other, Wheel, Steering }

    struct JointRef
    {
        public string name;
        public ArticulationBody body;
        public Kind kind;
    }

    readonly List<JointRef> joints = new List<JointRef>();
    ROSConnection ros;
    System.Random rng;

    bool havePrev;
    Vector3 prevPos;
    Quaternion prevRot;

    void Awake()
    {
        rng = new System.Random(seed);

        // Same lookup idea as DeliveryBotDrive: URDF joint name from UrdfJoint.
        foreach (var uj in GetComponentsInChildren<UrdfJoint>())
        {
            var ab = uj.GetComponent<ArticulationBody>();
            if (ab == null || ab.isRoot || ab.dofCount == 0) continue; // skips fixed joints

            string n = string.IsNullOrEmpty(uj.jointName) ? uj.name : uj.jointName;
            var kind = Kind.Other;
            if (n.EndsWith("wheel_joint")) kind = Kind.Wheel;
            else if (n.EndsWith("steering_joint")) kind = Kind.Steering;

            joints.Add(new JointRef { name = n, body = ab, kind = kind });
        }

        if (baseLink == null)
        {
            foreach (var t in GetComponentsInChildren<Transform>())
                if (t.name == baseFrame) { baseLink = t; break; }
            if (baseLink == null) baseLink = transform;
        }

        Debug.Log($"[Proprioception] {joints.Count} joints, base_link = {baseLink.name}");
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>(jointStatesTopic);
        ros.RegisterPublisher<OdometryMsg>(groundTruthTopic);
    }

    void FixedUpdate()
    {
        double simTime = Time.fixedTimeAsDouble;
        var stamp = ToStamp(simTime);

        PublishJointStates(stamp);
        PublishGroundTruth(stamp);
    }

    // ------------------------------------------------------------------ joint states
    void PublishJointStates(TimeMsg stamp)
    {
        int n = joints.Count;
        var names = new string[n];
        var pos = new double[n];
        var vel = new double[n];

        for (int i = 0; i < n; i++)
        {
            var j = joints[i];
            double p = j.body.jointPosition[0];   // rad (revolute) / m (prismatic)
            double v = j.body.jointVelocity[0];   // rad/s

            if (j.kind == Kind.Wheel)
            {
                if (invertWheelSigns) { p = -p; v = -v; }
                if (!noiseFree)
                {
                    v += Gaussian() * wheelVelocitySigma;
                    if (encoderTicksPerRev > 0)
                    {
                        double step = 2.0 * Math.PI / encoderTicksPerRev;
                        p = Math.Round(p / step) * step;
                    }
                }
            }
            else if (j.kind == Kind.Steering)
            {
                if (invertSteeringSigns) { p = -p; v = -v; }
                if (!noiseFree) p += Gaussian() * steeringSigma;
            }

            names[i] = j.name;
            pos[i] = p;
            vel[i] = v;
        }

        var msg = new JointStateMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = "" },
            name = names,
            position = pos,
            velocity = vel,
            effort = new double[0]
        };
        ros.Publish(jointStatesTopic, msg);
    }

    // ------------------------------------------------------------------ ground truth
    void PublishGroundTruth(TimeMsg stamp)
    {
        Vector3 p = baseLink.position;
        Quaternion q = baseLink.rotation;
        float dt = Time.fixedDeltaTime;

        var msg = new OdometryMsg();
        msg.header.stamp = stamp;
        msg.header.frame_id = mapFrame;
        msg.child_frame_id = baseFrame;

        // Pose: map origin = Unity world origin, converted to ROS FLU.
        var pFlu = p.To<FLU>();
        var qFlu = q.To<FLU>();
        msg.pose.pose.position.x = pFlu.x;
        msg.pose.pose.position.y = pFlu.y;
        msg.pose.pose.position.z = pFlu.z;
        msg.pose.pose.orientation.x = qFlu.x;
        msg.pose.pose.orientation.y = qFlu.y;
        msg.pose.pose.orientation.z = qFlu.z;
        msg.pose.pose.orientation.w = qFlu.w;

        // Twist in the child frame (base_link), by finite differences of the pose.
        if (havePrev && dt > 0f)
        {
            Vector3 vWorld = (p - prevPos) / dt;
            Vector3 vLocal = Quaternion.Inverse(q) * vWorld;           // Unity local: x right, y up, z fwd

            Quaternion dq = q * Quaternion.Inverse(prevRot);           // world-frame rotation step
            dq.ToAngleAxis(out float angDeg, out Vector3 axis);
            if (angDeg > 180f) angDeg -= 360f;
            Vector3 wWorld = axis * (angDeg * Mathf.Deg2Rad / dt);
            Vector3 wLocal = Quaternion.Inverse(q) * wWorld;

            // Unity (x right, y up, z fwd) -> FLU linear: (z, -x, y)
            msg.twist.twist.linear.x = vLocal.z;
            msg.twist.twist.linear.y = -vLocal.x;
            msg.twist.twist.linear.z = vLocal.y;
            // angular (axial vector, handedness flip): (-z, x, -y)  => yaw rate = -w_y, left turn positive
            msg.twist.twist.angular.x = -wLocal.z;
            msg.twist.twist.angular.y = wLocal.x;
            msg.twist.twist.angular.z = -wLocal.y;
        }

        prevPos = p;
        prevRot = q;
        havePrev = true;

        ros.Publish(groundTruthTopic, msg);
    }

    // ------------------------------------------------------------------ helpers
    static TimeMsg ToStamp(double t)
    {
        int sec = (int)Math.Floor(t);
        uint nsec = (uint)((t - sec) * 1e9);
        return new TimeMsg { sec = sec, nanosec = nsec };
    }

    double Gaussian() // Box-Muller, seeded => reproducible
    {
        double u1 = 1.0 - rng.NextDouble();
        double u2 = rng.NextDouble();
        return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
    }
}
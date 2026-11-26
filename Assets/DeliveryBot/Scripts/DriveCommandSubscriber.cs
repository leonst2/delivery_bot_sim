using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.DeliveryBotInterfaces; // generated: Tools > Delivery Bot > Generate ROS Messages

namespace DeliveryBot
{
    /// <summary>
    /// Applies /drive_cmd (delivery_bot_interfaces/AckermannCommand) to the robot:
    ///   speed          m/s at the rear-axle centre
    ///   steering_angle rad, + left (bicycle model)
    /// Commands go through DeliveryBotDrive.SetCommand(speed, steeringAngleDeg).
    ///
    /// Like DeliveryBotKeyboardDrive, it only calls SetCommand while ROS commands are fresh,
    /// plus one stop when they time out, so keyboard driving still works when no ROS
    /// commands arrive.
    /// </summary>
    public class DriveCommandSubscriber : MonoBehaviour
    {
        [Header("ROS")]
        public string topic = "/drive_cmd";

        [Header("Robot")]
        public DeliveryBotDrive drive;

        [Header("Limits")]
        public float maxForwardSpeed = 2.0f;   // m/s
        public float maxReverseSpeed = 0.6f;   // m/s (magnitude)
        [Tooltip("No command for this long (sim time) => the robot halts.")]
        public float commandTimeout = 0.5f;

        [Header("Debug")]
        [Tooltip("Logs how long the robot needs to come to rest (|v| < 0.02 m/s) after a stop.")]
        public bool logStopTime = true;

        /// <summary>True while ROS commands are arriving (not timed out).</summary>
        public bool CommandsFresh { get; private set; }

        const float k_RestSpeed = 0.02f;

        ROSConnection ros;
        float cmdSpeed, cmdSteerDeg;
        double lastRx;
        bool haveCommand;
        bool wasActive;
        bool wasMoving;        // last applied ROS speed was non-zero
        double stopSince = -1;

        void Start()
        {
            if (drive == null) drive = GetComponentInParent<DeliveryBotDrive>();
            if (drive == null) drive = FindFirstObjectByType<DeliveryBotDrive>();
            if (drive == null)
            {
                Debug.LogError("[DriveCmd] No DeliveryBotDrive found.", this);
                enabled = false;
                return;
            }

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<AckermannCommandMsg>(topic, OnCommand);
        }

        void OnDisable()
        {
            haveCommand = false;
        }

        void OnCommand(AckermannCommandMsg msg)
        {
            float speed = (float)msg.speed;
            float steerDeg = (float)msg.steering_angle * Mathf.Rad2Deg;

            // Ignore garbage; the timeout then stops the robot.
            if (!float.IsFinite(speed) || !float.IsFinite(steerDeg)) return;

            cmdSpeed = speed;
            cmdSteerDeg = steerDeg;
            lastRx = Time.timeAsDouble;
            haveCommand = true;
        }

        void FixedUpdate()
        {
            double now = Time.timeAsDouble;
            bool fresh = haveCommand && now - lastRx <= commandTimeout;
            CommandsFresh = fresh;

            if (fresh)
            {
                float speed = Mathf.Clamp(cmdSpeed, -maxReverseSpeed, maxForwardSpeed);
                float steer = Mathf.Clamp(cmdSteerDeg, -drive.MaxSteeringAngle, drive.MaxSteeringAngle);
                drive.SetCommand(speed, steer);

                bool moving = Mathf.Abs(speed) > 1e-3f;
                if (moving) stopSince = -1;
                else if (wasMoving) BeginStopTiming(now);   // moving -> zero command
                wasMoving = moving;
                wasActive = true;
            }
            else if (wasActive)
            {
                // Commands timed out: one stop, then hand control back (keyboard etc.).
                drive.SetCommand(0f, 0f);
                if (wasMoving) BeginStopTiming(now);
                wasMoving = false;
                wasActive = false;
            }

            TrackStop(now);
        }

        void BeginStopTiming(double now)
        {
            if (logStopTime && Mathf.Abs(drive.ForwardSpeed) >= k_RestSpeed) stopSince = now;
        }

        void TrackStop(double now)
        {
            if (stopSince < 0) return;
            if (Mathf.Abs(drive.ForwardSpeed) < k_RestSpeed)
            {
                Debug.Log($"[DriveCmd] at rest {now - stopSince:F2} s after the stop command");
                stopSince = -1;
            }
        }
    }
}

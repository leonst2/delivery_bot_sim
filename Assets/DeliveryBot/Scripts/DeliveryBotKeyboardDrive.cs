using UnityEngine;
using UnityEngine.InputSystem;

namespace DeliveryBot
{
    /// <summary>
    /// Manual driving for testing: W/S or Up/Down = throttle, A/D or Left/Right = steering.
    /// Only sends commands while a key is held (plus one stop command on release), so it doesn't
    /// override commands from other sources such as /cmd_vel.
    /// </summary>
    [RequireComponent(typeof(DeliveryBotDrive))]
    public class DeliveryBotKeyboardDrive : MonoBehaviour
    {
        [Tooltip("Speed at full throttle, m/s. Hub motors allow ~2.9 m/s.")]
        public float maxSpeed = 2f;
        [Tooltip("Speed in reverse, m/s.")]
        public float maxReverseSpeed = 0.6f;

        DeliveryBotDrive m_Drive;
        bool m_WasDriving;

        void Awake()
        {
            m_Drive = GetComponent<DeliveryBotDrive>();
        }

        void Update()
        {
            var kb = Keyboard.current;
            if (kb == null) return;

            float throttle = (kb.wKey.isPressed || kb.upArrowKey.isPressed ? 1f : 0f)
                           - (kb.sKey.isPressed || kb.downArrowKey.isPressed ? 1f : 0f);
            float steer = (kb.aKey.isPressed || kb.leftArrowKey.isPressed ? 1f : 0f)
                        - (kb.dKey.isPressed || kb.rightArrowKey.isPressed ? 1f : 0f);

            bool driving = throttle != 0f || steer != 0f;
            if (driving || m_WasDriving)
            {
                float speed = throttle >= 0f ? throttle * maxSpeed : throttle * maxReverseSpeed;
                m_Drive.SetCommand(speed, steer * m_Drive.MaxSteeringAngle);
            }
            m_WasDriving = driving;
        }
    }
}

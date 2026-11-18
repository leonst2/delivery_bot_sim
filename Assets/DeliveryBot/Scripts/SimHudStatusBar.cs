using RosMessageTypes.DeliveryBotInterfaces;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;

namespace DeliveryBot
{
    public class SimHudStatusBar : MonoBehaviour
    {
        const string k_GridStepTopic = "/grid_step/state";
        const string k_SafetyPlaceholder = "Safety: waiting for /safety/state";
        const string k_HealthPlaceholder = "Stack: waiting for /health";

        [SerializeField] Text m_ConnectionText;
        [SerializeField] Text m_GridStepText;
        [SerializeField] Text m_SafetyText;
        [SerializeField] Text m_HealthText;

        ROSConnection m_Ros;

        public void SetStatusTextReferences(Text connection, Text gridStep, Text safety, Text health)
        {
            m_ConnectionText = connection;
            m_GridStepText = gridStep;
            m_SafetyText = safety;
            m_HealthText = health;
        }

        void Start()
        {
            m_Ros = ROSConnection.GetOrCreateInstance();
            m_Ros.Subscribe<GridStepStateMsg>(k_GridStepTopic, OnGridStepState);
        }

        void Update()
        {
            m_ConnectionText.text = RosBridgeStatus.IsConnected
                ? "ROS: Connected"
                : "ROS: Disconnected";
        }

        void OnGridStepState(GridStepStateMsg message)
        {
            string stateName = string.IsNullOrEmpty(message.state_name)
                ? "UNKNOWN"
                : message.state_name;
            m_GridStepText.text = string.Format(
                "Grid: {0} | Step {1} | Replans {2} | Yields {3}",
                stateName,
                message.step_index,
                message.replans,
                message.yields);
        }

        public void SetSafetyState(string state)
        {
            m_SafetyText.text = string.IsNullOrEmpty(state)
                ? k_SafetyPlaceholder
                : "Safety: " + state;
        }

        public void SetStackHealth(string health)
        {
            m_HealthText.text = string.IsNullOrEmpty(health)
                ? k_HealthPlaceholder
                : "Stack: " + health;
        }
    }
}

using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem.UI;
using UnityEngine.UI;

namespace DeliveryBot
{
    /// <summary>Helpers shared by the HUD panels (camera panel, LiDAR panel).</summary>
    public static class HudPanelUtil
    {
        static readonly Color k_ButtonActive = new Color(0.25f, 0.55f, 0.95f, 0.95f);
        static readonly Color k_ButtonIdle = new Color(1f, 1f, 1f, 0.15f);

        /// <summary>Hides the HUD slot's "SlotPlaceholder" once a panel sits in the slot.</summary>
        public static void HideSlotPlaceholder(Transform panel)
        {
            var placeholder = panel.parent != null ? panel.parent.Find("SlotPlaceholder") : null;
            if (placeholder != null) placeholder.gameObject.SetActive(false);
        }

        /// <summary>The HUD's buttons need an EventSystem; CitySim doesn't have one of its own.</summary>
        public static void EnsureEventSystem()
        {
            if (EventSystem.current != null || Object.FindFirstObjectByType<EventSystem>() != null) return;
            var go = new GameObject("EventSystem", typeof(EventSystem), typeof(InputSystemUIInputModule));
            Object.DontDestroyOnLoad(go);
        }

        public static void HighlightButton(Button button, bool active)
        {
            if (button == null) return;
            var image = button.targetGraphic as Image;
            if (image != null) image.color = active ? k_ButtonActive : k_ButtonIdle;
        }

        public static void SetText(Text text, string value)
        {
            if (text != null && text.text != value) text.text = value;
        }
    }

    /// <summary>Capture rate from consecutive sim-time stamps, smoothed.</summary>
    public struct RateEstimator
    {
        double m_Last;
        bool m_HasLast;
        public double Hz;

        public void Reset()
        {
            m_HasLast = false;
            Hz = 0;
        }

        public void Add(double stamp)
        {
            if (m_HasLast && stamp > m_Last)
            {
                double rate = 1.0 / (stamp - m_Last);
                Hz = Hz <= 0 ? rate : 0.8 * Hz + 0.2 * rate;
            }
            m_Last = stamp;
            m_HasLast = true;
        }
    }
}

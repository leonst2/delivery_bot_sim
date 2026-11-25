using UnityEngine;
using UnityEngine.UI;

namespace DeliveryBot.Editor
{
    /// <summary>uGUI building blocks shared by the HUD panel prefab builders.</summary>
    internal static class HudBuilderUtil
    {
        public static Button CreateButton(RectTransform parent, string name, string label)
        {
            var rect = CreateRect(name, parent);
            var image = rect.gameObject.AddComponent<Image>();
            image.color = new Color(1, 1, 1, 0.15f);
            var button = rect.gameObject.AddComponent<Button>();
            button.targetGraphic = image;
            var text = CreateText(rect, "Label", label, 16, TextAnchor.MiddleCenter);
            Stretch(text.rectTransform, Vector2.zero, Vector2.zero);
            return button;
        }

        public static Text CreateText(Transform parent, string name, string value, int fontSize, TextAnchor alignment)
        {
            var go = new GameObject(name, typeof(RectTransform), typeof(Text));
            go.transform.SetParent(parent, false);
            var text = go.GetComponent<Text>();
            text.text = value;
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = fontSize;
            text.color = Color.white;
            text.alignment = alignment;
            text.horizontalOverflow = HorizontalWrapMode.Overflow;
            text.verticalOverflow = VerticalWrapMode.Truncate;
            text.raycastTarget = false;
            return text;
        }

        public static RectTransform CreateRect(string name, Transform parent)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            return go.GetComponent<RectTransform>();
        }

        public static void Stretch(RectTransform rect, Vector2 offsetMin, Vector2 offsetMax)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = offsetMin;
            rect.offsetMax = offsetMax;
        }

        public static void Band(RectTransform rect, bool top, float height)
        {
            rect.anchorMin = new Vector2(0, top ? 1 : 0);
            rect.anchorMax = new Vector2(1, top ? 1 : 0);
            rect.pivot = new Vector2(0.5f, top ? 1 : 0);
            rect.offsetMin = new Vector2(4, top ? -height : 0);
            rect.offsetMax = new Vector2(-4, top ? 0 : height);
        }
    }
}

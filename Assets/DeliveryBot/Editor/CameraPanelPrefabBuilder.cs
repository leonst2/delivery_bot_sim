using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Build Camera Panel: generates UI/Panels/CameraPanel.prefab (issue #19). The panel goes
    /// into the HUD's CameraSlot; Build Sim HUD nests it there automatically. Change the layout here, not in the prefab.
    /// </summary>
    public static class CameraPanelPrefabBuilder
    {
        public const string PanelFolder = "Assets/DeliveryBot/UI/Panels";
        public const string PrefabPath = PanelFolder + "/CameraPanel.prefab";
        const float k_ButtonBarHeight = 44f;
        const float k_TitleHeight = 26f;
        const float k_CaptionHeight = 22f;

        [MenuItem("Tools/Delivery Bot/Build Camera Panel")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(PanelFolder))
                AssetDatabase.CreateFolder("Assets/DeliveryBot/UI", "Panels");

            var root = new GameObject("CameraPanel", typeof(RectTransform));
            try
            {
                var rect = root.GetComponent<RectTransform>();
                Stretch(rect, Vector2.zero, Vector2.zero);
                var panel = root.AddComponent<CameraPanel>();

                // Button row at the top, status text on its right.
                var bar = CreateRect("ViewButtons", rect);
                bar.anchorMin = new Vector2(0, 1);
                bar.anchorMax = new Vector2(1, 1);
                bar.pivot = new Vector2(0.5f, 1);
                bar.offsetMin = new Vector2(12, -k_ButtonBarHeight - 8);
                bar.offsetMax = new Vector2(-12, -8);
                var buttons = CreateRect("Buttons", bar);
                buttons.anchorMin = Vector2.zero;
                buttons.anchorMax = new Vector2(0.6f, 1);
                buttons.offsetMin = buttons.offsetMax = Vector2.zero;
                var layout = buttons.gameObject.AddComponent<HorizontalLayoutGroup>();
                layout.spacing = 6;
                layout.childControlWidth = layout.childControlHeight = true;
                layout.childForceExpandWidth = layout.childForceExpandHeight = true;
                panel.stereoButton = CreateButton(buttons, "StereoButton", "Stereo");
                panel.leftButton = CreateButton(buttons, "LeftButton", "Left");
                panel.rightButton = CreateButton(buttons, "RightButton", "Right");
                panel.disparityButton = CreateButton(buttons, "DisparityButton", "Disparity");

                panel.statusText = CreateText(bar, "Status", "", 15, TextAnchor.MiddleRight);
                panel.statusText.rectTransform.anchorMin = new Vector2(0.6f, 0);
                panel.statusText.rectTransform.anchorMax = Vector2.one;
                panel.statusText.rectTransform.offsetMin = new Vector2(10, 0);
                panel.statusText.rectTransform.offsetMax = Vector2.zero;

                // Feeds share the remaining area equally; hidden feeds drop out of the layout.
                var feeds = CreateRect("Feeds", rect);
                Stretch(feeds, new Vector2(12, 12), new Vector2(-12, -k_ButtonBarHeight - 16));
                var feedLayout = feeds.gameObject.AddComponent<HorizontalLayoutGroup>();
                feedLayout.spacing = 10;
                feedLayout.childControlWidth = feedLayout.childControlHeight = true;
                feedLayout.childForceExpandWidth = feedLayout.childForceExpandHeight = true;
                panel.left = CreateFeed(feeds, "LeftFeed", "Left");
                panel.right = CreateFeed(feeds, "RightFeed", "Right");
                panel.disparity = CreateFeed(feeds, "DisparityFeed", "Disparity");

                PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            }
            finally
            {
                Object.DestroyImmediate(root);
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[DeliveryBot] Built {PrefabPath}. Run Tools > Delivery Bot > Build Sim HUD to nest it in CameraSlot.");
        }

        static CameraPanel.FeedView CreateFeed(RectTransform parent, string name, string title)
        {
            var feed = CreateRect(name, parent);
            feed.gameObject.AddComponent<LayoutElement>().flexibleWidth = 1;

            var titleText = CreateText(feed, "Title", title, 18, TextAnchor.MiddleLeft);
            titleText.fontStyle = FontStyle.Bold;
            Band(titleText.rectTransform, top: true, k_TitleHeight);

            // Dark letterbox area; the image keeps its aspect ratio inside it.
            var area = CreateRect("ImageArea", feed);
            Stretch(area, new Vector2(0, k_CaptionHeight), new Vector2(0, -k_TitleHeight));
            var letterbox = area.gameObject.AddComponent<Image>();
            letterbox.color = new Color(0f, 0f, 0f, 0.55f);
            letterbox.raycastTarget = false;

            var imageRect = CreateRect("Image", area);
            Stretch(imageRect, Vector2.zero, Vector2.zero);
            var image = imageRect.gameObject.AddComponent<RawImage>();
            image.raycastTarget = false;
            var fitter = imageRect.gameObject.AddComponent<AspectRatioFitter>();
            fitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
            fitter.aspectRatio = 4f / 3f;

            var caption = CreateText(feed, "Caption", "", 14, TextAnchor.MiddleLeft);
            caption.color = new Color(1, 1, 1, 0.75f);
            Band(caption.rectTransform, top: false, k_CaptionHeight);

            return new CameraPanel.FeedView
            {
                root = feed.gameObject,
                image = image,
                fitter = fitter,
                title = titleText,
                caption = caption,
            };
        }

        static Button CreateButton(RectTransform parent, string name, string label)
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

        static Text CreateText(Transform parent, string name, string value, int fontSize, TextAnchor alignment)
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

        static RectTransform CreateRect(string name, Transform parent)
        {
            var go = new GameObject(name, typeof(RectTransform));
            go.transform.SetParent(parent, false);
            return go.GetComponent<RectTransform>();
        }

        static void Stretch(RectTransform rect, Vector2 offsetMin, Vector2 offsetMax)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = offsetMin;
            rect.offsetMax = offsetMax;
        }

        static void Band(RectTransform rect, bool top, float height)
        {
            rect.anchorMin = new Vector2(0, top ? 1 : 0);
            rect.anchorMax = new Vector2(1, top ? 1 : 0);
            rect.pivot = new Vector2(0.5f, top ? 1 : 0);
            rect.offsetMin = new Vector2(4, top ? -height : 0);
            rect.offsetMax = new Vector2(-4, top ? 0 : height);
        }
    }
}

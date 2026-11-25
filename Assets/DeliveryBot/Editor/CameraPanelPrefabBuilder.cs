using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Build Camera Panel: generates UI/Panels/CameraPanel.prefab (issues #19, #41). The panel goes
    /// into the HUD's CameraSlot; Build Sim HUD nests it there automatically. Change the layout here, not in the prefab.
    /// </summary>
    public static class CameraPanelPrefabBuilder
    {
        public const string PanelFolder = "Assets/DeliveryBot/UI/Panels";
        public const string PrefabPath = PanelFolder + "/CameraPanel.prefab";
        const float k_ButtonBarHeight = 44f;
        const float k_TitleHeight = 26f;
        const float k_CaptionHeight = 22f;
        const float k_StatusHeight = 22f;
        const int k_MarkersPerSideFeed = 12;
        const float k_MarkerSize = 12f;

        [MenuItem("Tools/Delivery Bot/Build Camera Panel")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(PanelFolder))
                AssetDatabase.CreateFolder("Assets/DeliveryBot/UI", "Panels");

            var root = new GameObject("CameraPanel", typeof(RectTransform));
            try
            {
                var rect = root.GetComponent<RectTransform>();
                HudBuilderUtil.Stretch(rect, Vector2.zero, Vector2.zero);
                var panel = root.AddComponent<CameraPanel>();

                // Button row at the top, status line at the bottom.
                var bar = HudBuilderUtil.CreateRect("ViewButtons", rect);
                bar.anchorMin = new Vector2(0, 1);
                bar.anchorMax = new Vector2(1, 1);
                bar.pivot = new Vector2(0.5f, 1);
                bar.offsetMin = new Vector2(12, -k_ButtonBarHeight - 8);
                bar.offsetMax = new Vector2(-12, -8);
                var buttons = HudBuilderUtil.CreateRect("Buttons", bar);
                buttons.anchorMin = Vector2.zero;
                buttons.anchorMax = Vector2.one;
                buttons.offsetMin = buttons.offsetMax = Vector2.zero;
                var layout = buttons.gameObject.AddComponent<HorizontalLayoutGroup>();
                layout.spacing = 6;
                layout.childControlWidth = layout.childControlHeight = true;
                layout.childForceExpandWidth = layout.childForceExpandHeight = true;
                panel.stereoButton = HudBuilderUtil.CreateButton(buttons, "StereoButton", "Stereo");
                panel.leftButton = HudBuilderUtil.CreateButton(buttons, "LeftButton", "Stereo L");
                panel.rightButton = HudBuilderUtil.CreateButton(buttons, "RightButton", "Stereo R");
                panel.disparityButton = HudBuilderUtil.CreateButton(buttons, "DisparityButton", "Disparity");
                panel.sideButton = HudBuilderUtil.CreateButton(buttons, "SideButton", "Side");
                panel.detectionsButton = HudBuilderUtil.CreateButton(buttons, "DetectionsButton", "Detections");

                panel.statusText = HudBuilderUtil.CreateText(rect, "Status", "", 15, TextAnchor.MiddleRight);
                var status = panel.statusText.rectTransform;
                status.anchorMin = Vector2.zero;
                status.anchorMax = new Vector2(1, 0);
                status.pivot = new Vector2(0.5f, 0);
                status.offsetMin = new Vector2(12, 8);
                status.offsetMax = new Vector2(-12, 8 + k_StatusHeight);

                // Feeds share the remaining area equally; hidden feeds drop out of the layout. Hierarchy order gives
                // the Side view its left side | stereo | right side order.
                var feeds = HudBuilderUtil.CreateRect("Feeds", rect);
                HudBuilderUtil.Stretch(feeds, new Vector2(12, k_StatusHeight + 12), new Vector2(-12, -k_ButtonBarHeight - 16));
                var feedLayout = feeds.gameObject.AddComponent<HorizontalLayoutGroup>();
                feedLayout.spacing = 10;
                feedLayout.childControlWidth = feedLayout.childControlHeight = true;
                feedLayout.childForceExpandWidth = feedLayout.childForceExpandHeight = true;
                panel.sideLeft = CreateFeed(feeds, "SideLeftFeed", "Left side", 16f / 9f, k_MarkersPerSideFeed);
                panel.left = CreateFeed(feeds, "LeftFeed", "Stereo left", 4f / 3f);
                panel.right = CreateFeed(feeds, "RightFeed", "Stereo right", 4f / 3f);
                panel.sideRight = CreateFeed(feeds, "SideRightFeed", "Right side", 16f / 9f, k_MarkersPerSideFeed);
                panel.disparity = CreateFeed(feeds, "DisparityFeed", "Disparity", 4f / 3f);
                panel.sideLeft.root.SetActive(false);
                panel.sideRight.root.SetActive(false);

                PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            }
            finally
            {
                Object.DestroyImmediate(root);
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[DeliveryBot] Built {PrefabPath}. Run Tools > Delivery Bot > Build Sim HUD to nest it in CameraSlot.");
        }

        static CameraPanel.FeedView CreateFeed(RectTransform parent, string name, string title, float aspect,
            int markerCount = 0)
        {
            var feed = HudBuilderUtil.CreateRect(name, parent);
            feed.gameObject.AddComponent<LayoutElement>().flexibleWidth = 1;

            var titleText = HudBuilderUtil.CreateText(feed, "Title", title, 18, TextAnchor.MiddleLeft);
            titleText.fontStyle = FontStyle.Bold;
            HudBuilderUtil.Band(titleText.rectTransform, top: true, k_TitleHeight);

            // Dark letterbox area; the image keeps its aspect ratio inside it.
            var area = HudBuilderUtil.CreateRect("ImageArea", feed);
            HudBuilderUtil.Stretch(area, new Vector2(0, k_CaptionHeight), new Vector2(0, -k_TitleHeight));
            var letterbox = area.gameObject.AddComponent<Image>();
            letterbox.color = new Color(0f, 0f, 0f, 0.55f);
            letterbox.raycastTarget = false;

            var imageRect = HudBuilderUtil.CreateRect("Image", area);
            HudBuilderUtil.Stretch(imageRect, Vector2.zero, Vector2.zero);
            var image = imageRect.gameObject.AddComponent<RawImage>();
            image.raycastTarget = false;
            var fitter = imageRect.gameObject.AddComponent<AspectRatioFitter>();
            fitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
            fitter.aspectRatio = aspect;

            // Detection markers: a pool positioned by CameraPanel in pixels from the image's bottom-left corner.
            var markers = new RectTransform[markerCount];
            if (markerCount > 0)
            {
                var container = HudBuilderUtil.CreateRect("Markers", imageRect);
                HudBuilderUtil.Stretch(container, Vector2.zero, Vector2.zero);
                for (int i = 0; i < markerCount; i++)
                    markers[i] = CreateMarker(container, i);
            }

            var caption = HudBuilderUtil.CreateText(feed, "Caption", "", 14, TextAnchor.MiddleLeft);
            caption.color = new Color(1, 1, 1, 0.75f);
            HudBuilderUtil.Band(caption.rectTransform, top: false, k_CaptionHeight);

            return new CameraPanel.FeedView
            {
                root = feed.gameObject,
                image = image,
                fitter = fitter,
                title = titleText,
                caption = caption,
                markers = markers,
            };
        }

        static RectTransform CreateMarker(RectTransform parent, int index)
        {
            // A diamond (rotated square) with a dark outline, centred on the detection's ground contact point.
            var marker = HudBuilderUtil.CreateRect($"Marker{index}", parent);
            marker.anchorMin = marker.anchorMax = Vector2.zero;
            marker.pivot = new Vector2(0.5f, 0.5f);
            marker.sizeDelta = new Vector2(k_MarkerSize, k_MarkerSize);
            marker.localRotation = Quaternion.Euler(0f, 0f, 45f);
            var image = marker.gameObject.AddComponent<Image>();
            image.color = new Color(1f, 0.25f, 0.85f, 0.9f);
            image.raycastTarget = false;
            var outline = marker.gameObject.AddComponent<Outline>();
            outline.effectColor = new Color(0f, 0f, 0f, 0.8f);
            outline.effectDistance = new Vector2(2f, -2f);
            marker.gameObject.SetActive(false);
            return marker;
        }
    }
}

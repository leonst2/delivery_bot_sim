using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using static DeliveryBot.Editor.HudBuilderUtil;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Build LiDAR Panel: generates UI/Panels/LidarPanel.prefab (issue #20). The panel goes into
    /// the HUD's LidarSlot; Build Sim HUD nests it there automatically. Change the layout here, not in the prefab.
    /// </summary>
    public static class LidarPanelPrefabBuilder
    {
        public const string PrefabPath = CameraPanelPrefabBuilder.PanelFolder + "/LidarPanel.prefab";
        const float k_ButtonBarHeight = 32f;
        const float k_TextHeight = 20f;
        const float k_Padding = 8f;

        [MenuItem("Tools/Delivery Bot/Build LiDAR Panel")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(CameraPanelPrefabBuilder.PanelFolder))
                AssetDatabase.CreateFolder("Assets/DeliveryBot/UI", "Panels");

            var root = new GameObject("LidarPanel", typeof(RectTransform));
            try
            {
                var rect = root.GetComponent<RectTransform>();
                Stretch(rect, Vector2.zero, Vector2.zero);
                var panel = root.AddComponent<LidarPanel>();

                // Button row at the top: scan sources, then the overlay toggles.
                var bar = CreateRect("Buttons", rect);
                bar.anchorMin = new Vector2(0, 1);
                bar.anchorMax = new Vector2(1, 1);
                bar.pivot = new Vector2(0.5f, 1);
                bar.offsetMin = new Vector2(k_Padding, -k_ButtonBarHeight - k_Padding);
                bar.offsetMax = new Vector2(-k_Padding, -k_Padding);
                var layout = bar.gameObject.AddComponent<HorizontalLayoutGroup>();
                layout.spacing = 4;
                layout.childControlWidth = layout.childControlHeight = true;
                layout.childForceExpandWidth = layout.childForceExpandHeight = true;
                panel.rawButton = SmallButton(bar, "RawButton", "Raw");
                panel.filteredButton = SmallButton(bar, "FilteredButton", "Filtered");
                panel.mockButton = SmallButton(bar, "MockButton", "Mock");
                panel.obstaclesButton = SmallButton(bar, "ObstaclesButton", "Obstacles");
                panel.stopZoneButton = SmallButton(bar, "StopZoneButton", "Stop zone");

                float top = k_ButtonBarHeight + 2 * k_Padding;
                panel.title = CreateText(rect, "Title", "LiDAR", 15, TextAnchor.MiddleLeft);
                panel.title.fontStyle = FontStyle.Bold;
                TopBand(panel.title.rectTransform, top, k_TextHeight);

                panel.statusText = CreateText(rect, "Status", "", 13, TextAnchor.MiddleLeft);
                panel.statusText.color = new Color(1, 1, 1, 0.75f);
                BottomBand(panel.statusText.rectTransform, k_Padding, k_TextHeight);
                panel.caption = CreateText(rect, "Caption", "", 13, TextAnchor.MiddleLeft);
                panel.caption.color = new Color(1, 1, 1, 0.75f);
                BottomBand(panel.caption.rectTransform, k_Padding + k_TextHeight, k_TextHeight);

                // Square plot, letterboxed in the remaining area.
                var area = CreateRect("PlotArea", rect);
                Stretch(area, new Vector2(k_Padding, k_Padding + 2 * k_TextHeight + 4),
                    new Vector2(-k_Padding, -(top + k_TextHeight + 2)));
                var square = CreateRect("Plot", area);
                Stretch(square, Vector2.zero, Vector2.zero);
                var fitter = square.gameObject.AddComponent<AspectRatioFitter>();
                fitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
                fitter.aspectRatio = 1f;
                var background = square.gameObject.AddComponent<Image>();
                background.color = new Color(0f, 0f, 0f, 0.55f);
                background.raycastTarget = false;

                var graphicRect = CreateRect("ScanPlot", square);
                Stretch(graphicRect, Vector2.zero, Vector2.zero);
                panel.plot = graphicRect.gameObject.AddComponent<LidarPlotGraphic>();
                panel.plot.raycastTarget = false;

                // Labels on the brighter rings (every 3rd ring), just right of the forward axis.
                for (float r = 3 * panel.plot.ringStep; r <= panel.plot.maxRange + 1e-3f; r += 3 * panel.plot.ringStep)
                {
                    var label = CreateText(square, $"Ring{r:F0}m", $"{r:F0} m", 11, TextAnchor.LowerLeft);
                    label.color = new Color(1, 1, 1, 0.6f);
                    var lr = label.rectTransform;
                    float y = 0.5f + 0.5f * r / panel.plot.maxRange;
                    lr.anchorMin = lr.anchorMax = new Vector2(0.5f, y);
                    lr.pivot = new Vector2(0f, 1f);
                    lr.sizeDelta = new Vector2(40, 14);
                    lr.anchoredPosition = new Vector2(3, -1);
                }

                PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            }
            finally
            {
                Object.DestroyImmediate(root);
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[DeliveryBot] Built {PrefabPath}. Run Tools > Delivery Bot > Build Sim HUD to nest it in LidarSlot.");
        }

        static Button SmallButton(RectTransform parent, string name, string label)
        {
            var button = CreateButton(parent, name, label);
            button.GetComponentInChildren<Text>().fontSize = 14;
            return button;
        }

        static void TopBand(RectTransform rect, float offsetFromTop, float height)
        {
            rect.anchorMin = new Vector2(0, 1);
            rect.anchorMax = new Vector2(1, 1);
            rect.pivot = new Vector2(0.5f, 1);
            rect.offsetMin = new Vector2(k_Padding + 2, -offsetFromTop - height);
            rect.offsetMax = new Vector2(-k_Padding, -offsetFromTop);
        }

        static void BottomBand(RectTransform rect, float offsetFromBottom, float height)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = new Vector2(1, 0);
            rect.pivot = new Vector2(0.5f, 0);
            rect.offsetMin = new Vector2(k_Padding + 2, offsetFromBottom);
            rect.offsetMax = new Vector2(-k_Padding, offsetFromBottom + height);
        }
    }
}

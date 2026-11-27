using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using static DeliveryBot.Editor.HudBuilderUtil;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Build Minimap Panel: generates UI/Panels/MinimapPanel.prefab (issue #18). The panel goes
    /// into the HUD's MinimapSlot; Build Sim HUD nests it there automatically. Change the layout here, not in the prefab.
    /// </summary>
    public static class MinimapPanelPrefabBuilder
    {
        public const string PrefabPath = CameraPanelPrefabBuilder.PanelFolder + "/MinimapPanel.prefab";
        const float k_ButtonBarHeight = 30f;
        const float k_TextHeight = 20f;
        const float k_Padding = 8f;

        [MenuItem("Tools/Delivery Bot/Build Minimap Panel")]
        public static void Build()
        {
            if (!AssetDatabase.IsValidFolder(CameraPanelPrefabBuilder.PanelFolder))
                AssetDatabase.CreateFolder("Assets/DeliveryBot/UI", "Panels");

            var root = new GameObject("MinimapPanel", typeof(RectTransform));
            try
            {
                var rect = root.GetComponent<RectTransform>();
                Stretch(rect, Vector2.zero, Vector2.zero);
                var panel = root.AddComponent<MinimapPanel>();

                // Button row at the top.
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
                panel.costmapButton = SmallButton(bar, "CostmapButton", "Costmap (M)");
                panel.planButton = SmallButton(bar, "PlanButton", "Plan");
                panel.refreshButton = SmallButton(bar, "RefreshButton", "Refresh");

                float top = k_ButtonBarHeight + 2 * k_Padding;
                panel.title = CreateText(rect, "Title", "Minimap", 15, TextAnchor.MiddleLeft);
                panel.title.fontStyle = FontStyle.Bold;
                TopBand(panel.title.rectTransform, top, k_TextHeight);

                panel.statusText = CreateText(rect, "Status", "", 13, TextAnchor.MiddleLeft);
                panel.statusText.color = new Color(1, 1, 1, 0.75f);
                BottomBand(panel.statusText.rectTransform, k_Padding, k_TextHeight);
                panel.caption = CreateText(rect, "Caption", "", 13, TextAnchor.MiddleLeft);
                panel.caption.color = new Color(1, 1, 1, 0.75f);
                BottomBand(panel.caption.rectTransform, k_Padding + k_TextHeight, k_TextHeight);

                // Map, letterboxed with the city's aspect ratio (set at runtime) in the remaining area.
                var area = CreateRect("MapArea", rect);
                Stretch(area, new Vector2(k_Padding, k_Padding + 2 * k_TextHeight + 4),
                    new Vector2(-k_Padding, -(top + k_TextHeight + 2)));
                var map = CreateRect("Map", area);
                Stretch(map, Vector2.zero, Vector2.zero);
                panel.aspectFitter = map.gameObject.AddComponent<AspectRatioFitter>();
                panel.aspectFitter.aspectMode = AspectRatioFitter.AspectMode.FitInParent;
                panel.aspectFitter.aspectRatio = 1f;
                map.gameObject.AddComponent<RectMask2D>();
                panel.mapArea = map;

                // The city image is the click target; the overlays above it don't take raycasts.
                var city = CreateRect("City", map);
                Stretch(city, Vector2.zero, Vector2.zero);
                panel.cityImage = city.gameObject.AddComponent<RawImage>();
                panel.cityImage.color = new Color(0f, 0f, 0f, 0.55f);
                panel.cityImage.raycastTarget = true;

                var costmap = CreateRect("Costmap", map);
                panel.costmapImage = costmap.gameObject.AddComponent<RawImage>();
                panel.costmapImage.raycastTarget = false;
                panel.costmapImage.enabled = false;

                var markers = CreateRect("Markers", map);
                Stretch(markers, Vector2.zero, Vector2.zero);
                panel.markers = markers.gameObject.AddComponent<MinimapMarkerGraphic>();
                panel.markers.raycastTarget = false;

                PrefabUtility.SaveAsPrefabAsset(root, PrefabPath);
            }
            finally
            {
                Object.DestroyImmediate(root);
            }
            AssetDatabase.SaveAssets();
            Debug.Log($"[DeliveryBot] Built {PrefabPath}. Run Tools > Delivery Bot > Build Sim HUD to nest it in MinimapSlot.");
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

using DeliveryBot;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

namespace DeliveryBot.Editor
{
    static class SimHudPrefabBuilder
    {
        const string k_UiFolder = "Assets/DeliveryBot/UI";
        const string k_PrefabPath = k_UiFolder + "/SimHUD.prefab";
        const string k_MenuItem = "Tools/Delivery Bot/Build Sim HUD";

        [MenuItem(k_MenuItem)]
        static void BuildSimHud()
        {
            EnsureUiFolder();

            var root = new GameObject(
                "SimHUD",
                typeof(RectTransform),
                typeof(Canvas),
                typeof(CanvasScaler),
                typeof(GraphicRaycaster));

            GameObject prefab;
            try
            {
                ConfigureCanvas(root);
                CreatePanelSlot(root.transform, "LidarSlot", "LiDAR panel slot",
                    new Vector2(0.02f, 0.56f), new Vector2(0.27f, 0.96f));
                CreatePanelSlot(root.transform, "MinimapSlot", "Minimap panel slot",
                    new Vector2(0.02f, 0.10f), new Vector2(0.27f, 0.50f));
                CreatePanelSlot(root.transform, "CameraSlot", "Camera panel slot",
                    new Vector2(0.50f, 0.08f), new Vector2(0.99f, 0.96f));
                CreateStatusBar(root.transform);
                prefab = PrefabUtility.SaveAsPrefabAsset(root, k_PrefabPath);
            }
            finally
            {
                Object.DestroyImmediate(root);
            }

            if (prefab == null)
                throw new System.InvalidOperationException("Failed to save the SimHUD prefab.");

            var scene = SceneManager.GetActiveScene();
            if (!scene.IsValid() || string.IsNullOrEmpty(scene.path))
                throw new System.InvalidOperationException("Open a saved scene before building the SimHUD.");

            if (!HasSimHudInstance(scene))
                PrefabUtility.InstantiatePrefab(prefab, scene);

            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
                throw new System.InvalidOperationException("Failed to save the active scene with the SimHUD.");

            AssetDatabase.SaveAssets();
            Debug.Log("Built Assets/DeliveryBot/UI/SimHUD.prefab and added it to the active scene.");
        }

        static void EnsureUiFolder()
        {
            if (!AssetDatabase.IsValidFolder(k_UiFolder))
                AssetDatabase.CreateFolder("Assets/DeliveryBot", "UI");
        }

        static void ConfigureCanvas(GameObject root)
        {
            var canvas = root.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder = 20;

            var scaler = root.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920, 1080);
            scaler.screenMatchMode = CanvasScaler.ScreenMatchMode.MatchWidthOrHeight;
            scaler.matchWidthOrHeight = 0.5f;
        }

        static RectTransform CreatePanelSlot(
            Transform parent,
            string name,
            string placeholder,
            Vector2 anchorMin,
            Vector2 anchorMax)
        {
            var slot = CreateRectObject(name, parent);
            slot.anchorMin = anchorMin;
            slot.anchorMax = anchorMax;
            slot.offsetMin = new Vector2(10, 10);
            slot.offsetMax = new Vector2(-10, -10);

            var background = slot.gameObject.AddComponent<Image>();
            background.color = new Color(0.025f, 0.04f, 0.07f, 0.64f);
            background.raycastTarget = false;

            var label = CreateText(slot, "SlotPlaceholder", placeholder, 20);
            Stretch(label.rectTransform, new Vector2(16, 16), new Vector2(-16, -16));
            label.alignment = TextAnchor.MiddleCenter;

            return slot;
        }

        static void CreateStatusBar(Transform parent)
        {
            var statusBar = CreateRectObject("StatusBar", parent);
            statusBar.anchorMin = new Vector2(0, 0);
            statusBar.anchorMax = new Vector2(1, 0.075f);
            statusBar.offsetMin = Vector2.zero;
            statusBar.offsetMax = Vector2.zero;

            var background = statusBar.gameObject.AddComponent<Image>();
            background.color = new Color(0.015f, 0.025f, 0.04f, 0.92f);
            background.raycastTarget = false;

            var connection = CreateStatusText(statusBar, "ConnectionStatus", "ROS: Disconnected", 0);
            var gridStep = CreateStatusText(statusBar, "GridStepStatus",
                "Grid: waiting for /grid_step/state", 1);
            var safety = CreateStatusText(statusBar, "SafetyStatus",
                "Safety: waiting for /safety/state", 2);
            var health = CreateStatusText(statusBar, "HealthStatus", "Stack: waiting for /health", 3);

            var status = statusBar.gameObject.AddComponent<SimHudStatusBar>();
            status.SetStatusTextReferences(connection, gridStep, safety, health);
        }

        static Text CreateStatusText(RectTransform parent, string name, string text, int segment)
        {
            var label = CreateText(parent, name, text, 17);
            float minX = segment / 4f;
            float maxX = (segment + 1) / 4f;
            label.rectTransform.anchorMin = new Vector2(minX, 0);
            label.rectTransform.anchorMax = new Vector2(maxX, 1);
            label.rectTransform.offsetMin = new Vector2(14, 4);
            label.rectTransform.offsetMax = new Vector2(-8, -4);
            label.alignment = TextAnchor.MiddleLeft;
            return label;
        }

        static Text CreateText(Transform parent, string name, string value, int fontSize)
        {
            var textObject = new GameObject(name, typeof(RectTransform), typeof(Text));
            textObject.transform.SetParent(parent, false);

            var text = textObject.GetComponent<Text>();
            text.text = value;
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = fontSize;
            text.color = Color.white;
            text.horizontalOverflow = HorizontalWrapMode.Wrap;
            text.verticalOverflow = VerticalWrapMode.Truncate;
            text.raycastTarget = false;
            return text;
        }

        static RectTransform CreateRectObject(string name, Transform parent)
        {
            var gameObject = new GameObject(name, typeof(RectTransform));
            gameObject.transform.SetParent(parent, false);
            return gameObject.GetComponent<RectTransform>();
        }

        static void Stretch(RectTransform rect, Vector2 offsetMin, Vector2 offsetMax)
        {
            rect.anchorMin = Vector2.zero;
            rect.anchorMax = Vector2.one;
            rect.offsetMin = offsetMin;
            rect.offsetMax = offsetMax;
        }

        static bool HasSimHudInstance(Scene scene)
        {
            foreach (var root in scene.GetRootGameObjects())
            {
                if (root.name == "SimHUD")
                    return true;
            }

            return false;
        }
    }
}

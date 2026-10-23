using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using Unity.Robotics.UrdfImporter;
using Unity.Robotics.UrdfImporter.Control;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Setup
    ///  1. Project settings (tags, Robot layer, physics timestep).
    ///  2. Scene with a walled test ground.
    ///  3. URDF import (Y-axis) + robot configuration (drive, keyboard control, sensor mounts), saved as a prefab.
    /// </summary>
    public class DeliveryBotSetupWindow : EditorWindow
    {
        public const string Root = "Assets/DeliveryBot";
        public const string ScenePath = Root + "/Scenes/DeliveryBotTraining.unity";
        public const string RobotPrefabPath = Root + "/Prefabs/DeliveryBot.prefab";
        public const string DefaultUrdfPath = "Assets/URDF/delivery_bot/urdf/delivery_bot.urdf";
        public const string RobotLayer = "Robot";
        public const string ObstacleTag = "Obstacle";
        public const string WallTag = "Wall";
        public const string TestGroundName = "TestGround";
        const float TestGroundSize = 20f;

        // Sensor frames from the URDF that get a mount object (sensors can't sit on the ArticulationBody links).
        static readonly (string link, string mount)[] SensorMounts =
        {
            ("lidar_link", "LidarSensor"),
            ("radar_left_link", "RadarSensor"),
            ("radar_right_link", "RadarSensor"),
            ("stereo_left_camera", "CameraSensor"),
            ("stereo_right_camera", "CameraSensor"),
            ("mono_left_camera", "CameraSensor"),
            ("mono_right_camera", "CameraSensor"),
        };

        [SerializeField] string m_UrdfPath = DefaultUrdfPath;

        [MenuItem("Tools/Delivery Bot/Setup", priority = 0)]
        static void Open() => GetWindow<DeliveryBotSetupWindow>("Delivery Bot Setup");

        void OnGUI()
        {
            EditorGUILayout.LabelField("1. Project", EditorStyles.boldLabel);
            if (GUILayout.Button("Apply tags, layer and physics settings"))
                ApplyProjectSettings();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("2. Scene", EditorStyles.boldLabel);
            if (GUILayout.Button("Create / open scene"))
                OpenOrCreateScene();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("3. Robot", EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                m_UrdfPath = EditorGUILayout.TextField("URDF", m_UrdfPath);
                if (GUILayout.Button("…", GUILayout.Width(24)))
                {
                    string picked = EditorUtility.OpenFilePanel("URDF file", "Assets", "urdf");
                    if (!string.IsNullOrEmpty(picked))
                        m_UrdfPath = ToAssetPath(picked);
                }
            }
            if (GUILayout.Button("Import URDF and configure robot"))
                ImportAndConfigure();
            if (GUILayout.Button("Configure selected robot (already imported)"))
            {
                var selected = Selection.activeGameObject;
                if (selected == null || selected.GetComponentInParent<UrdfRobot>() == null)
                    EditorUtility.DisplayDialog("Delivery Bot", "Select the imported robot (the object with UrdfRobot).", "OK");
                else
                    ConfigureRobot(selected.GetComponentInParent<UrdfRobot>().gameObject);
            }
        }

        // ---------------------------------------------------------------- project settings

        public static void ApplyProjectSettings()
        {
            EnsureTag(ObstacleTag);
            EnsureTag(WallTag);
            int layer = EnsureLayer(RobotLayer);
            if (layer >= 0)
                Physics.IgnoreLayerCollision(layer, layer, true);

            // 100 Hz physics keeps the articulated suspension and hub-motor drives stable.
            Time.fixedDeltaTime = 0.01f;
            Time.maximumDeltaTime = 0.1f;
            Physics.defaultSolverIterations = 12;
            Physics.defaultSolverVelocityIterations = 4;
            AssetDatabase.SaveAssets();
            Debug.Log("[DeliveryBot] Project settings applied (tags, Robot layer, fixed timestep 0.01 s).");
        }

        static void EnsureTag(string tag)
        {
            if (UnityEditorInternal.InternalEditorUtility.tags.Contains(tag)) return;
            var tagManager = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
            var tags = tagManager.FindProperty("tags");
            tags.InsertArrayElementAtIndex(tags.arraySize);
            tags.GetArrayElementAtIndex(tags.arraySize - 1).stringValue = tag;
            tagManager.ApplyModifiedPropertiesWithoutUndo();
        }

        static int EnsureLayer(string layerName)
        {
            int existing = LayerMask.NameToLayer(layerName);
            if (existing >= 0) return existing;
            var tagManager = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
            var layers = tagManager.FindProperty("layers");
            for (int i = 8; i < layers.arraySize; i++)
            {
                var element = layers.GetArrayElementAtIndex(i);
                if (string.IsNullOrEmpty(element.stringValue))
                {
                    element.stringValue = layerName;
                    tagManager.ApplyModifiedPropertiesWithoutUndo();
                    return i;
                }
            }
            Debug.LogError("[DeliveryBot] No free user layer for 'Robot'.");
            return -1;
        }

        // ---------------------------------------------------------------- scene

        public static void OpenOrCreateScene()
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;

            if (File.Exists(ScenePath))
            {
                EditorSceneManager.OpenScene(ScenePath);
                return;
            }

            ApplyProjectSettings();
            EnsureFolder(Root + "/Scenes");
            EnsureFolder(Root + "/Materials");

            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            var cam = Camera.main;
            if (cam != null)
            {
                cam.transform.position = new Vector3(0f, 18f, -16f);
                cam.transform.rotation = Quaternion.Euler(50f, 0f, 0f);
            }

            CreateTestGround(Vector3.zero);
            EditorSceneManager.SaveScene(scene, ScenePath);
            AddSceneToBuildSettings(ScenePath);
            Debug.Log($"[DeliveryBot] Scene created at {ScenePath}.");
        }

        /// <summary>Flat floor with walls for drive tests. Replaced by the city scene later.</summary>
        static GameObject CreateTestGround(Vector3 position)
        {
            var groundGo = new GameObject(TestGroundName);
            groundGo.transform.position = position;

            var groundMat = GetMaterial("Ground", new Color(0.55f, 0.55f, 0.52f));
            var wallMat = GetMaterial("Wall", new Color(0.35f, 0.4f, 0.5f));

            const float size = TestGroundSize;
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Floor";
            floor.transform.SetParent(groundGo.transform, false);
            floor.transform.localPosition = new Vector3(0f, -0.05f, 0f);
            floor.transform.localScale = new Vector3(size + 1f, 0.1f, size + 1f);
            floor.GetComponent<Renderer>().sharedMaterial = groundMat;
            floor.GetComponent<Collider>().sharedMaterial = GetPhysicsMaterial();

            const float wallHeight = 1f, wallThickness = 0.2f;
            for (int i = 0; i < 4; i++)
            {
                var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                wall.name = $"Wall_{i}";
                wall.tag = WallTag;
                wall.transform.SetParent(groundGo.transform, false);
                bool alongX = i < 2;
                float offset = (size + wallThickness) * 0.5f * (i % 2 == 0 ? 1f : -1f);
                wall.transform.localPosition = alongX ? new Vector3(0f, wallHeight * 0.5f, offset) : new Vector3(offset, wallHeight * 0.5f, 0f);
                wall.transform.localScale = alongX
                    ? new Vector3(size + 2f * wallThickness, wallHeight, wallThickness)
                    : new Vector3(wallThickness, wallHeight, size);
                wall.GetComponent<Renderer>().sharedMaterial = wallMat;
            }

            Undo.RegisterCreatedObjectUndo(groundGo, "Create Test Ground");
            return groundGo;
        }

        static Material GetMaterial(string name, Color color)
        {
            string path = $"{Root}/Materials/{name}.mat";
            var mat = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (mat != null) return mat;
            var shader = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
            mat = new Material(shader);
            mat.SetColor("_BaseColor", color);
            mat.color = color;
            AssetDatabase.CreateAsset(mat, path);
            return mat;
        }

        static PhysicsMaterial GetPhysicsMaterial()
        {
            string path = $"{Root}/Materials/Floor.physicMaterial";
            var mat = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(path);
            if (mat != null) return mat;
            mat = new PhysicsMaterial("Floor")
            {
                staticFriction = 1.0f,
                dynamicFriction = 0.9f,
                bounciness = 0f,
                frictionCombine = PhysicsMaterialCombine.Average,
            };
            AssetDatabase.CreateAsset(mat, path);
            return mat;
        }

        static void AddSceneToBuildSettings(string path)
        {
            var scenes = EditorBuildSettings.scenes.ToList();
            if (scenes.Any(s => s.path == path)) return;
            scenes.Insert(0, new EditorBuildSettingsScene(path, true));
            EditorBuildSettings.scenes = scenes.ToArray();
        }

        // ---------------------------------------------------------------- URDF import

        void ImportAndConfigure()
        {
            if (!File.Exists(m_UrdfPath))
            {
                EditorUtility.DisplayDialog("Delivery Bot",
                    $"URDF not found at\n{m_UrdfPath}\n\nCopy the whole delivery_bot folder to Assets/URDF/ (or pick the file with …).", "OK");
                return;
            }
            if (SceneManager.GetActiveScene().path != ScenePath)
                OpenOrCreateScene();

            var robot = ImportUrdf(m_UrdfPath);
            if (robot != null)
                ConfigureRobot(robot);
        }

        public static GameObject ImportUrdf(string urdfPath)
        {
            string importPath = PrepareUrdfForImport(urdfPath);

            var settings = new ImportSettings
            {
                chosenAxis = ImportSettings.axisType.yAxis,
                convexMethod = ImportSettings.convexDecomposer.unity,
            };

            Selection.activeObject = null; // the importer parents the robot to the selection
            GameObject robot = null;
            var routine = UrdfRobotExtensions.Create(importPath, settings, false, false);
            while (routine.MoveNext())
                robot = routine.Current ?? robot;

            if (robot == null)
                Debug.LogError($"[DeliveryBot] URDF import failed: {importPath}");
            else
                Debug.Log($"[DeliveryBot] Imported {robot.name} from {importPath}");
            return robot;
        }

        /// <summary>
        /// The importer resolves "package://pkg/..." relative to the folder that contains the .urdf file.
        /// With the ROS layout pkg/urdf/robot.urdf that points to pkg/urdf/pkg/..., so we write a copy of the
        /// URDF next to the package folder, where both package:// and relative paths resolve.
        /// </summary>
        static string PrepareUrdfForImport(string urdfPath)
        {
            string text = File.ReadAllText(urdfPath);
            var match = Regex.Match(text, "filename\\s*=\\s*\"package://([^/\"]+)/");
            if (!match.Success)
                return urdfPath;

            string package = match.Groups[1].Value;
            string dir = Path.GetDirectoryName(urdfPath)?.Replace('\\', '/');
            string packageDir = dir;
            while (!string.IsNullOrEmpty(packageDir) && Path.GetFileName(packageDir) != package)
                packageDir = Path.GetDirectoryName(packageDir)?.Replace('\\', '/');

            if (string.IsNullOrEmpty(packageDir))
            {
                Debug.LogWarning($"[DeliveryBot] URDF references package '{package}' but no parent folder has that name; importing as is.");
                return urdfPath;
            }

            string root = Path.GetDirectoryName(packageDir)?.Replace('\\', '/');
            if (root == dir)
                return urdfPath;

            string copy = $"{root}/{Path.GetFileNameWithoutExtension(urdfPath)}.import.urdf";
            File.WriteAllText(copy, text);
            AssetDatabase.ImportAsset(copy);
            Debug.Log($"[DeliveryBot] Wrote {copy} so package://{package}/ resolves to {packageDir}/.");
            return copy;
        }

        // ---------------------------------------------------------------- robot configuration

        public void ConfigureRobot(GameObject robot)
        {
            ApplyProjectSettings();
            Undo.RegisterFullObjectHierarchyUndo(robot, "Configure Delivery Bot");

            // The importer's keyboard joint controller fights our drives.
            foreach (var controller in robot.GetComponentsInChildren<Controller>(true))
                Object.DestroyImmediate(controller);

            int layer = LayerMask.NameToLayer(RobotLayer);
            foreach (var t in robot.GetComponentsInChildren<Transform>(true))
                t.gameObject.layer = layer;

            // Frame-only links (sensors, optical frames) have no <inertial>; Unity would give each 1 kg.
            foreach (var body in robot.GetComponentsInChildren<ArticulationBody>(true))
            {
                if (body.GetComponent<UrdfInertial>() != null) continue;
                body.mass = 0.01f;
                body.automaticInertiaTensor = false;
                body.inertiaTensor = Vector3.one * 1e-5f;
            }

            GetOrAdd<DeliveryBotDrive>(robot);
            GetOrAdd<DeliveryBotKeyboardDrive>(robot);

            foreach (var (linkName, mountName) in SensorMounts)
            {
                var link = FindLink(robot, linkName);
                if (link != null)
                    Mount(link, mountName);
            }

            // Put the robot onto the test ground of the open scene.
            var ground = GameObject.Find(TestGroundName);
            if (ground != null && robot.transform.parent == null)
            {
                robot.transform.SetParent(ground.transform, false);
                robot.transform.localPosition = new Vector3(0f, 0.02f, 0f);
                robot.transform.localRotation = Quaternion.identity;
            }

            EnsureFolder(Root + "/Prefabs");
            // Re-running: if the robot is nested inside another prefab instance, unpack that first, because Unity
            // can't save a nested part of a prefab instance as its own prefab.
            var outermost = PrefabUtility.GetOutermostPrefabInstanceRoot(robot);
            if (outermost != null && outermost != robot)
                PrefabUtility.UnpackPrefabInstance(outermost, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
            PrefabUtility.SaveAsPrefabAssetAndConnect(robot, RobotPrefabPath, InteractionMode.AutomatedAction);

            EditorSceneManager.MarkSceneDirty(robot.scene);
            EditorSceneManager.SaveScene(robot.scene);
            Selection.activeGameObject = robot;
            Debug.Log($"[DeliveryBot] Robot configured: wheelbase/track/radius are measured at Play. Prefab: {RobotPrefabPath}");
        }

        static GameObject Mount(Transform link, string name)
        {
            var existing = link.Find(name);
            if (existing != null) return existing.gameObject;
            var mount = new GameObject(name) { layer = link.gameObject.layer };
            mount.transform.SetParent(link, false);
            mount.AddComponent<SensorPoseRandomizer>();
            return mount;
        }

        static Transform FindLink(GameObject robot, string linkName)
        {
            var link = robot.GetComponentsInChildren<Transform>(true).FirstOrDefault(t => t.name == linkName);
            if (link == null)
                Debug.LogWarning($"[DeliveryBot] Link '{linkName}' not found; sensor skipped.");
            return link;
        }

        static T GetOrAdd<T>(GameObject go) where T : Component
        {
            return go.TryGetComponent(out T component) ? component : go.AddComponent<T>();
        }

        // ---------------------------------------------------------------- utilities

        static void EnsureFolder(string path)
        {
            if (AssetDatabase.IsValidFolder(path)) return;
            string parent = Path.GetDirectoryName(path)?.Replace('\\', '/');
            if (!string.IsNullOrEmpty(parent)) EnsureFolder(parent);
            AssetDatabase.CreateFolder(parent, Path.GetFileName(path));
        }

        static string ToAssetPath(string absolute)
        {
            string project = Path.GetFullPath(Application.dataPath + "/..").Replace('\\', '/') + "/";
            absolute = absolute.Replace('\\', '/');
            return absolute.StartsWith(project) ? absolute.Substring(project.Length) : absolute;
        }
    }
}

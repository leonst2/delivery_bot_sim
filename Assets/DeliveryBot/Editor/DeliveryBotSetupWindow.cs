using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
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
    ///  2. Training scene with one arena.
    ///  3. URDF import (Y-axis) + robot configuration (drives, agent, lidar/radar, optional cameras), saved as prefabs.
    /// </summary>
    public class DeliveryBotSetupWindow : EditorWindow
    {
        public const string Root = "Assets/DeliveryBot";
        public const string ScenePath = Root + "/Scenes/DeliveryBotTraining.unity";
        public const string RobotPrefabPath = Root + "/Prefabs/DeliveryBot.prefab";
        public const string ArenaPrefabPath = Root + "/Prefabs/TrainingArena.prefab";
        public const string DefaultUrdfPath = "Assets/URDF/delivery_bot/urdf/delivery_bot.urdf";
        public const string RobotLayer = "Robot";
        public const string BehaviorName = "DeliveryBot";

        [SerializeField] string m_UrdfPath = DefaultUrdfPath;
        [SerializeField] bool m_AddRadar = true;
        [SerializeField] bool m_AddCameras;
        [SerializeField] int m_CameraWidth = 84;
        [SerializeField] int m_CameraHeight = 84;
        [SerializeField] int m_ArenaCount = 8;

        [MenuItem("Tools/Delivery Bot/Setup", priority = 0)]
        static void Open() => GetWindow<DeliveryBotSetupWindow>("Delivery Bot Setup");

        void OnGUI()
        {
            EditorGUILayout.LabelField("1. Project", EditorStyles.boldLabel);
            if (GUILayout.Button("Apply tags, layer and physics settings"))
                ApplyProjectSettings();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("2. Scene", EditorStyles.boldLabel);
            if (GUILayout.Button("Create / open training scene"))
                OpenOrCreateTrainingScene();

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
            m_AddRadar = EditorGUILayout.Toggle("Radar ray sensors", m_AddRadar);
            m_AddCameras = EditorGUILayout.Toggle("Camera sensors (slow)", m_AddCameras);
            using (new EditorGUI.DisabledScope(!m_AddCameras))
            {
                m_CameraWidth = EditorGUILayout.IntField("  Width", m_CameraWidth);
                m_CameraHeight = EditorGUILayout.IntField("  Height", m_CameraHeight);
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

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("4. Parallel training", EditorStyles.boldLabel);
            m_ArenaCount = EditorGUILayout.IntSlider("Arenas", m_ArenaCount, 1, 32);
            if (GUILayout.Button("Lay out arena copies"))
                LayOutArenas(m_ArenaCount);
        }

        // ---------------------------------------------------------------- project settings

        public static void ApplyProjectSettings()
        {
            EnsureTag(TrainingArena.ObstacleTag);
            EnsureTag(TrainingArena.WallTag);
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

        public static void OpenOrCreateTrainingScene()
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

            CreateArena(Vector3.zero);
            EditorSceneManager.SaveScene(scene, ScenePath);
            AddSceneToBuildSettings(ScenePath);
            Debug.Log($"[DeliveryBot] Training scene created at {ScenePath}.");
        }

        static TrainingArena CreateArena(Vector3 position)
        {
            var arenaGo = new GameObject("TrainingArena");
            arenaGo.transform.position = position;
            var arena = arenaGo.AddComponent<TrainingArena>();

            var groundMat = GetMaterial("Ground", new Color(0.55f, 0.55f, 0.52f));
            var wallMat = GetMaterial("Wall", new Color(0.35f, 0.4f, 0.5f));
            var goalMat = GetMaterial("Goal", new Color(0.2f, 0.85f, 0.3f));

            float size = arena.size;
            var floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
            floor.name = "Floor";
            floor.transform.SetParent(arenaGo.transform, false);
            floor.transform.localPosition = new Vector3(0f, -0.05f, 0f);
            floor.transform.localScale = new Vector3(size + 1f, 0.1f, size + 1f);
            floor.GetComponent<Renderer>().sharedMaterial = groundMat;
            floor.GetComponent<Collider>().sharedMaterial = GetPhysicsMaterial();

            const float wallHeight = 1f, wallThickness = 0.2f;
            for (int i = 0; i < 4; i++)
            {
                var wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
                wall.name = $"Wall_{i}";
                wall.tag = TrainingArena.WallTag;
                wall.transform.SetParent(arenaGo.transform, false);
                bool alongX = i < 2;
                float offset = (size + wallThickness) * 0.5f * (i % 2 == 0 ? 1f : -1f);
                wall.transform.localPosition = alongX ? new Vector3(0f, wallHeight * 0.5f, offset) : new Vector3(offset, wallHeight * 0.5f, 0f);
                wall.transform.localScale = alongX
                    ? new Vector3(size + 2f * wallThickness, wallHeight, wallThickness)
                    : new Vector3(wallThickness, wallHeight, size);
                wall.GetComponent<Renderer>().sharedMaterial = wallMat;
            }

            var goal = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            goal.name = "Goal";
            Object.DestroyImmediate(goal.GetComponent<Collider>());
            goal.transform.SetParent(arenaGo.transform, false);
            goal.transform.localPosition = new Vector3(0f, 0.01f, 5f);
            goal.transform.localScale = new Vector3(1.2f, 0.01f, 1.2f);
            goal.GetComponent<Renderer>().sharedMaterial = goalMat;
            arena.goal = goal.transform;

            Undo.RegisterCreatedObjectUndo(arenaGo, "Create Training Arena");
            return arena;
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
                OpenOrCreateTrainingScene();

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

            var drive = GetOrAdd<DeliveryBotDrive>(robot);
            var agent = GetOrAdd<DeliveryBotAgent>(robot);
            agent.MaxStep = 3000; // 30 s at 100 Hz

            var behavior = GetOrAdd<BehaviorParameters>(robot);
            behavior.BehaviorName = BehaviorName;
            behavior.BrainParameters.VectorObservationSize = DeliveryBotAgent.VectorObservationSize;
            behavior.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(DeliveryBotAgent.ContinuousActionSize);
            behavior.UseChildSensors = true;

            var requester = GetOrAdd<DecisionRequester>(robot);
            requester.DecisionPeriod = 5; // 20 Hz decisions
            requester.TakeActionsBetweenDecisions = true;

            int rayMask = ~((1 << layer) | (1 << LayerMask.NameToLayer("Ignore Raycast")));
            var tags = new System.Collections.Generic.List<string> { TrainingArena.ObstacleTag, TrainingArena.WallTag };

            var lidarLink = FindLink(robot, "lidar_link");
            if (lidarLink != null)
            {
                var lidar = GetOrAdd<RayPerceptionSensorComponent3D>(Mount(lidarLink, "LidarSensor"));
                lidar.SensorName = "Lidar";
                lidar.DetectableTags = tags;
                lidar.RaysPerDirection = 18; // 37 rays, ~10 deg apart
                lidar.MaxRayDegrees = 180f;
                lidar.RayLength = 12f;
                lidar.SphereCastRadius = 0.05f;
                lidar.RayLayerMask = rayMask;
                lidar.ObservationStacks = 1;
            }

            if (m_AddRadar)
            {
                foreach (var side in new[] { "left", "right" })
                {
                    var link = FindLink(robot, $"radar_{side}_link");
                    if (link == null) continue;
                    var radar = GetOrAdd<RayPerceptionSensorComponent3D>(Mount(link, "RadarSensor"));
                    radar.SensorName = $"Radar_{side}";
                    radar.DetectableTags = tags;
                    radar.RaysPerDirection = 2;
                    radar.MaxRayDegrees = 30f;
                    radar.RayLength = 4f;
                    radar.SphereCastRadius = 0.15f;
                    radar.RayLayerMask = rayMask;
                }
            }

            if (m_AddCameras)
            {
                foreach (var cameraLink in new[] { "stereo_left_camera", "stereo_right_camera", "mono_left_camera", "mono_right_camera" })
                {
                    var link = FindLink(robot, cameraLink);
                    if (link == null) continue;
                    var mount = Mount(link, "CameraSensor");
                    var cam = GetOrAdd<Camera>(mount);
                    cam.fieldOfView = 50f; // PLACEHOLDER: set the real vertical FOV of the camera
                    cam.nearClipPlane = 0.05f;
                    cam.farClipPlane = 30f;
                    cam.enabled = false; // rendered on demand by the sensor
                    var sensor = GetOrAdd<CameraSensorComponent>(mount);
                    sensor.Camera = cam;
                    sensor.SensorName = cameraLink;
                    sensor.Width = m_CameraWidth;
                    sensor.Height = m_CameraHeight;
                    sensor.Grayscale = false;
                }
            }

            // Put the robot into the arena of the open scene.
            var arena = Object.FindFirstObjectByType<TrainingArena>();
            if (arena != null)
            {
                robot.transform.SetParent(arena.transform, false);
                robot.transform.localPosition = new Vector3(0f, 0.02f, 0f);
                robot.transform.localRotation = Quaternion.identity;
                agent.arena = arena;
            }

            EnsureFolder(Root + "/Prefabs");
            // Re-running: unpack so the robot can be saved on its own, then nest it in the arena prefab again.
            var outermost = PrefabUtility.GetOutermostPrefabInstanceRoot(robot);
            if (outermost != null)
                PrefabUtility.UnpackPrefabInstance(outermost, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
            PrefabUtility.SaveAsPrefabAssetAndConnect(robot, RobotPrefabPath, InteractionMode.AutomatedAction);
            if (arena != null)
                PrefabUtility.SaveAsPrefabAssetAndConnect(arena.gameObject, ArenaPrefabPath, InteractionMode.AutomatedAction);

            EditorSceneManager.MarkSceneDirty(robot.scene);
            EditorSceneManager.SaveScene(robot.scene);
            Selection.activeGameObject = robot;
            Debug.Log($"[DeliveryBot] Robot configured: wheelbase/track/radius are measured at Play. Prefabs: {RobotPrefabPath}, {ArenaPrefabPath}");
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

        // ---------------------------------------------------------------- parallel arenas

        public static void LayOutArenas(int count)
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(ArenaPrefabPath);
            if (prefab == null)
            {
                EditorUtility.DisplayDialog("Delivery Bot", "Configure the robot first; that saves the arena prefab.", "OK");
                return;
            }

            var arenas = Object.FindObjectsByType<TrainingArena>(FindObjectsSortMode.InstanceID);
            for (int i = 1; i < arenas.Length; i++)
                Undo.DestroyObjectImmediate(arenas[i].gameObject);

            float spacing = prefab.GetComponent<TrainingArena>().size + 6f;
            int columns = Mathf.CeilToInt(Mathf.Sqrt(count));
            for (int i = 0; i < count; i++)
            {
                var pos = new Vector3((i % columns) * spacing, 0f, (i / columns) * spacing);
                GameObject arena = i == 0 && arenas.Length > 0 ? arenas[0].gameObject : (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                arena.transform.position = pos;
                arena.name = $"TrainingArena_{i}";
                if (i > 0) Undo.RegisterCreatedObjectUndo(arena, "Lay out arenas");
            }
            EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());
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

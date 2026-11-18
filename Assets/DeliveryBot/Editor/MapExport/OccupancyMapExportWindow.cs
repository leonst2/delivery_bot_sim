using System;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace DeliveryBot.MapExport
{
    /// <summary>
    /// Tools > City > Export Occupancy Map: rasterises the loaded scene (the city) into map.pgm + map.yaml
    /// and optionally cuts it into tiles for the offline benchmark corpus (delivery_bot_core#26).
    /// The output is copied into delivery_bot_core/delivery_bot_map_server/maps/ by a PR to that repo.
    /// </summary>
    public class OccupancyMapExportWindow : EditorWindow
    {
        const string PrefsPrefix = "DeliveryBot.MapExport.";
        const string DefaultOutput = "Build/maps/city";

        [SerializeField] RasterSettings m_Settings = new RasterSettings();
        [SerializeField] string m_OutputDir = DefaultOutput;
        [SerializeField] string m_BaseName = "map";
        [SerializeField] bool m_ExportTiles = true;
        [SerializeField] float m_TileSize = 20f;
        [SerializeField] bool m_SkipEmptyTiles = true;

        [MenuItem("Tools/City/Export Occupancy Map", priority = 0)]
        static void Open() => GetWindow<OccupancyMapExportWindow>("Occupancy Map Export");

        void OnEnable()
        {
            m_OutputDir = EditorPrefs.GetString(PrefsPrefix + "OutputDir", DefaultOutput);
            m_BaseName = EditorPrefs.GetString(PrefsPrefix + "BaseName", "map");
            m_Settings.Resolution = ParseDouble(EditorPrefs.GetString(PrefsPrefix + "Resolution", "0.05"), 0.05);
            m_Settings.MinObstacleHeight = EditorPrefs.GetFloat(PrefsPrefix + "MinHeight", 0.08f);
            m_Settings.MaxObstacleHeight = EditorPrefs.GetFloat(PrefsPrefix + "MaxHeight", 0.6f);
            m_ExportTiles = EditorPrefs.GetBool(PrefsPrefix + "ExportTiles", true);
            m_TileSize = EditorPrefs.GetFloat(PrefsPrefix + "TileSize", 20f);
            m_SkipEmptyTiles = EditorPrefs.GetBool(PrefsPrefix + "SkipEmptyTiles", true);
        }

        void OnDisable()
        {
            EditorPrefs.SetString(PrefsPrefix + "OutputDir", m_OutputDir);
            EditorPrefs.SetString(PrefsPrefix + "BaseName", m_BaseName);
            EditorPrefs.SetString(PrefsPrefix + "Resolution", m_Settings.Resolution.ToString("R", CultureInfo.InvariantCulture));
            EditorPrefs.SetFloat(PrefsPrefix + "MinHeight", m_Settings.MinObstacleHeight);
            EditorPrefs.SetFloat(PrefsPrefix + "MaxHeight", m_Settings.MaxObstacleHeight);
            EditorPrefs.SetBool(PrefsPrefix + "ExportTiles", m_ExportTiles);
            EditorPrefs.SetFloat(PrefsPrefix + "TileSize", m_TileSize);
            EditorPrefs.SetBool(PrefsPrefix + "SkipEmptyTiles", m_SkipEmptyTiles);
        }

        void OnGUI()
        {
            EditorGUILayout.LabelField("Map", EditorStyles.boldLabel);
            m_Settings.Resolution = Math.Max(0.005, EditorGUILayout.DoubleField("Resolution (m/cell)", m_Settings.Resolution));
            m_Settings.MinObstacleHeight = EditorGUILayout.FloatField(
                new GUIContent("Obstacle band min (m)", "Geometry below this height is drivable (curbs are 0.05 m)."),
                m_Settings.MinObstacleHeight);
            m_Settings.MaxObstacleHeight = EditorGUILayout.FloatField(
                new GUIContent("Obstacle band max (m)", "Covers LiDAR height (0.515 m) and robot top (0.52 m)."),
                m_Settings.MaxObstacleHeight);
            m_Settings.ExcludedLayers = LayerMaskField(
                new GUIContent("Extra excluded layers", "Robot and Dynamic are always excluded."), m_Settings.ExcludedLayers);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Output", EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                m_OutputDir = EditorGUILayout.TextField("Folder", m_OutputDir);
                if (GUILayout.Button("…", GUILayout.Width(24)))
                {
                    string picked = EditorUtility.OpenFolderPanel("Map output folder", AbsoluteOutputDir(), "");
                    if (!string.IsNullOrEmpty(picked)) m_OutputDir = picked;
                }
            }
            m_BaseName = EditorGUILayout.TextField("File name", m_BaseName);
            m_ExportTiles = EditorGUILayout.Toggle("Export benchmark tiles", m_ExportTiles);
            using (new EditorGUI.DisabledScope(!m_ExportTiles))
            {
                m_TileSize = Mathf.Max(1f, EditorGUILayout.FloatField("Tile size (m)", m_TileSize));
                m_SkipEmptyTiles = EditorGUILayout.Toggle(
                    new GUIContent("Skip tiles without obstacles"), m_SkipEmptyTiles);
            }

            EditorGUILayout.Space();
            EditorGUILayout.HelpBox(
                "Map frame = Unity world origin, axes ROS FLU (x = Unity +Z, y = Unity −X). " +
                "Objects on the Robot and Dynamic layers are never exported.", MessageType.Info);
            if (GUILayout.Button("Export", GUILayout.Height(28)))
                Export();
        }

        string AbsoluteOutputDir() =>
            Path.IsPathRooted(m_OutputDir) ? m_OutputDir : Path.GetFullPath(Path.Combine(Application.dataPath, "..", m_OutputDir));

        void Export()
        {
            try
            {
                var colliders = OccupancyMapRasterizer.CollectStaticColliders(m_Settings);
                foreach (var c in OccupancyMapRasterizer.FindSuspiciousColliders(colliders))
                    Debug.LogWarning($"[MapExport] '{c.name}' has a non-kinematic Rigidbody or an ArticulationBody " +
                                     "but is exported as static. Put moving objects on the Dynamic layer.", c);

                MapRect area = OccupancyMapRasterizer.FootprintOf(colliders);
                var grid = OccupancyMapRasterizer.Rasterize(m_Settings, area,
                    p => EditorUtility.DisplayCancelableProgressBar("Occupancy map", $"Rasterising {area}", p));
                if (grid == null)
                {
                    Debug.Log("[MapExport] Cancelled.");
                    return;
                }

                string dir = AbsoluteOutputDir();
                string comment = Comment();
                grid.Save(dir, m_BaseName, comment);
                int tiles = m_ExportTiles ? SaveTiles(grid, Path.Combine(dir, "tiles"), comment) : 0;

                Debug.Log($"[MapExport] {grid.Width}×{grid.Height} cells @ {grid.Resolution.ToString(CultureInfo.InvariantCulture)} m, origin " +
                          $"[{grid.OriginX.ToString(CultureInfo.InvariantCulture)}, {grid.OriginY.ToString(CultureInfo.InvariantCulture)}], " +
                          $"{grid.Count(OccupancyGrid.Occupied)} occupied / {grid.Count(OccupancyGrid.Free)} free / " +
                          $"{grid.Count(OccupancyGrid.Unknown)} unknown, {colliders.Count} colliders, {tiles} tiles → {dir}");
                EditorUtility.RevealInFinder(Path.Combine(dir, m_BaseName + ".yaml"));
            }
            catch (Exception e)
            {
                Debug.LogException(e);
                EditorUtility.DisplayDialog("Occupancy map", e.Message, "OK");
            }
            finally
            {
                EditorUtility.ClearProgressBar();
            }
        }

        int SaveTiles(OccupancyGrid grid, string dir, string comment)
        {
            if (Directory.Exists(dir))
                foreach (string f in Directory.GetFiles(dir, "tile_*.*")) File.Delete(f);
            int n = Mathf.Max(1, Mathf.RoundToInt(m_TileSize / (float)grid.Resolution));
            int saved = 0;
            for (int row0 = 0, r = 0; row0 < grid.Height; row0 += n, r++)
            for (int col0 = 0, c = 0; col0 < grid.Width; col0 += n, c++)
            {
                var tile = grid.Crop(col0, row0, n, n);
                if (tile.Count(OccupancyGrid.Unknown) == tile.Cells.Length) continue;
                if (m_SkipEmptyTiles && tile.Count(OccupancyGrid.Occupied) == 0) continue;
                tile.Save(dir, $"tile_r{r:D2}_c{c:D2}", comment);
                saved++;
            }
            return saved;
        }

        string Comment()
        {
            string scenes = string.Join(", ", Enumerable.Range(0, EditorSceneManager.sceneCount)
                .Select(i => EditorSceneManager.GetSceneAt(i)).Where(s => s.isLoaded).Select(s => s.path));
            return "Exported by Tools > City > Export Occupancy Map (delivery_bot_sim)\n" +
                   $"scenes: {scenes}\n" +
                   "frame: map = Unity world origin, axes ROS FLU\n" +
                   string.Format(CultureInfo.InvariantCulture, "obstacle band: [{0}, {1}] m above Unity Y = 0",
                       m_Settings.MinObstacleHeight, m_Settings.MaxObstacleHeight);
        }

        static double ParseDouble(string s, double fallback) =>
            double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) && v > 0 ? v : fallback;

        static LayerMask LayerMaskField(GUIContent label, LayerMask mask)
        {
            var names = Enumerable.Range(0, 32).Select(LayerMask.LayerToName).ToArray();
            var used = Enumerable.Range(0, 32).Where(i => !string.IsNullOrEmpty(names[i])).ToArray();
            int compact = 0;
            for (int i = 0; i < used.Length; i++)
                if ((mask.value & (1 << used[i])) != 0) compact |= 1 << i;
            compact = EditorGUILayout.MaskField(label, compact, used.Select(i => names[i]).ToArray());
            int full = 0;
            for (int i = 0; i < used.Length; i++)
                if ((compact & (1 << i)) != 0) full |= 1 << used[i];
            mask.value = full;
            return mask;
        }
    }
}

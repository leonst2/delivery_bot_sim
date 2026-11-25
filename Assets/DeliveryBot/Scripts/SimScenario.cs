using System;
using System.Collections.Generic;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// A named scenario loaded by /sim/load_scenario: the objects the simulation adds to the static city. All poses are
    /// in the map frame (x, y in metres, yaw in rad counter-clockwise; map = Unity world origin with ROS axes).
    /// The robot's start pose and the goal are not part of it: the scenario runner (delivery_bot_core#24) sends them via
    /// /sim/reset and the navigation interface.
    ///
    /// Definitions live in Resources/DeliveryBot/Scenarios/&lt;name&gt;.json, so standalone builds have them too.
    /// </summary>
    [Serializable]
    public class SimScenario
    {
        public const string ResourceFolder = "DeliveryBot/Scenarios";

        public string name;
        public string description;
        public StaticObstacle[] staticObstacles = Array.Empty<StaticObstacle>();
        public Cyclist[] cyclists = Array.Empty<Cyclist>();

        public bool NeedsDynamicObjects => cyclists != null && cyclists.Length > 0;

        /// <summary>A box standing on the ground.</summary>
        [Serializable]
        public class StaticObstacle
        {
            public string name;
            public float x, y, yaw;
            [Tooltip("Size along the box's own x (m).")]
            public float length;
            [Tooltip("Size along the box's own y (m).")]
            public float width;
            public float height;
        }

        /// <summary>A cyclist for the dynamic object spawner (delivery_bot_sim#16).</summary>
        [Serializable]
        public class Cyclist
        {
            public string name;
            public Waypoint[] route = Array.Empty<Waypoint>();
            [Tooltip("m/s")]
            public float speed = 4f;
            [Tooltip("Sim seconds after the scenario is loaded before the cyclist starts.")]
            public float startDelay;
            [Tooltip("Start again from the first waypoint at the end of the route instead of leaving.")]
            public bool loop;
            [Tooltip("Uniform random offset of the speed in [-speedJitter, +speedJitter] (m/s), drawn from the seed.")]
            public float speedJitter;
            [Tooltip("Uniform random offset of the start delay in [-startDelayJitter, +startDelayJitter] (s), from the seed.")]
            public float startDelayJitter;
        }

        [Serializable]
        public struct Waypoint
        {
            public float x, y;
        }

        /// <summary>Loads Resources/DeliveryBot/Scenarios/&lt;name&gt;.json; null if there is none.</summary>
        public static SimScenario Load(string scenarioName)
        {
            if (string.IsNullOrWhiteSpace(scenarioName)) return null;
            var asset = Resources.Load<TextAsset>(ResourceFolder + "/" + scenarioName);
            if (asset == null) return null;
            var scenario = JsonUtility.FromJson<SimScenario>(asset.text);
            if (string.IsNullOrEmpty(scenario.name)) scenario.name = scenarioName;
            scenario.staticObstacles ??= Array.Empty<StaticObstacle>();
            scenario.cyclists ??= Array.Empty<Cyclist>();
            return scenario;
        }

        /// <summary>Names of all scenario definitions, sorted.</summary>
        public static List<string> AvailableNames()
        {
            var names = new List<string>();
            foreach (var asset in Resources.LoadAll<TextAsset>(ResourceFolder))
                names.Add(asset.name);
            names.Sort(StringComparer.Ordinal);
            return names;
        }
    }

    /// <summary>
    /// Spawns and removes the dynamic objects (cyclists) of a scenario. Implemented by delivery_bot_sim#16 and registered
    /// with <see cref="DynamicObjectSpawners.Register"/>; until then scenarios with cyclists can't be loaded.
    /// </summary>
    public interface IDynamicObjectSpawner
    {
        /// <summary>Spawns the scenario's dynamic objects. Draw randomness only from <paramref name="rng"/>.</summary>
        void Spawn(SimScenario scenario, System.Random rng);

        /// <summary>Removes every dynamic object.</summary>
        void Clear();
    }

    public static class DynamicObjectSpawners
    {
        public static IDynamicObjectSpawner Current { get; private set; }

        public static void Register(IDynamicObjectSpawner spawner) => Current = spawner;

        public static void Unregister(IDynamicObjectSpawner spawner)
        {
            if (Current == spawner) Current = null;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetOnPlay() => Current = null;
    }
}

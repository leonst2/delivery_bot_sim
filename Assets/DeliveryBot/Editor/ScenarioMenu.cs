using UnityEditor;
using UnityEngine;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Tools > Delivery Bot > Load Scenario: loads a scenario in Play mode without ROS, through the same
    /// <see cref="SimControl.LoadScenario"/> that /sim/load_scenario calls (seed 42). For checking the scenarios, the
    /// cyclists and the HUD by hand; the scenario runner still goes through the ROS service.
    /// </summary>
    static class ScenarioMenu
    {
        const string k_Menu = "Tools/Delivery Bot/Load Scenario/";
        const uint k_Seed = 42;

        [MenuItem(k_Menu + "empty_street")] static void EmptyStreet() => Load("empty_street");
        [MenuItem(k_Menu + "static_obstacles")] static void StaticObstacles() => Load("static_obstacles");
        [MenuItem(k_Menu + "narrow_passage")] static void NarrowPassage() => Load("narrow_passage");
        [MenuItem(k_Menu + "u_turn")] static void UTurn() => Load("u_turn");
        [MenuItem(k_Menu + "multi_stop")] static void MultiStop() => Load("multi_stop");
        [MenuItem(k_Menu + "cyclist_crossing")] static void CyclistCrossing() => Load("cyclist_crossing");
        [MenuItem(k_Menu + "cyclist_oncoming")] static void CyclistOncoming() => Load("cyclist_oncoming");
        [MenuItem(k_Menu + "cyclist_overtaking")] static void CyclistOvertaking() => Load("cyclist_overtaking");

        [MenuItem(k_Menu + "empty_street", true)]
        [MenuItem(k_Menu + "static_obstacles", true)]
        [MenuItem(k_Menu + "narrow_passage", true)]
        [MenuItem(k_Menu + "u_turn", true)]
        [MenuItem(k_Menu + "multi_stop", true)]
        [MenuItem(k_Menu + "cyclist_crossing", true)]
        [MenuItem(k_Menu + "cyclist_oncoming", true)]
        [MenuItem(k_Menu + "cyclist_overtaking", true)]
        static bool CanLoad() => EditorApplication.isPlaying;

        static void Load(string scenario)
        {
            var control = Object.FindFirstObjectByType<SimControl>();
            if (control == null)
            {
                Debug.LogWarning("[Scenario] No SimControl in the scene; open CitySim and press Play.");
                return;
            }
            bool ok = control.LoadScenario(scenario, k_Seed, out string message);
            if (ok) Debug.Log($"[Scenario] {message}");
            else Debug.LogWarning($"[Scenario] {message}");
        }
    }
}

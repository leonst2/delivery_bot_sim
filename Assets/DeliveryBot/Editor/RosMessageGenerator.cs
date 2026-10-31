using System.IO;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEditor;
using UnityEngine;

namespace DeliveryBot.Editor
{
    /// <summary>
    /// Regenerates the C# types of delivery_bot_interfaces from the pinned submodule. Run it after bumping
    /// delivery_bot_core if the interfaces changed, then commit Assets/DeliveryBot/RosMessages.
    /// Actions are left out: Unity only uses the messages and services.
    /// </summary>
    public static class RosMessageGenerator
    {
        const string InterfacesPath = "ros2_ws/src/delivery_bot_core/delivery_bot_interfaces";
        const string OutputPath = "Assets/DeliveryBot/RosMessages";

        [MenuItem("Tools/Delivery Bot/Generate ROS Messages", priority = 1)]
        public static void Generate()
        {
            string projectRoot = Path.GetDirectoryName(Application.dataPath);
            string source = Path.Combine(projectRoot, InterfacesPath);
            string output = Path.Combine(projectRoot, OutputPath);
            if (!Directory.Exists(source))
            {
                Debug.LogError($"[DeliveryBot] {InterfacesPath} not found. Run: git submodule update --init --recursive");
                return;
            }

            var warnings = MessageAutoGen.GeneratePackageMessages(source, output);
            warnings.AddRange(ServiceAutoGen.GeneratePackageServices(source, output));
            foreach (string warning in warnings)
                Debug.LogWarning("[DeliveryBot] " + warning);

            AssetDatabase.Refresh();
            Debug.Log($"[DeliveryBot] Generated ROS messages in {OutputPath}.");
        }
    }
}

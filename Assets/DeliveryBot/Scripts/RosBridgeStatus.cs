using Unity.Robotics.ROSTCPConnector;

namespace DeliveryBot
{
    /// <summary>
    /// Connection state of the ROS-TCP bridge, for other Unity components such as the UI.
    /// Doesn't use ROSConnection.LastMessageReceivedRealtime: keepalive replies don't update it, so a connection
    /// that only publishes would look dead.
    /// </summary>
    public static class RosBridgeStatus
    {
        public static bool IsConnected
        {
            get
            {
                var ros = ROSConnection.GetOrCreateInstance();
                return ros.HasConnectionThread && !ros.HasConnectionError;
            }
        }
    }
}

using UnityEngine;

namespace DeliveryBot
{
    /// <summary>Forwards collisions of one robot link to the agent.</summary>
    public class LinkCollisionReporter : MonoBehaviour
    {
        public DeliveryBotAgent agent;

        void OnCollisionEnter(Collision collision)
        {
            if (agent != null)
                agent.OnLinkCollision(collision);
        }
    }
}

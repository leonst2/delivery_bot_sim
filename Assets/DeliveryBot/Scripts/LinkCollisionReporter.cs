using System;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>Forwards collisions of one robot link (added to each ArticulationBody by <see cref="DeliveryBotDrive"/>).</summary>
    public class LinkCollisionReporter : MonoBehaviour
    {
        public event Action<Collision> CollisionEntered;

        void OnCollisionEnter(Collision collision)
        {
            CollisionEntered?.Invoke(collision);
        }
    }
}

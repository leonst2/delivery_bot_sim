using System;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// The one seed behind every random source of the simulation (sensor noise, pose jitter, dynamic-object spawning).
    /// Set by /sim/reset and /sim/load_scenario (<see cref="SimControl"/>).
    ///
    /// Each source asks for its own named stream, so sources don't depend on each other or on the order in which they
    /// draw numbers: the same seed and the same stream name always give the same sequence. Components recreate their
    /// streams when <see cref="Reseeded"/> fires. Don't use UnityEngine.Random or unseeded System.Random in the sim.
    /// </summary>
    public static class SimRandom
    {
        /// <summary>The current seed. 0 until a reset or scenario sets one.</summary>
        public static uint Seed { get; private set; }

        /// <summary>Raised after <see cref="SetSeed"/>; recreate your streams here.</summary>
        public static event Action Reseeded;

        public static void SetSeed(uint seed)
        {
            Seed = seed;
            Reseeded?.Invoke();
        }

        /// <summary>A new generator for the stream <paramref name="name"/> under the current seed.</summary>
        public static System.Random Stream(string name) => new System.Random(StreamSeed(Seed, name));

        /// <summary>
        /// FNV-1a over the seed and the stream name. Fixed across runtimes, unlike string.GetHashCode.
        /// </summary>
        public static int StreamSeed(uint seed, string name)
        {
            const uint prime = 16777619;
            uint hash = 2166136261;
            for (int i = 0; i < 4; i++)
            {
                hash ^= (seed >> (8 * i)) & 0xff;
                hash *= prime;
            }
            foreach (char c in name ?? string.Empty)
            {
                hash ^= c & 0xffu;
                hash *= prime;
                hash ^= (uint)c >> 8;
                hash *= prime;
            }
            return (int)(hash & 0x7fffffff);
        }

        /// <summary>Standard normal sample (Box-Muller) from <paramref name="rng"/>.</summary>
        public static double Gaussian(System.Random rng)
        {
            double u1 = 1.0 - rng.NextDouble();
            double u2 = rng.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        static void ResetOnPlay()
        {
            Seed = 0;
            Reseeded = null;
        }
    }
}

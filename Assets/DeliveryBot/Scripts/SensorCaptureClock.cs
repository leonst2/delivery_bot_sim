using System;

namespace DeliveryBot
{
    /// <summary>
    /// Decides on the physics clock when a sensor running at a given rate captures. Captures fall on the sim-time
    /// grid k / rateHz (absolute, not relative to when the component started), so independent sensors with equal
    /// rates capture on the same physics step and carry the same stamp. Missed ticks are not caught up.
    /// </summary>
    public struct SensorCaptureClock
    {
        const double k_Epsilon = 1e-3;

        long m_LastTick;
        bool m_Started;

        /// <summary>Call once per FixedUpdate with Time.fixedTimeAsDouble; true on the step a new tick starts.</summary>
        public bool IsDue(double fixedTime, double rateHz)
        {
            // fixedTime accumulates the float fixedDeltaTime, so 8.2 s arrives as 8.1999998. The epsilon (in ticks)
            // absorbs that and is still far below one physics step (0.2 ticks at 10 Hz on 50 Hz physics).
            double ticks = fixedTime * rateHz;
            long tick = (long)Math.Floor(ticks + k_Epsilon);
            if (!m_Started)
            {
                // The first step only captures if it lies on the grid, so a sensor enabled late stays in step.
                m_Started = true;
                m_LastTick = tick;
                return Math.Abs(ticks - tick) < k_Epsilon;
            }
            if (tick <= m_LastTick) return false;
            m_LastTick = tick;
            return true;
        }
    }
}

using System;
using System.Collections.Generic;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Geometry of a planned path for display: arc length, driving direction per segment and the split into runs
    /// (forward / reverse, inside / outside the current step). Plain C#, 2D in the path's own frame (map, metres).
    /// Deterministic: no randomness, fixed tie-breaking.
    /// </summary>
    public class PlanPathGeometry
    {
        /// <summary>A contiguous piece of the path drawn in one style.</summary>
        public class Run
        {
            public bool reverse;
            public bool inStep;
            public readonly List<Vector2> points = new List<Vector2>();
            /// <summary>Path height (m) at each point, interpolated like the points.</summary>
            public readonly List<float> heights = new List<float>();
        }

        const float k_MinSegment = 1e-3f; // m; shorter segments inherit the previous direction
        const float k_ReprojectDistance = 1f; // m; farther off than this, the projection searches the whole path

        Vector2[] m_Points = Array.Empty<Vector2>();
        float[] m_Heights = Array.Empty<float>();
        float[] m_ArcLength = Array.Empty<float>();
        bool[] m_Reverse = Array.Empty<bool>(); // per segment i -> i + 1
        int m_Count;
        int m_LastSegment;

        public int Count => m_Count;
        public float Length => m_Count > 0 ? m_ArcLength[m_Count - 1] : 0f;
        public Vector2 Point(int i) => m_Points[i];
        public bool SegmentIsReverse(int i) => m_Reverse[i];

        /// <param name="xy">Positions (m).</param>
        /// <param name="heights">Heights (m), or null for 0.</param>
        /// <param name="yaw">Heading of each pose (rad, counter-clockwise from +x).</param>
        public void SetPath(Vector2[] xy, float[] heights, float[] yaw, int count)
        {
            Grow(count);
            m_Count = count;
            m_LastSegment = 0;
            if (count == 0) return;

            Array.Copy(xy, m_Points, count);
            for (int i = 0; i < count; i++)
                m_Heights[i] = heights != null ? heights[i] : 0f;

            m_ArcLength[0] = 0f;
            bool previousReverse = false;
            for (int i = 0; i + 1 < count; i++)
            {
                Vector2 d = m_Points[i + 1] - m_Points[i];
                float length = d.magnitude;
                m_ArcLength[i + 1] = m_ArcLength[i] + length;
                // A segment is driven in reverse when it points against the pose's heading.
                bool reverse = length < k_MinSegment
                    ? previousReverse
                    : Vector2.Dot(d, new Vector2(Mathf.Cos(yaw[i]), Mathf.Sin(yaw[i]))) < 0f;
                m_Reverse[i] = reverse;
                previousReverse = reverse;
            }
        }

        void Grow(int count)
        {
            if (m_Points.Length >= count) return;
            int size = Mathf.NextPowerOfTwo(Mathf.Max(count, 16));
            m_Points = new Vector2[size];
            m_Heights = new float[size];
            m_ArcLength = new float[size];
            m_Reverse = new bool[size];
        }

        /// <summary>
        /// Arc length (m) of the point on the path closest to <paramref name="position"/>. Searches forward from the
        /// last result (the robot moves along the path) and over the whole path when it is far off.
        /// </summary>
        public float Project(Vector2 position)
        {
            if (m_Count == 0) return 0f;
            if (m_Count == 1) return 0f;

            int segments = m_Count - 1;
            int start = Mathf.Clamp(m_LastSegment, 0, segments - 1);
            int best = -1;
            float bestDistance = float.MaxValue, bestT = 0f;

            // Local search: from the last segment forward while the distance doesn't grow by much.
            for (int i = start; i < segments; i++)
            {
                float distance = DistanceToSegment(i, position, out float t);
                if (distance < bestDistance)
                {
                    best = i;
                    bestDistance = distance;
                    bestT = t;
                }
                else if (distance > bestDistance + k_ReprojectDistance)
                {
                    break;
                }
            }
            if (bestDistance > k_ReprojectDistance)
            {
                for (int i = 0; i < segments; i++)
                {
                    float distance = DistanceToSegment(i, position, out float t);
                    if (distance < bestDistance) // strict: the first of equal candidates wins
                    {
                        best = i;
                        bestDistance = distance;
                        bestT = t;
                    }
                }
            }

            m_LastSegment = best;
            return Mathf.Lerp(m_ArcLength[best], m_ArcLength[best + 1], bestT);
        }

        float DistanceToSegment(int i, Vector2 p, out float t)
        {
            Vector2 a = m_Points[i], b = m_Points[i + 1];
            Vector2 ab = b - a;
            float lengthSq = ab.sqrMagnitude;
            t = lengthSq < 1e-12f ? 0f : Mathf.Clamp01(Vector2.Dot(p - a, ab) / lengthSq);
            return Vector2.Distance(p, a + t * ab);
        }

        /// <summary>
        /// Splits the path into runs of equal (reverse, inStep), where the step is the arc-length interval
        /// [<paramref name="stepStart"/>, <paramref name="stepStart"/> + <paramref name="stepLength"/>]. The step's ends
        /// are interpolated exactly. Uses <paramref name="pool"/> for the Run objects; returns the number of runs.
        /// </summary>
        public int BuildRuns(float stepStart, float stepLength, List<Run> pool)
        {
            if (m_Count < 2) return 0;
            float stepEnd = stepStart + Mathf.Max(0f, stepLength);
            int runCount = 0;
            Run run = null;

            for (int i = 0; i + 1 < m_Count; i++)
            {
                float s0 = m_ArcLength[i], s1 = m_ArcLength[i + 1];
                bool reverse = m_Reverse[i];

                // Cut the segment at the step boundaries that fall inside it.
                float a = s0;
                while (true)
                {
                    float cut = s1;
                    if (stepStart > a && stepStart < cut) cut = stepStart;
                    if (stepEnd > a && stepEnd < cut) cut = stepEnd;
                    float mid = 0.5f * (a + cut);
                    bool inStep = mid >= stepStart && mid <= stepEnd && stepLength > 0f;

                    if (run == null || run.reverse != reverse || run.inStep != inStep)
                    {
                        // Start a new run at the current point, so neighbouring runs share their end points.
                        run = NextRun(pool, runCount++);
                        run.reverse = reverse;
                        run.inStep = inStep;
                        Add(run, i, a, s0, s1);
                    }
                    Add(run, i, cut, s0, s1);
                    if (cut >= s1) break;
                    a = cut;
                }
            }
            return runCount;
        }

        static Run NextRun(List<Run> pool, int index)
        {
            if (index == pool.Count) pool.Add(new Run());
            var run = pool[index];
            run.points.Clear();
            run.heights.Clear();
            return run;
        }

        void Add(Run run, int segment, float s, float s0, float s1)
        {
            float t = s1 - s0 < 1e-9f ? 0f : (s - s0) / (s1 - s0);
            run.points.Add(Vector2.Lerp(m_Points[segment], m_Points[segment + 1], t));
            run.heights.Add(Mathf.Lerp(m_Heights[segment], m_Heights[segment + 1], t));
        }
    }
}

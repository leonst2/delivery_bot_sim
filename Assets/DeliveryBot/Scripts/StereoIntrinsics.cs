using System;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using UnityEngine;

namespace DeliveryBot
{
    /// <summary>
    /// Pinhole intrinsics of a simulated (distortion-free) Unity camera, in the ROS/OpenCV convention: pixel centres
    /// at integer coordinates, u to the right, v down, points in the optical frame (x right, y down, z forward).
    /// </summary>
    public readonly struct StereoIntrinsics
    {
        public readonly int Width;
        public readonly int Height;
        public readonly double Fx;
        public readonly double Fy;
        public readonly double Cx;
        public readonly double Cy;

        /// <summary>Tx of the projection matrix: 0 for the left camera, -fx * baseline for the right one.</summary>
        public readonly double Tx;

        public StereoIntrinsics(int width, int height, float verticalFovDeg, double baseline = 0.0)
        {
            Width = width;
            Height = height;
            // A Unity camera without physical properties has square pixels and a symmetric frustum whose vertical
            // extent is set by fieldOfView. The image edges are at -0.5 and size - 0.5 in pixel-centre coordinates.
            Fy = 0.5 * height / Math.Tan(0.5 * verticalFovDeg * Math.PI / 180.0);
            Fx = Fy;
            Cx = 0.5 * (width - 1);
            Cy = 0.5 * (height - 1);
            Tx = baseline == 0.0 ? 0.0 : -Fx * baseline; // avoid -0 for the left camera
        }

        /// <summary>Pixel of a point given in the camera's own optical frame (metres).</summary>
        public Vector2 Project(Vector3 opticalPoint)
        {
            double z = opticalPoint.z;
            return new Vector2((float)(Fx * opticalPoint.x / z + Cx), (float)(Fy * opticalPoint.y / z + Cy));
        }

        /// <summary>Optical-frame point of a Unity camera-local point (x right, y up, z forward).</summary>
        public static Vector3 UnityLocalToOptical(Vector3 local) => new Vector3(local.x, -local.y, local.z);

        public CameraInfoMsg ToCameraInfo(HeaderMsg header)
        {
            return new CameraInfoMsg
            {
                header = header,
                height = (uint)Height,
                width = (uint)Width,
                distortion_model = "plumb_bob",
                D = new double[5],
                K = new[] { Fx, 0.0, Cx, 0.0, Fy, Cy, 0.0, 0.0, 1.0 },
                R = new[] { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },
                P = new[] { Fx, 0.0, Cx, Tx, 0.0, Fy, Cy, 0.0, 0.0, 0.0, 1.0, 0.0 },
                binning_x = 0,
                binning_y = 0,
                roi = new RegionOfInterestMsg(),
            };
        }
    }
}

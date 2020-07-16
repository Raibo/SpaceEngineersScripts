using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Text;
using System;
using VRage.Collections;
using VRage.Game.Components;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRage.Game;
using VRageMath;

namespace IngameScript
{
    public class VectorUtility
    {
        private const double LengthPrecision = 0.000000001d;

        public static double Angle(Vector3D vector1, Vector3D vector2)
        {
            var length1 = vector1.Length();
            var length2 = vector2.Length();
            return length1 < LengthPrecision || length2 < LengthPrecision
                ? 0
                : Math.Acos(Vector3D.Dot(vector1, vector2) / (vector1.Length() * vector2.Length()));
        }

        public static double Angle(Vector2D vector1, Vector2D vector2)
        {
            var length1 = vector1.Length();
            var length2 = vector2.Length();
            return length1 < LengthPrecision || length2 < LengthPrecision
                ? 0
                : Math.Acos(Dot(vector1, vector2) / (vector1.Length() * vector2.Length()));
        }

        public static Vector3D Normalize(Vector3D vector) =>
            vector == Vector3D.Zero ? Vector3D.Zero : vector / vector.Length();

        public static Vector2D Normalize(Vector2D vector) =>
            vector == Vector2D.Zero ? Vector2D.Zero : vector / vector.Length();

        public static string StringVect(Vector3D vector) => $"X:{vector.X:0.###} Y:{vector.Y:0.###} Z:{vector.Z:0.###}";
        public static string StringVect(Vector2D vector) => $"X:{vector.X:0.###} Y:{vector.Y:0.###}";

        public static Vector3D Rotate(Vector3D target, Vector3D axis, float angle)
        {
            var matrix = Matrix.CreateFromAxisAngle(axis, angle);
            return Vector3D.Transform(target, matrix);
        }

        public static double Dot(Vector2D a, Vector2D b) => a.X * b.X + a.Y * b.Y;
        public static double Cross(Vector2D a, Vector2D b) => a.X * b.Y - a.Y * b.X;
    }

}

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

        // Get a local vector in anchor's local coordinate system from a world point
        public static Vector3D GetLocalVector(MatrixD anchorMatrix, Vector3D anchorPosition, Vector3D point)
        {
            var worldVector = point - anchorPosition;
            return Vector3D.Rotate(worldVector, MatrixD.Transpose(anchorMatrix));
        }

        // Get a world point from a local point in anchor's local coordinate system
        public static Vector3D GetWorldPoint(MatrixD anchorMatrix, Vector3D anchorPosition, Vector3D point)
        {
            var worldVector = Vector3D.Rotate(point, anchorMatrix);
            return anchorPosition + worldVector;
        }

        public static Vector2D FindClosestPointOnSegment(Vector2D anchor, Vector2D a, Vector2D b)
        {
            var point = FindClosestPointOnSegment(
                new Vector3D(anchor.X, anchor.Y, 0d), new Vector3D(a.X, a.Y, 0d), new Vector3D(b.X, b.Y, 0d));

            return new Vector2D(point.X, point.Y);
        }

        // on a segment ab, finds a closest point to anchor point
        public static Vector3D FindClosestPointOnSegment(Vector3D anchor, Vector3D a, Vector3D b)
        {
            var o = anchor;
            var distanceA = Vector3D.Distance(o, a);
            var distanceB = Vector3D.Distance(o, b);

            Vector3D c; // closest point
            Vector3D f; // further point
            double oc;

            if (distanceA < distanceB)
            {
                oc = distanceA;
                c = a;
                f = b;
            }
            else
            {
                oc = distanceB;
                c = b;
                f = a;
            }

            // angle (farPoint - closePoint - o)
            var closeAngle = Angle(f - c, o - c);
            if (closeAngle >= Math.PI / 2)
                return c;

            // h stands for answer, it is height point of the triangle, drawn from o onto fc
            var hc = Math.Cos(closeAngle) * oc;
            var fc = Vector3D.Distance(f, c);
            var hcPart = hc / fc;

            var h = (f - c) * hcPart + c;
            return h;
        }

        public static Vector3D GetAverageVector(Vector3D a, Vector3D b) => new Vector3D(a.X + b.X, a.Y + b.Y, a.Z + b.Z) / 2;

        public static string ToLabel(Vector3D point, string label = "x") => $"GPS:{label}:{point.X:0.###}:{point.Y:0.###}:{point.Z:0.###}:";
    }
}

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
    class CircleIntersectFinder
    {
        public static List<Vector2D> Find(Vector2D o1, float r1, Vector2D o2, float r2)
        {
            var d = Sqrt(Sq(o1.X - o2.X) + Sq(o1.Y - o2.Y));
            var a = (Sq(r1) - Sq(r2) + Sq(d)) / (2 * d);
            var b = d - a;
            var h = Sqrt(Sq(r1) - Sq(a));
            var p2 = o1 + (o2 - o1) * (a / d);

            var x1 = p2.X + (o2.Y - o1.Y) * (h / d);
            var x2 = p2.X - (o2.Y - o1.Y) * (h / d);

            var y1 = p2.Y - (o2.X - o1.X) * (h / d);
            var y2 = p2.Y + (o2.X - o1.X) * (h / d);

            return new List<Vector2D> { new Vector2D { X = x1, Y = y1 }, new Vector2D { X = x2, Y = y2 } };
        }

        static double Sq(double a) => a * a;
        static double Sqrt(double a) => Math.Sqrt(a);
    }
}

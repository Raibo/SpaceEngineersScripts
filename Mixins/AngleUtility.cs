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
    public class AngleUtility
    {
        public const float PI = (float) Math.PI;
        public static double RadToDegree(double angle) => angle / Math.PI * 180;
        public static double DegreeToRad(double angle) => angle * Math.PI / 180;

        public static double NormalizeAngle(double angle)
        {
            if (angle > PI)
                angle -= 2 * PI;
            if (angle < PI)
                angle += 2 * PI;

            return angle;
        }
        public static double MinimizeAngle(double angle)
        {
            if (angle > PI)
                angle -= 2 * PI;
            if (angle < -PI)
                angle += 2 * PI;

            return angle;
        }
        public static double Positive(double angle)
        {
            if (angle > 2 * PI)
                angle -= 2 * PI;
            if (angle < 0)
                angle += 2 * PI;

            return angle;
        }
    }

}

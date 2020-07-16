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
    class HandyRotor
    {
        Program p;
        float PI = (float)Math.PI;
        float precision = 0.001f;
        const float TicksInInterval = 10;
        const float TicksInSecond = 60;
        const float IntervalsInSecond = TicksInSecond / TicksInInterval;

        public IMyMotorStator Rotor;
        public float Angle => Rotor.Angle;
        public readonly Vector3D Local0MarkVector = new Vector3D(0d, 0d, 1d);
        public readonly Vector3D Local90MarkVector = new Vector3D(-1d, 0d, 0d);
        public readonly Vector3D Local270MarkVector = new Vector3D(1d, 0d, 0d);

        public HandyRotor(IMyMotorStator rotor)
        {
            this.Rotor = rotor;
        }

        public void MoveTowardsAngle(double angle, double radsPerSecond) => MoveTowardsAngle((float) angle, (float) radsPerSecond);

        public void MoveTowardsAngle(float angle, float radsPerSecond)
        {
            var curAngle = Rotor.Angle;
            var diff = angle - curAngle;

            if (diff > PI)
                diff -= 2 * PI;

            if (diff < -PI)
                diff += 2 * PI;

            if (Math.Abs(diff) <= precision)
            {
                Stop();
                return;
            }

            if (diff > 0)
                radsPerSecond = Math.Abs(radsPerSecond);
            else
                radsPerSecond = -Math.Abs(radsPerSecond);

            var velocityToReachInOneInterval = diff * IntervalsInSecond;
            Rotor.TargetVelocityRad = Math.Abs(radsPerSecond) < Math.Abs(velocityToReachInOneInterval)
                ? radsPerSecond
                : velocityToReachInOneInterval;
        }

        public void Stop() => Rotor.TargetVelocityRad = 0f;
    }
}

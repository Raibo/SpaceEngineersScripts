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
        const float SecondsInInterval = TicksInInterval / TicksInSecond;

        public IMyMotorStator Rotor;
        public readonly Vector3D Local0MarkVector = new Vector3D(0d, 0d, 1d);
        public readonly Vector3D Local90MarkVector = new Vector3D(-1d, 0d, 0d);
        public readonly Vector3D Local270MarkVector = new Vector3D(1d, 0d, 0d);

        public float OffsetAngle;
        public bool ChangedDirection;

        public HandyRotor(IMyMotorStator rotor, double offsetAngle, bool changedDirection = false) :
            this(rotor, (float)offsetAngle, changedDirection)
        { }

        public HandyRotor(IMyMotorStator rotor, float offsetAngle = 0f, bool changedDirection = false)
        {
            Rotor = rotor;
            OffsetAngle = offsetAngle;
            ChangedDirection = changedDirection;
        }

        public float Angle
        {
            get
            {
                var unappliedOffset = Rotor.Angle - OffsetAngle;
                var unappliedDirection = ChangedDirection ? -unappliedOffset : unappliedOffset;
                return AngleUtility.Positive(unappliedDirection);
            }
        }

        public void MoveTowardsAngle(double angle, double radsPerSecond) => MoveTowardsAngle((float) angle, (float) radsPerSecond);

        public void MoveTowardsAngle(float angle, float radsPerSecond)
        {
            if (float.IsNaN(radsPerSecond) || float.IsNaN(angle))
            {
                Stop();
                return;
            }

            // applying mirrored if needed
            var angleMaybeChangedDirection = ChangedDirection ? -angle : angle;
            // applying offset
            var angleWithOffset = angleMaybeChangedDirection + OffsetAngle;
            // angle could end up negative or greater than 2*PI
            var finalAngle = AngleUtility.Positive(angleWithOffset);

            var curAngle = Rotor.Angle;
            var diff = finalAngle - curAngle;

            // bringing angle diff value to [-PI to +PI] segment
            diff = AngleUtility.MinimizeAngle(diff);

            if (Math.Abs(diff) <= precision)
            {
                Stop();
                return;
            }

            // calculating the direction of rotation
            if (diff > 0)
                radsPerSecond = Math.Abs(radsPerSecond);
            else
                radsPerSecond = -Math.Abs(radsPerSecond);

            // if radsPerSecond is too fast and by the end of the interval rotor will overlap, we need to slow down
            var radsPerSecondToReachInOneInterval = diff / SecondsInInterval;

            Rotor.TargetVelocityRad = Math.Abs(radsPerSecond) < Math.Abs(radsPerSecondToReachInOneInterval)
                ? radsPerSecond
                : radsPerSecondToReachInOneInterval;
        }

        public void Stop() => Rotor.TargetVelocityRad = 0f;
    }
}

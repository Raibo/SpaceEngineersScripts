using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    class Crawler
    {
        private RoboticArm LeftBack;
        private RoboticArm LeftFront;
        private RoboticArm RightBack;
        private RoboticArm RightFront;

        private IMyRemoteControl Rc;
        private Vector3D Center;

        private const double StepIndentMultiplier = 0.8d;
        private double StepIndent;

        private Vector3D LbMarchPoint;
        private Vector3D LfMarchPoint;
        private Vector3D RbMarchPoint;
        private Vector3D RfMarchPoint;

        // home
        private Vector3D Lbh;
        private Vector3D Lfh;
        private Vector3D Rbh;
        private Vector3D Rfh;

        // step
        private Vector3D Lbs;
        private Vector3D Lfs;
        private Vector3D Rbs;
        private Vector3D Rfs;

        private int State = -1;

        public Crawler(RoboticArm leftBack, RoboticArm leftFront, RoboticArm rightBack, RoboticArm rightFront, IMyRemoteControl rc)
        {
            LeftBack = leftBack;
            LeftFront = leftFront;
            RightBack = rightBack;
            RightFront = rightFront;
            Rc = rc;

            StepIndent = new []
            {
                GetAverageArmSegmentLength(LeftBack),
                GetAverageArmSegmentLength(LeftFront),
                GetAverageArmSegmentLength(RightBack),
                GetAverageArmSegmentLength(RightFront),
            }.Average() * StepIndentMultiplier;

            // getting local positions of leg base rotation rotors
            var lb = VectorUtility.GetLocalVector(Rc.WorldMatrix, Rc.GetPosition(), LeftBack.RotorRotation.Rotor.GetPosition());
            var lf = VectorUtility.GetLocalVector(Rc.WorldMatrix, Rc.GetPosition(), LeftFront.RotorRotation.Rotor.GetPosition());
            var rb = VectorUtility.GetLocalVector(Rc.WorldMatrix, Rc.GetPosition(), RightBack.RotorRotation.Rotor.GetPosition());
            var rf = VectorUtility.GetLocalVector(Rc.WorldMatrix, Rc.GetPosition(), RightFront.RotorRotation.Rotor.GetPosition());

            Center = VectorUtility.GetAverageVector(
                VectorUtility.GetAverageVector(lb, lf),
                VectorUtility.GetAverageVector(rb, rf));

            lb -= Center;
            lf -= Center;
            rb -= Center;
            rf -= Center;

            LbMarchPoint = VectorUtility.Normalize(lb) * (StepIndent + lb.Length());
            LfMarchPoint = VectorUtility.Normalize(lf) * (StepIndent + lf.Length());
            RbMarchPoint = VectorUtility.Normalize(rb) * (StepIndent + rb.Length());
            RfMarchPoint = VectorUtility.Normalize(rf) * (StepIndent + rf.Length());

            Lbh = new Vector3D(LbMarchPoint.X, LbMarchPoint.Y - 0.5, LbMarchPoint.Z);
            Lfh = new Vector3D(LfMarchPoint.X, LfMarchPoint.Y - 0.5, LfMarchPoint.Z);
            Rbh = new Vector3D(RbMarchPoint.X, RbMarchPoint.Y - 0.5, RbMarchPoint.Z);
            Rfh = new Vector3D(RfMarchPoint.X, RfMarchPoint.Y - 0.5, RfMarchPoint.Z);

            Lbs = new Vector3D(LbMarchPoint.X, LbMarchPoint.Y - 1.5, LbMarchPoint.Z + 3);
            Lfs = new Vector3D(LfMarchPoint.X, LfMarchPoint.Y - 1.5, LfMarchPoint.Z - 3);
            Rbs = new Vector3D(RbMarchPoint.X, RbMarchPoint.Y - 1.5, RbMarchPoint.Z + 3);
            Rfs = new Vector3D(RfMarchPoint.X, RfMarchPoint.Y - 1.5, RfMarchPoint.Z - 3);
        }

        public void KeepMoving()
        {
            var lbd = State == 1 ? Lbs : Lbh;
            var lfd = State == 1 ? Lfh : Lfs;
            var rbd = State == 1 ? Rbh : Rbs;
            var rfd = State == 1 ? Rfs : Rfh;

            lbd = VectorUtility.GetWorldPoint(Rc.WorldMatrix, Rc.GetPosition(), lbd + Center);
            lfd = VectorUtility.GetWorldPoint(Rc.WorldMatrix, Rc.GetPosition(), lfd + Center);
            rbd = VectorUtility.GetWorldPoint(Rc.WorldMatrix, Rc.GetPosition(), rbd + Center);
            rfd = VectorUtility.GetWorldPoint(Rc.WorldMatrix, Rc.GetPosition(), rfd + Center);

            if (NearDest(LeftBack, lbd) && NearDest(LeftFront, lfd) && NearDest(RightBack, rbd) && NearDest(RightFront, rfd))
                FlipState();

            LeftBack.KeepMoving(lbd, 2);
            LeftFront.KeepMoving(lfd, 2);
            RightBack.KeepMoving(rbd, 2);
            RightFront.KeepMoving(rfd, 2);

            var pts = new[]
            {
                lbd, lfd, rbd, rfd, LeftBack.Tip.GetPosition(), LeftFront.Tip.GetPosition(),
                RightBack.Tip.GetPosition(), RightFront.Tip.GetPosition()
            };

            var str = "";
            int i = 1;

            foreach (var p in pts)
            {

                str += VectorUtility.ToLabel(p, i.ToString()) + "\n";
                i++;
            }

            Rc.CustomData = str;
        }

        double GetAverageArmSegmentLength(RoboticArm arm)
        {
            var segment1Length = Vector3D.Distance(arm.Rotor1.Rotor.GetPosition(), arm.Rotor2.Rotor.GetPosition());
            var segment2Length = Vector3D.Distance(arm.Rotor2.Rotor.GetPosition(), arm.Tip.GetPosition());
            return (segment1Length + segment2Length) / 2;
        }

        bool NearDest(RoboticArm arm, Vector3D dest)
            => Vector3D.Distance(arm.Tip.GetPosition(), dest) < 0.3;

        void FlipState() => State *= -1;
    }
}

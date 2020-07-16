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
    public enum RotationBaseOrientationLayout
    {
        Upper,
        Lower
    }

    public enum ArmBaseOrientationLayout
    {
        Right,
        Left
    }

    class RoboticArm
    {
        private IMyTerminalBlock tip;
        private HandyRotor rotorRotation;
        private HandyRotor rotor1;
        private HandyRotor rotor2;

        private const double LowerLimit = 0.1d;

        private readonly Vector3D localArmUpVector;

        public IMyTextSurface lcd;
        public readonly RotationBaseOrientationLayout RotationBaseOrientation;
        public readonly ArmBaseOrientationLayout ArmBaseOrientation;

        public RoboticArm(List<IMyTerminalBlock> allBlocks, IMyMotorStator baseRotor, IMyTerminalBlock tip, 
            RotationBaseOrientationLayout rotationBaseOrientation = RotationBaseOrientationLayout.Upper,
            ArmBaseOrientationLayout armBaseOrientation = ArmBaseOrientationLayout.Right)
        {
            rotorRotation = new HandyRotor(baseRotor);
            this.tip = tip;
            this.RotationBaseOrientation = rotationBaseOrientation;
            this.ArmBaseOrientation = armBaseOrientation;

            // finding next 2 rotors to assign as robotic arm rotors
            var connectionBlocks = allBlocks.OfType<IMyMechanicalConnectionBlock>();
            var currentBlock = baseRotor;



            
            localArmUpVector = GetArmUpVector();
        }

        IMyMotorStator GetNextRotorBlock(List<IMyMechanicalConnectionBlock> connectionBlocks, IMyMechanicalConnectionBlock currentBlock)
        {
            var currentConnectionBlock = currentBlock;
            do
            {
                currentConnectionBlock = GetNextConnectionBlock(connectionBlocks, currentConnectionBlock);
                if (currentConnectionBlock is IMyMotorStator)
                    return currentConnectionBlock as IMyMotorStator;
            } while (currentConnectionBlock != null);
        }

        IMyMechanicalConnectionBlock GetNextConnectionBlock(List<IMyMechanicalConnectionBlock> connectionBlocks, IMyMechanicalConnectionBlock currentBlock)
        {
            var targetGrid = currentBlock.TopGrid;
            
            for(int i = 0; i < connectionBlocks.Count; i++)
                if (connectionBlocks[i].CubeGrid == targetGrid)
                    return connectionBlocks[i];

            return null;
        }

        Vector3D GetArmUpVector()
        {
            var baseUp = RotationBaseOrientation == RotationBaseOrientationLayout.Upper
                ? rotorRotation.Rotor.WorldMatrix.Up
                : rotorRotation.Rotor.WorldMatrix.Down;

            var armUp = Vector3D.Rotate(baseUp, MatrixD.Transpose(rotor1.Rotor.WorldMatrix));

            return armUp;
        }

        private double GetRotationDistance(Vector3D destonation, out Vector3D rotationEndPoint)
        {
            var anchorPoint = rotorRotation.Rotor.GetPosition();
            var localDestination = destonation - anchorPoint;
            var localTip = tip.GetPosition() - anchorPoint;

            var rotorUpVector = rotorRotation.Rotor.WorldMatrix.Up;
            var plane = new PlaneD(new Vector3D(0d, 0d, 0d), rotorUpVector);
            // Project destination point onto horizontal rotation plane
            var projectedLocalDestination = plane.ProjectPoint(ref localDestination);
            var projectedLocalTip = plane.ProjectPoint(ref localTip);
            
            var abovePlaneVector = localTip - projectedLocalTip;
            rotationEndPoint = projectedLocalDestination / projectedLocalDestination.Length() * projectedLocalTip.Length() + anchorPoint + abovePlaneVector;

            var angle = VectorUtility.Angle(projectedLocalDestination, projectedLocalTip);
            return Math.Abs(angle) * localDestination.Length();
        }

        private Vector3D PutDestinationInRange(Vector3D destination)
        {
            return destination;
        }

        public void KeepMoving(Vector3D destination, double velocity /* Meters per sec*/)
        {
            // calculate rotation angle
            destination = PutDestinationInRange(destination);
            
            Vector3D rotationEndPoint;
            var rotationDistance = GetRotationDistance(destination, out rotationEndPoint);
            var armDistance = Vector3D.Distance(destination, rotationEndPoint);

            // adjust speeds
            var rotationSpeedPart = rotationDistance / (rotationDistance + armDistance);
            var armSpeedPart = 1d - rotationSpeedPart;

            var rotationVelocity = rotationSpeedPart * velocity;
            var armVelocity = armSpeedPart * velocity;

            // calculate base rotor rotation speed
            var rotationBasePoint = rotorRotation.Rotor.GetPosition();
            var targetRps = rotationVelocity / Vector3.Distance(rotationBasePoint, destination);

            // calculate local points
            var localRotationBase = Vector3D.Zero;
            var localTip = GetLocalVector(rotorRotation.Rotor.WorldMatrix, rotationBasePoint, tip.GetPosition());
            var localDestination = GetLocalVector(rotorRotation.Rotor.WorldMatrix, rotationBasePoint, destination);

            // calculate local points
            var rotationPlane = new PlaneD(new Vector3D(0d, 0d, 0d), new Vector3D(0d, 1d, 0d)); // note that rotation plane is XZ
            var localProjectedTip = rotationPlane.ProjectPoint(ref localTip);
            var localProjectedDestination = rotationPlane.ProjectPoint(ref localDestination);

            // calculating tip and destination as 2D points for the arm
            var armTipPoint = new Vector2D(Vector3D.Distance(localRotationBase, localProjectedTip), localTip.Y);
            var armDestinationPoint = new Vector2D(Vector3D.Distance(localRotationBase, localProjectedDestination), localDestination.Y);

            // launch movement
            MakeRotationMovement(localProjectedTip, localProjectedDestination, targetRps);
            MakeArmMovement(armDestinationPoint, armVelocity);
        }

        private void MakeRotationMovement(Vector3D localProjectedTip, Vector3D localProjectedDestination, double targetRps)
        {
            var tipAngle = VectorUtility.Angle(rotorRotation.Local0MarkVector, localProjectedTip); // 0 degrees mark on rotor\
            if (VectorUtility.Angle(rotorRotation.Local90MarkVector, localProjectedTip) <
                VectorUtility.Angle(rotorRotation.Local270MarkVector, localProjectedTip))
                tipAngle = 2 * AngleUtility.PI - tipAngle;
            tipAngle = 2 * AngleUtility.PI - tipAngle; // SE rotors angles are reversed

            var diff = VectorUtility.Angle(localProjectedDestination, localProjectedTip);
            if (Vector3D.Dot(
                    Vector3D.Cross(localProjectedTip, localProjectedDestination),
                    new Vector3D(0d, 1d, 0d)
                    ) > 0) // Tip is to the left
                diff = -diff;

            var destinationAngle = AngleUtility.Positive(rotorRotation.Angle + diff);

            rotorRotation.MoveTowardsAngle(destinationAngle, targetRps);
        }

        private void MakeArmMovement(Vector2D newLocalArmTip,  double armVelocity)
        {
            // calculating points
            var armBasePoint = rotor1.Rotor.GetPosition();
            var armElbowPoint = rotor2.Rotor.GetPosition();
            var armTipPoint = tip.GetPosition();

            // calculating local points
            var localArmBase = new Vector2D(0, 0);
            
            var localArmElbow3D = GetLocalVector(rotor1.Rotor.WorldMatrix, armBasePoint, armElbowPoint);
            var localArmElbow = new Vector2D(localArmElbow3D.X, localArmElbow3D.Y);

            var localArmTip3D = GetLocalVector(rotor1.Rotor.WorldMatrix, armBasePoint, armTipPoint);
            var localArmTip = new Vector2D(localArmTip3D.X, localArmTip3D.Y);

            // calculating arm segments lengths
            var segment1Length = localArmElbow.Length(); // from base to elbow
            var segment2Length = (localArmTip - localArmElbow).Length(); // from elbow to tip
            var averageSegmentLength = (segment1Length + segment2Length) / 2f;

            // avoiding tip getting too close to center of rotation
            if (newLocalArmTip.Length() < LowerLimit * averageSegmentLength)
                newLocalArmTip *= averageSegmentLength / newLocalArmTip.Length();

            // avoiding out of range movement
            if (newLocalArmTip.Length() > segment1Length + segment2Length)
            {
                rotor1.Stop();
                rotor2.Stop();
                return;
            }

            // calculating new elbow point
            var points = CircleIntersectFinder.Find(localArmBase, (float)segment1Length, newLocalArmTip, (float)segment2Length);
            var newLocalArmElbow = points[1]; // TODO determine out of directions

            // calculating new rotor angles
            var rotor1NewAngle = (float)VectorUtility.Angle(Vector2D.UnitY, newLocalArmElbow); // TODO allow to orient arm rotors freely when build
            if (newLocalArmElbow.X < 0) rotor1NewAngle = 2 * (float)Math.PI - rotor1NewAngle;

            var rotor2NewAngle = VectorUtility.Cross(newLocalArmTip - newLocalArmElbow, newLocalArmElbow) < 0
                ? (float)(Math.PI - VectorUtility.Angle(-newLocalArmElbow, newLocalArmTip - newLocalArmElbow))
                : (float)(Math.PI + VectorUtility.Angle(-newLocalArmElbow, newLocalArmTip - newLocalArmElbow));

            rotor1.MoveTowardsAngle(rotor1NewAngle, armVelocity);
            rotor2.MoveTowardsAngle(rotor2NewAngle, armVelocity);
        }
        
        // Get point in local system of coordinates of an object (anchor). 
        Vector3D GetLocalVector(MatrixD anchorMatrix, Vector3D anchorPosition, Vector3D point)
        {
            var worldPoint = point - anchorPosition;
            var localPoint = Vector3D.Rotate(worldPoint, MatrixD.Transpose(anchorMatrix));
            return new Vector3D(localPoint.X, localPoint.Y, localPoint.Z);
        }
        
    }
}

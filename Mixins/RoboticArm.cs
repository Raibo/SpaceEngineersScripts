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
    public enum VerticalOrientation
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
        public IMyTextSurface lcd;
        public readonly VerticalOrientation VerticalOrientation;
        public readonly ArmBaseOrientationLayout ArmBaseOrientation;

        public RoboticArm(List<IMyTerminalBlock> allBlocks, IMyMotorStator baseRotor, IMyTerminalBlock tip, 
            VerticalOrientation verticalOrientation = VerticalOrientation.Upper,
            ArmBaseOrientationLayout armBaseOrientation = ArmBaseOrientationLayout.Right)
        {
            rotorRotation = new HandyRotor(baseRotor);
            this.tip = tip;
            VerticalOrientation = verticalOrientation;
            ArmBaseOrientation = armBaseOrientation;

            // finding next 2 rotors to assign as robotic arm rotors
            var connectionBlocks = allBlocks.OfType<IMyMechanicalConnectionBlock>().ToList();
            var currentBlock = baseRotor;

            currentBlock = GetNextRotorBlock(connectionBlocks, currentBlock);
            rotor1 = new HandyRotor(currentBlock);

            currentBlock = GetNextRotorBlock(connectionBlocks, currentBlock);
            rotor2 = new HandyRotor(currentBlock);
            
            localArmUpVector = GetArmUpVector();
        }

        private IMyMotorStator GetNextRotorBlock(List<IMyMechanicalConnectionBlock> connectionBlocks, IMyMechanicalConnectionBlock currentBlock)
        {
            var currentConnectionBlock = currentBlock;

            do
            {
                currentConnectionBlock = GetNextConnectionBlock(connectionBlocks, currentConnectionBlock);
                if (currentConnectionBlock is IMyMotorStator)
                    return currentConnectionBlock as IMyMotorStator;
            } while (currentConnectionBlock != null);

            return null;
        }

        private IMyMechanicalConnectionBlock GetNextConnectionBlock(List<IMyMechanicalConnectionBlock> connectionBlocks, IMyMechanicalConnectionBlock currentBlock)
        {
            var targetGrid = currentBlock.TopGrid;
            
            for(int i = 0; i < connectionBlocks.Count; i++)
                if (connectionBlocks[i].CubeGrid == targetGrid)
                    return connectionBlocks[i];

            return null;
        }

        public void KeepMoving(Vector3D destination, double velocity /* Meters per sec*/)
        {

            var matrix = rotorRotation.Rotor.WorldMatrix;

            // getting world points
            var rotationBasePoint = rotorRotation.Rotor.GetPosition();
            var armBasePoint = rotor1.Rotor.GetPosition();
            var armElbowPoint = rotor2.Rotor.GetPosition();
            var armTipPoint = tip.GetPosition();

            // adjust destination in case its something wrong with it
            //destination = CutTrailAtClosestToBase(rotationBasePoint, armTipPoint, destination);
            destination = BringToSafeZone(destination, armBasePoint, armElbowPoint, armTipPoint);
            destination = TakeNearbyPoint(destination, armTipPoint, velocity);

            // no point doing anything if the tip is already there
            if (Vector3D.Distance(armTipPoint, destination) < DistanceToCancelMovement)
            {
                rotorRotation.Stop();
                rotor1.Stop();
                rotor2.Stop();
                return;
            }

            var rotationEndPoint = GetRotationEndPoint(destination, rotationBasePoint, armTipPoint);

            // calculate local points
            var localRotationBase = Vector3D.Zero;
            var localArmBasePoint = VectorUtility.GetLocalVector(matrix, rotationBasePoint, armBasePoint);
            var localArmTipPoint = VectorUtility.GetLocalVector(matrix, rotationBasePoint, armTipPoint);
            var localDestination = VectorUtility.GetLocalVector(matrix, rotationBasePoint, destination);
            var localRotationEndPoint = VectorUtility.GetLocalVector(matrix, rotationBasePoint, rotationEndPoint);

            // calculating planes to project on
            var rotationPlane = new PlaneD(new Vector3D(0d, 0d, 0d), new Vector3D(0d, 1d, 0d)); // note that rotation plane is XZ
            var nonRotationPlane = new PlaneD(new Vector3D(0d, 0d, 0d), new Vector3D(1d, 0d, 0d)); // could be anything perpendicular to rotation plane

            // calculate local projected points
            var localProjectedTip = rotationPlane.ProjectPoint(ref localArmTipPoint);
            var localProjectedDestination = rotationPlane.ProjectPoint(ref localDestination);
            var localProjectedArmBase = nonRotationPlane.ProjectPoint(ref localArmBasePoint);
            
            // calculate rotation distance
            var rotationDistance = Math.Abs(VectorUtility.Angle(localProjectedTip, localProjectedDestination)) * localProjectedTip.Length();

            // adjust speeds
            //var rotationSpeedPart = GetRotationSpeedPart(localDestination, localArmTipPoint, localRotationBase, armDistance);
            var armDistance = Vector3D.Distance(localDestination, localRotationEndPoint);
            var rotationSpeedPart = rotationDistance / (rotationDistance + armDistance);
            var armSpeedPart = 1d - rotationSpeedPart;

            var rotationVelocity = rotationSpeedPart * velocity;
            var armVelocity = armSpeedPart * velocity;

            // calculate base rotor rotation speed
            var targetRps = rotationVelocity / localProjectedTip.Length();

            // calculating tip and destination as 2D points for the arm // TODO: if arm base is too far to the side, this must be adjusted, probably it is, in rotation method
            var armBaseElevation = localProjectedArmBase.Length(); // TODO: adjust for upside down variant

            var armTipPoint2D = new Vector2D(Vector3D.Distance(localRotationBase, localProjectedTip), localArmTipPoint.Y - armBaseElevation);
            var armDestinationPoint2D = new Vector2D(Vector3D.Distance(localRotationBase, localProjectedDestination), localDestination.Y - armBaseElevation);

            // launch movement
            MakeRotationMovement(localProjectedTip, localProjectedDestination, targetRps);
            MakeArmMovement(armDestinationPoint2D, armVelocity);
        }

        private void MakeRotationMovement(Vector3D localProjectedTip, Vector3D localProjectedDestination, double targetRps)
        {
            // calculating angle diff
            var diff = VectorUtility.Angle(localProjectedDestination, localProjectedTip);

            // calculating the direction to which to rotate.
            var movementCrossVector = Vector3D.Cross(localProjectedTip, localProjectedDestination);
            var isMovingClockwise = Vector3D.Dot(movementCrossVector, new Vector3D(0d, -1d, 0d)) > 0;

            if (!isMovingClockwise) // in SE rotors have their angle increased in clockwise movement when their head pointed upwards (for some reason)
                diff = -diff;

            // applying new angle to the rotor
            var destinationAngle = AngleUtility.Positive(rotorRotation.Angle + diff);
            rotorRotation.MoveTowardsAngle(destinationAngle, targetRps);
        }

        private void MakeArmMovement(Vector2D newLocalArmTip, double armVelocity)
        {
            // calculating points
            var armBasePoint = rotor1.Rotor.GetPosition();
            var armElbowPoint = rotor2.Rotor.GetPosition();
            var armTipPoint = tip.GetPosition();

            // calculating local points
            var localArmBase = new Vector2D(0, 0);
            
            var localArmElbow3D = VectorUtility.GetLocalVector(rotor1.Rotor.WorldMatrix, armBasePoint, armElbowPoint);
            var localArmElbow = new Vector2D(-localArmElbow3D.X, localArmElbow3D.Z); // -X and Z are just how they are and who knows why

            var localArmTip3D = VectorUtility.GetLocalVector(rotor1.Rotor.WorldMatrix, armBasePoint, armTipPoint);
            var localArmTip = new Vector2D(-localArmTip3D.X, localArmTip3D.Z);

            // calculating arm segments lengths
            var segment1Length = localArmElbow.Length(); // from base to elbow
            var segment2Length = (localArmTip - localArmElbow).Length(); // from elbow to tip

            var adjustedVelocity = armVelocity / ((segment1Length + segment2Length) / 2);

            // calculating new elbow point | might be NaN, even with destination checks
            var points = CircleIntersectFinder.Find(localArmBase, (float)segment1Length, newLocalArmTip, (float)segment2Length);
            var newLocalArmElbow = VerticalOrientation == VerticalOrientation.Upper ? points[1] : points[0];

            // flip points around Y axis if orientation is left, default calculations made for right orientation
            if (ArmBaseOrientation == ArmBaseOrientationLayout.Left)
            {
                newLocalArmElbow = new Vector2D(-newLocalArmElbow.X, newLocalArmElbow.Y);
                newLocalArmTip = new Vector2D(-newLocalArmTip.X, newLocalArmTip.Y);
            }

            // calculating new rotor angles
            var rotor1NewAngle = (float)VectorUtility.Angle(Vector2D.UnitY, newLocalArmElbow); // TODO allow to orient arm rotors freely when build
            if (newLocalArmElbow.X < 0) rotor1NewAngle = 2 * (float)Math.PI - rotor1NewAngle;

            var rotor2NewAngle = VectorUtility.Cross(newLocalArmTip - newLocalArmElbow, newLocalArmElbow) < 0
                ? (float)(Math.PI - VectorUtility.Angle(-newLocalArmElbow, newLocalArmTip - newLocalArmElbow))
                : (float)(Math.PI + VectorUtility.Angle(-newLocalArmElbow, newLocalArmTip - newLocalArmElbow));

            // calculating speed ratio
            var diff1 = Math.Abs(AngleUtility.MinimizeAngle(rotor1NewAngle - rotor1.Angle));
            var diff2 = Math.Abs(AngleUtility.MinimizeAngle(rotor2NewAngle - rotor2.Angle));
            var rotor1SpeedPart = diff1 / (diff1 + diff2);
            var rotor2SpeedPart = 1 - rotor1SpeedPart;

            // calculating target rps
            var targetRps1 = adjustedVelocity * rotor1SpeedPart;
            var targetRps2 = adjustedVelocity * rotor2SpeedPart;

            rotor1.MoveTowardsAngle(rotor1NewAngle, targetRps1 * ArmRpsMultiplier);
            rotor2.MoveTowardsAngle(rotor2NewAngle, targetRps2 * ArmRpsMultiplier);
        }

        private Vector3D GetArmUpVector()
        {
            var baseUp = VerticalOrientation == VerticalOrientation.Upper
                ? rotorRotation.Rotor.WorldMatrix.Up
                : rotorRotation.Rotor.WorldMatrix.Down;

            var armUp = Vector3D.Rotate(baseUp, MatrixD.Transpose(rotor1.Rotor.WorldMatrix));

            return armUp;
        }

        private Vector3D GetRotationEndPoint(Vector3D destination, Vector3D rotationBasePoint, Vector3D armTipPoint)
        {
            // caution: local points here are different than in main method
            var localDestination = destination - rotationBasePoint;
            var localTip = armTipPoint - rotationBasePoint;

            var rotorUpVector = rotorRotation.Rotor.WorldMatrix.Up;
            var plane = new PlaneD(new Vector3D(0d, 0d, 0d), rotorUpVector);
            // Project destination point onto horizontal rotation plane
            var projectedLocalDestination = plane.ProjectPoint(ref localDestination);
            var projectedLocalTip = plane.ProjectPoint(ref localTip);

            var abovePlaneVector = localTip - projectedLocalTip;
            return VectorUtility.Normalize(projectedLocalDestination) * projectedLocalTip.Length() + rotationBasePoint + abovePlaneVector;
        }

        private Vector3D BringToSafeZone(Vector3D destination, Vector3D armBasePoint, Vector3D armElbowPoint, Vector3D armTipPoint)
        {
            // calculating blind radius and max reach radius
            var segment1Length = Vector3D.Distance(armBasePoint, armElbowPoint);
            var segment2Length = Vector3D.Distance(armTipPoint, armElbowPoint);
            var armBlindRadius = Math.Abs(segment1Length - segment2Length) * (InnerLimit + 1);
            var armMaxReachRadius = Math.Abs(segment1Length + segment2Length) * OuterLimit;

            // calculating just an average thing not to let segments angle too small
            var averageSegmentLength = (segment1Length + segment2Length) / 2;
            var averageSafeRadius = averageSegmentLength * InnerLimit;

            // taking most strict limitation for inner radius
            var safeInnerRadius = Math.Max(armBlindRadius, averageSafeRadius);

            // taking limitation for outer radius
            var destinationDistance = Vector3D.Distance(destination, armBasePoint);
            var safeOuterRadius = Math.Min(armMaxReachRadius, destinationDistance);

            // checking if it is OK
            if (destinationDistance >= safeInnerRadius && destinationDistance <= safeOuterRadius)
                return destination;

            // if not OK, then calculating closest point inside safe zone
            var awayVector = VectorUtility.Normalize(destination - armBasePoint);

            if (destinationDistance < safeInnerRadius)
                return armBasePoint + awayVector * safeInnerRadius;

            return armBasePoint + awayVector * safeOuterRadius;
        }

        private Vector3D CutTrailAtClosestToBase(Vector3D rotationBasePoint, Vector3D armTipPoint, Vector3D destination)
        {
            // if the trail is so that the tip approaches the base at first, and then move away, we will aim to the point when it turns away
            // so that arm will have to move in one direction, either towards the base or away from it
            // and such we can calculate everything for tip to follow straight line (kind of)

            var closestPoint = VectorUtility.FindClosestPointOnSegment(rotationBasePoint, armTipPoint, destination);
            var isCloseToBasePoint = Vector3D.Distance(armTipPoint, closestPoint) <
                                     Vector3D.Distance(destination, closestPoint) * 0.1d;
            return isCloseToBasePoint
                ? destination
                : closestPoint;
        }

        private double GetRotationSpeedPart(Vector3D localDestination, Vector3D localArmTipPoint, Vector3D localRotationBase, double armDistance)
        {
            var localArmDirectionPoint = VectorUtility.Normalize(localRotationBase - localArmTipPoint) * armDistance + localArmTipPoint;
            var pathCenter = new Vector3D(
                (localArmTipPoint.X + localDestination.X) / 2,
                (localArmTipPoint.Y + localDestination.Y) / 2,
                (localArmTipPoint.Z + localDestination.Z) / 2);

            // using completion to parallelogram
            var centerToArmVector = localArmDirectionPoint - pathCenter;
            var localRotationDirectionPoint = localArmDirectionPoint - 2 * centerToArmVector;

            var armSpeed = Vector3D.Distance(localArmTipPoint, localArmDirectionPoint);
            var rotationSpeed = Vector3D.Distance(localArmTipPoint, localRotationDirectionPoint);

            return rotationSpeed / (rotationSpeed + armSpeed);
        }

        private Vector3D TakeNearbyPoint(Vector3D destination, Vector3D armTipPoint, double velocity)
        {
            // calculating where the tip is heading
            var generalDirection = VectorUtility.Normalize(destination - armTipPoint);

            // calculating how far should we cut the trail to make destination more nearby
            var generalDistance = Vector3D.Distance(armTipPoint, destination);
            var preferredDistance = velocity * NearbyPointFromVelocityMultiplier;
            var minDistance = Math.Min(generalDistance, preferredDistance);

            return armTipPoint + generalDirection * minDistance;
        }


        private IMyTerminalBlock tip;
        private readonly HandyRotor rotorRotation;
        private readonly HandyRotor rotor1;
        private readonly HandyRotor rotor2;

        private const double InnerLimit = 0.1d;
        private const double OuterLimit = 0.95d;
        private const double NearbyPointFromVelocityMultiplier = 0.4d;
        private const double ArmRpsMultiplier = 1.0d;
        private const double DistanceToCancelMovement = 0.01d;

        private readonly Vector3D localArmUpVector;
    }
}

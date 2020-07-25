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


        public readonly IMyTerminalBlock Tip;
        public readonly HandyRotor RotorRotation;
        public readonly HandyRotor Rotor1;
        public readonly HandyRotor Rotor2;

        public RoboticArm(List<IMyTerminalBlock> allBlocks, IMyMotorStator baseRotor, IMyTerminalBlock tip, 
            VerticalOrientation verticalOrientation = VerticalOrientation.Upper,
            ArmBaseOrientationLayout armBaseOrientation = ArmBaseOrientationLayout.Right)
        {
            RotorRotation = new HandyRotor(baseRotor);
            Tip = tip;
            VerticalOrientation = verticalOrientation;
            ArmBaseOrientation = armBaseOrientation;

            // finding next 2 rotors to assign as robotic arm rotors
            var connectionBlocks = allBlocks.OfType<IMyMechanicalConnectionBlock>().ToList();
            var currentBlock = baseRotor;

            currentBlock = GetNextRotorBlock(connectionBlocks, currentBlock);
            Rotor1 = new HandyRotor(currentBlock);

            currentBlock = GetNextRotorBlock(connectionBlocks, currentBlock);
            Rotor2 = new HandyRotor(currentBlock);
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
            var matrix = RotorRotation.Rotor.WorldMatrix;

            // getting world points
            var rotationBasePoint = RotorRotation.Rotor.GetPosition();
            var armBasePoint = Rotor1.Rotor.GetPosition();
            var armElbowPoint = Rotor2.Rotor.GetPosition();
            var armTipPoint = Tip.GetPosition();

            // adjust destination in case its something wrong with it
            var averageLength = (Vector3D.Distance(armBasePoint, armElbowPoint) + Vector3D.Distance(armElbowPoint, armTipPoint)) / 2;
            destination = CutTrailAtClosestToBase(rotationBasePoint, armTipPoint, destination);
            destination = BringToSafeZone(destination, armBasePoint, armElbowPoint, armTipPoint);
            destination = TakeNearbyPoint(destination, armTipPoint, averageLength);

            // no point doing anything if the tip is already there
            if (Vector3D.Distance(armTipPoint, destination) < DistanceToCancelMovement)
            {
                RotorRotation.Stop();
                Rotor1.Stop();
                Rotor2.Stop();
                return;
            }

            var rotationEndPoint = GetRotationEndPoint(destination, rotationBasePoint, armTipPoint);

            // calculate local points
            var localArmBasePoint = VectorUtility.GetLocalVector(matrix, rotationBasePoint, armBasePoint);
            var localArmTipPoint = VectorUtility.GetLocalVector(matrix, rotationBasePoint, armTipPoint);
            var localDestination = VectorUtility.GetLocalVector(matrix, rotationBasePoint, destination);
            var localRotationEndPoint = VectorUtility.GetLocalVector(matrix, rotationBasePoint, rotationEndPoint);
            var localArmElbow = VectorUtility.GetLocalVector(matrix, rotationBasePoint, armElbowPoint);

            var localArmPlaneNormalPoint = /*localArmBasePoint + */Vector3D.Rotate(Rotor1.Rotor.WorldMatrix.Up, MatrixD.Transpose(matrix));

            // calculating planes to project on
            var rotationPlane = new PlaneD(Vector3D.Zero, Vector3D.UnitY); // note that rotation plane is XZ
            var armPlane = new PlaneD(localArmBasePoint, localArmPlaneNormalPoint);

            // normalizing arm points by projecting them to the arm plane for the arm part calculations
            var normalizedArmTip = armPlane.ProjectPoint(ref localArmTipPoint);
            var normalizedArmElbow = armPlane.ProjectPoint(ref localArmElbow);
            var normalizedDestination = armPlane.ProjectPoint(ref localDestination);

            // calculate local projected points
            var localProjectedArmBase = rotationPlane.ProjectPoint(ref localArmBasePoint);
            var localProjectedTip = rotationPlane.ProjectPoint(ref normalizedArmTip);
            var localProjectedTipForRotation = rotationPlane.ProjectPoint(ref localArmTipPoint);
            var localProjectedDestination = rotationPlane.ProjectPoint(ref localDestination); // for rotation

            // calculate rotation distance
            var rotationDistance = Math.Abs(VectorUtility.Angle(localProjectedTipForRotation, localProjectedDestination)) * localProjectedTip.Length();

            // adjust speeds
            //var rotationSpeedPart = GetRotationSpeedPart(localDestination, localArmTipPoint, localRotationBase, armDistance);
            var armDistance = Vector3D.Distance(localDestination, localRotationEndPoint);
            var rotationSpeedPart = rotationDistance / (rotationDistance + armDistance);
            var armSpeedPart = 1d - rotationSpeedPart;

            var rotationVelocity = rotationSpeedPart * velocity;
            var armVelocity = armSpeedPart * velocity;

            // calculate base rotor rotation speed
            var targetRps = rotationVelocity / localProjectedTip.Length();

            // calculating tip and destination as 2D points for the arm by rotating them parallel to YZ plane
            var armBaseElevation = localArmBasePoint.Y;

            var angle2D = (float)VectorUtility.Angle(localProjectedTip - localProjectedArmBase, Vector3D.UnitX);
            if (localProjectedTip.Z < 0) angle2D *= -1;

            var armTipR = VectorUtility.Rotate(normalizedArmTip, Vector3D.UnitY, angle2D);
            var armElbowR = VectorUtility.Rotate(normalizedArmElbow, Vector3D.UnitY, angle2D);
            var armDestinationR = VectorUtility.Rotate(normalizedDestination, Vector3D.UnitY, angle2D);

            var armTipPoint2D = new Vector2D(armTipR.X, armTipR.Y - armBaseElevation);
            var armElbow2D = new Vector2D(armElbowR.X, armElbowR.Y - armBaseElevation);
            var armDestinationPoint2D = new Vector2D(armDestinationR.X, armDestinationR.Y - armBaseElevation);

            // launch movement
            MakeRotationMovement(localProjectedTipForRotation, localProjectedDestination, targetRps);
            MakeArmMovement(armTipPoint2D, armElbow2D, armDestinationPoint2D, armVelocity);
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
            var destinationAngle = AngleUtility.Positive(RotorRotation.Angle + diff);
            RotorRotation.MoveTowardsAngle(destinationAngle, targetRps);
        }

        private void MakeArmMovement(Vector2D localArmTip, Vector2D localArmElbow, Vector2D newLocalArmTip, double armVelocity)
        {
            // calculating local points
            var localArmBase = new Vector2D(0, 0);

            // flip points around Y axis if orientation is left, default calculations made for right orientation
            if (ArmBaseOrientation == ArmBaseOrientationLayout.Left)
            {
                localArmElbow = new Vector2D(-localArmElbow.X, localArmElbow.Y);
                localArmTip = new Vector2D(-localArmTip.X, localArmTip.Y);
                newLocalArmTip = new Vector2D(-newLocalArmTip.X, newLocalArmTip.Y);
            }

            // calculating arm segments lengths
            var segment1Length = localArmElbow.Length(); // from base to elbow
            var segment2Length = (localArmTip - localArmElbow).Length(); // from elbow to tip

            var adjustedVelocity = armVelocity / ((segment1Length + segment2Length) / 2);

            // calculating new elbow point | might be NaN, even with destination checks
            var points = CircleIntersectFinder.Find(localArmBase, (float)segment1Length, newLocalArmTip, (float)segment2Length);
            var newLocalArmElbow = VerticalOrientation == VerticalOrientation.Upper && ArmBaseOrientation == ArmBaseOrientationLayout.Right
                                   || VerticalOrientation == VerticalOrientation.Lower && ArmBaseOrientation == ArmBaseOrientationLayout.Left
                ? points[1] : points[0];

            // calculate diff for angle of base rotor
            var baseAngleForCurrent = CalculateBaseArmAngle(localArmElbow);
            var baseAngleForNew = CalculateBaseArmAngle(newLocalArmElbow);
            var diff1 = AngleUtility.Positive(baseAngleForNew - baseAngleForCurrent);

            // calculate diff for angle of elbow rotor
            var elbowAngleForCurrent = CalculateElbowArmAngle(localArmElbow, localArmTip);
            var elbowAngleForNew = CalculateElbowArmAngle(newLocalArmElbow, newLocalArmTip);
            var diff2 = AngleUtility.Positive(elbowAngleForNew - elbowAngleForCurrent);

            // calculating speed ratio
            var diff1ForSpeed = Math.Abs(AngleUtility.MinimizeAngle(diff1));
            var diff2ForSpeed = Math.Abs(AngleUtility.MinimizeAngle(diff2));
            var rotor1SpeedPart = diff1ForSpeed / (diff1ForSpeed + diff2ForSpeed);
            var rotor2SpeedPart = 1 - rotor1SpeedPart;

            // calculating target rps
            var targetRps1 = adjustedVelocity * rotor1SpeedPart;
            var targetRps2 = adjustedVelocity * rotor2SpeedPart;

            Rotor1.MoveTowardsAngle(Rotor1.Angle + diff1, targetRps1 * ArmRpsMultiplier);
            Rotor2.MoveTowardsAngle(Rotor2.Angle + diff2, targetRps2 * ArmRpsMultiplier);
        }

        private float CalculateBaseArmAngle(Vector2D elbowPoint)
        {
            var angle = (float)VectorUtility.Angle(Vector2D.UnitY, elbowPoint);
            if (elbowPoint.X < 0) angle = 2 * (float)Math.PI - angle;
            return angle;
        }

        private float CalculateElbowArmAngle(Vector2D elbowPoint, Vector2D tipPoint)
        {
            return VectorUtility.Cross(tipPoint - elbowPoint, elbowPoint) < 0
                ? (float)(Math.PI - VectorUtility.Angle(-elbowPoint, tipPoint - elbowPoint))
                : (float)(Math.PI + VectorUtility.Angle(-elbowPoint, tipPoint - elbowPoint));
        }

        private Vector3D GetRotationEndPoint(Vector3D destination, Vector3D rotationBasePoint, Vector3D armTipPoint)
        {
            // caution: local points here are different than in main method
            var localDestination = destination - rotationBasePoint;
            var localTip = armTipPoint - rotationBasePoint;

            var rotorUpVector = RotorRotation.Rotor.WorldMatrix.Up;
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

        private Vector3D TakeNearbyPoint(Vector3D destination, Vector3D armTipPoint, double averageLength)
        {
            // calculating where the tip is heading
            var generalDirection = VectorUtility.Normalize(destination - armTipPoint);

            // calculating how far should we cut the trail to make destination more nearby
            var generalDistance = Vector3D.Distance(armTipPoint, destination);
            var preferredDistance = averageLength * NearbyPointFromLengthMultiplier;
            var minDistance = Math.Min(generalDistance, preferredDistance);

            return armTipPoint + generalDirection * minDistance;
        }

        private const double InnerLimit = 0.1d;
        private const double OuterLimit = 0.95d;
        private const double NearbyPointFromLengthMultiplier = 0.2d;
        private const double ArmRpsMultiplier = 1.0d;
        private const double DistanceToCancelMovement = 0.01d;
    }
}

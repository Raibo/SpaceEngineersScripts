using System;
using IngameScript;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using VRageMath;

namespace Tests
{
    [TestClass]
    public class VectorUtilityTests
    {
        [TestMethod]
        public void TestData2D_1()
        {
            var anchor = new Vector2D(1, 3);
            var a = new Vector2D(1, 0);
            var b = new Vector2D(4, 4);

            var h = VectorUtility.FindClosestPointOnSegment(anchor, a, b);

            var expectedX = 2.4;
            var expectedY = 1.9;

            Assert.IsTrue(RoughlyEquals(h.X, expectedX, 0.1));
            Assert.IsTrue(RoughlyEquals(h.Y, expectedY, 0.1));
        }


        [TestMethod]
        public void TestData2D_2()
        {
            var anchor = new Vector2D(-1, 2);
            var a = new Vector2D(1, 3);
            var b = new Vector2D(-2, -2);

            var h = VectorUtility.FindClosestPointOnSegment(anchor, a, b);

            var expectedX = 0;
            var expectedY = 1.4;

            Assert.IsTrue(RoughlyEquals(h.X, expectedX, 0.1));
            Assert.IsTrue(RoughlyEquals(h.Y, expectedY, 0.1));
        }


        [TestMethod]
        public void TestData2D_3_closest_point_is_edge()
        {
            var anchor = new Vector2D(1, 3);
            var a = new Vector2D(3, 2);
            var b = new Vector2D(5, 4);

            var h = VectorUtility.FindClosestPointOnSegment(anchor, a, b);

            var expectedX = a.X;
            var expectedY = a.Y;

            Assert.IsTrue(RoughlyEquals(h.X, expectedX, 0.01));
            Assert.IsTrue(RoughlyEquals(h.Y, expectedY, 0.01));
        }


        [TestMethod]
        public void TestData3D_1()
        {
            var anchor = new Vector3D(0, 0, 0);
            var a = new Vector3D(-1, 4, 2);
            var b = new Vector3D(2, -2, 3);

            var h = VectorUtility.FindClosestPointOnSegment(anchor, a, b);

            var expectedX = 0.63;
            var expectedY = 0.739;
            var expectedZ = 2.543;

            Assert.IsTrue(RoughlyEquals(h.X, expectedX, 0.001));
            Assert.IsTrue(RoughlyEquals(h.Y, expectedY, 0.001));
            Assert.IsTrue(RoughlyEquals(h.Z, expectedZ, 0.001));
        }

        bool RoughlyEquals(double a, double b, double epsilon) => a - b < epsilon;
    }
}

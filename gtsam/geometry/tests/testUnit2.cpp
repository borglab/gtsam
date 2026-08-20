/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testUnit2.cpp
 * @brief   Unit tests for the planar unit-sphere type.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Unit2.h>

#include <cmath>

using namespace gtsam;

constexpr double kPi = 3.14159265358979323846;

/* ************************************************************************* */
namespace construction_fixture {

// A direction constructed from an angle reports that angle back, and its vector
// is the corresponding point on the circle.
TEST(Unit2, FromAngleRoundTrip) {
  for (double theta : {-3.0, -1.0, 0.0, 0.5, 2.5}) {
    const Unit2 direction = Unit2::FromAngle(theta);
    EXPECT_DOUBLES_EQUAL(theta, direction.angle(), 1e-12);
    EXPECT(assert_equal(Point2(std::cos(theta), std::sin(theta)),
                        Point2(direction.unitVector()), 1e-12));
  }
}

// Construction from an unnormalized vector normalizes, so the invariant holds
// whatever the caller passes in.
TEST(Unit2, ConstructorNormalizes) {
  const Unit2 direction{Point2(3.0, -4.0)};
  EXPECT_DOUBLES_EQUAL(1.0, direction.unitVector().norm(), 1e-12);
  EXPECT(
      assert_equal(Point2(0.6, -0.8), Point2(direction.unitVector()), 1e-12));
}

// The manifold has one degree of freedom, one fewer than its ambient dimension.
TEST(Unit2, Dimension) {
  EXPECT_LONGS_EQUAL(1, static_cast<long>(Unit2::dimension));
  EXPECT_LONGS_EQUAL(
      1, static_cast<long>(traits<Unit2>::GetDimension(Unit2())));
}

}  // namespace construction_fixture
/* ************************************************************************* */
namespace manifold_fixture {

// retract moves along the circle by the tangent amount, so it composes as an
// angle increment and stays on the manifold.
TEST(Unit2, RetractAddsAngle) {
  const Unit2 base = Unit2::FromAngle(0.4);
  const Unit2 moved = base.retract(Vector1(0.25));
  EXPECT_DOUBLES_EQUAL(0.65, moved.angle(), 1e-12);
  EXPECT_DOUBLES_EQUAL(1.0, moved.unitVector().norm(), 1e-12);
}

// localCoordinates inverts retract, which is the property the optimizer relies
// on and the one a hand-written wrap can silently break.
TEST(Unit2, LocalCoordinatesInvertsRetract) {
  const Unit2 base = Unit2::FromAngle(-0.8);
  for (double step : {-1.2, -0.3, 0.0, 0.45, 1.1}) {
    const Unit2 moved = base.retract(Vector1(step));
    EXPECT(assert_equal(Vector1(step), base.localCoordinates(moved), 1e-12));
  }
}

// The tangent is wrapped into (-pi, pi], so directions either side of the
// branch cut are a short step apart rather than nearly a full turn.
TEST(Unit2, LocalCoordinatesWrapsAcrossBranchCut) {
  const Unit2 justBelow = Unit2::FromAngle(kPi - 0.05);
  const Unit2 justAbove = Unit2::FromAngle(-kPi + 0.05);
  const Vector1 tangent = justBelow.localCoordinates(justAbove);
  EXPECT_DOUBLES_EQUAL(0.1, tangent(0), 1e-12);
  // And retracting by it lands on the same direction, not on its reflection.
  EXPECT(assert_equal(justAbove.unitVector(),
                      justBelow.retract(tangent).unitVector(), 1e-12));
}

// A round trip through the tangent space is the identity on the manifold, which
// is what numerical differentiation of a factor in this variable depends on.
TEST(Unit2, RetractLocalCoordinatesRoundTrip) {
  for (double theta : {-2.9, -0.4, 0.0, 1.3, 3.0}) {
    const Unit2 direction = Unit2::FromAngle(theta);
    const Unit2 recovered =
        direction.retract(direction.localCoordinates(direction));
    EXPECT(assert_equal(direction.unitVector(), recovered.unitVector(), 1e-12));
  }
}

// equals compares directions, so two representations of the same one agree.
TEST(Unit2, Equals) {
  const Unit2 a = Unit2::FromAngle(0.7);
  const Unit2 b{Point2(std::cos(0.7) * 5.0, std::sin(0.7) * 5.0)};
  EXPECT(a.equals(b, 1e-9));
  EXPECT(!a.equals(Unit2::FromAngle(0.9), 1e-9));
}

}  // namespace manifold_fixture
/* ************************************************************************* */
namespace qcqp_traits_fixture {

// The D=1 homogenized lift is [1; u], the layout the QCQP uses for a variable
// carried as a vector rather than a matrix.
TEST(Unit2, QcqpValueHomogenized) {
  const Unit2 direction = Unit2::FromAngle(0.3);
  const Matrix lifted = traits<Unit2>::QcqpValue<1>(direction);
  EXPECT_LONGS_EQUAL(traits<Unit2>::QcqpVectorDim, lifted.rows());
  EXPECT_LONGS_EQUAL(1, lifted.cols());
  EXPECT_DOUBLES_EQUAL(1.0, lifted(0, 0), 1e-12);
  EXPECT(assert_equal(Point2(direction.unitVector()),
                      Point2(lifted.block(1, 0, 2, 1)), 1e-12));
}

// At staircase rank D the lift is a single row, zero-padded past the ambient
// dimension so a lifted direction has the same row count at every level.
TEST(Unit2, QcqpValueAtHigherRank) {
  const Unit2 direction = Unit2::FromAngle(-1.1);
  const Matrix lifted = traits<Unit2>::QcqpValue<4>(direction);
  EXPECT_LONGS_EQUAL(1, lifted.rows());
  EXPECT_LONGS_EQUAL(4, lifted.cols());
  EXPECT(assert_equal(Point2(direction.unitVector()),
                      Point2(lifted.block(0, 0, 1, 2).transpose()), 1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, lifted.block(0, 2, 1, 2).norm(), 1e-12);
}

// FromQcqpValue inverts QcqpValue at both D=1 and D>=2, and normalizes, so a
// row that rounding left slightly off the circle still recovers a direction.
TEST(Unit2, FromQcqpValueInvertsQcqpValue) {
  const Unit2 direction = Unit2::FromAngle(0.9);
  EXPECT(assert_equal(direction.unitVector(),
                      traits<Unit2>::FromQcqpValue<1>(
                          traits<Unit2>::QcqpValue<1>(direction)).unitVector(),
                      1e-12));
  EXPECT(assert_equal(direction.unitVector(),
                      traits<Unit2>::FromQcqpValue<3>(
                          traits<Unit2>::QcqpValue<3>(direction)).unitVector(),
                      1e-12));

  Matrix shrunk(1, 2);
  shrunk << 0.06, -0.08;  // norm 0.1, as a truncation would leave it
  EXPECT_DOUBLES_EQUAL(
      1.0, traits<Unit2>::FromQcqpValue<2>(shrunk).unitVector().norm(), 1e-12);
}

// The unit-norm constraint lives with the type rather than with the factor that
// happens to introduce the variable, and at rank D it is the single scalar
// ||u||^2 = 1.
TEST(Unit2, QcqpConstraintsAreUnitNorm) {
  const auto constraints = traits<Unit2>::QcqpConstraints<2>();
  EXPECT_LONGS_EQUAL(1, static_cast<long>(constraints.size()));
  EXPECT(assert_equal(Matrix(Matrix::Identity(1, 1)), constraints[0].first,
                      1e-12));
  EXPECT_DOUBLES_EQUAL(1.0, constraints[0].second, 1e-12);
}

// A wrongly shaped lift is rejected rather than silently reinterpreted.
TEST(Unit2, FromQcqpValueRejectsBadShapes) {
  CHECK_EXCEPTION(traits<Unit2>::FromQcqpValue<1>(Matrix::Zero(2, 1)),
                  std::invalid_argument);
  CHECK_EXCEPTION(traits<Unit2>::FromQcqpValue<3>(Matrix::Zero(2, 3)),
                  std::invalid_argument);
}

}  // namespace qcqp_traits_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

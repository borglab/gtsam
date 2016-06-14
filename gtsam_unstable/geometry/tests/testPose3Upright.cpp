/**
 * @file testPose3Upright.cpp
 *
 * @date Jan 24, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam_unstable/geometry/Pose3Upright.h>

using namespace gtsam;

static const double tol = 1e-5;

/* ************************************************************************* */
TEST( testPose3Upright, basics ) {
  Pose3Upright origin;
  EXPECT_DOUBLES_EQUAL(0.0, origin.x(), tol);
  EXPECT_DOUBLES_EQUAL(0.0, origin.y(), tol);
  EXPECT_DOUBLES_EQUAL(0.0, origin.z(), tol);
  EXPECT_DOUBLES_EQUAL(0.0, origin.theta(), tol);

  Pose3Upright actual1(Rot2::fromAngle(0.1), Point3(1.0, 2.0, 3.0));
  EXPECT_DOUBLES_EQUAL(1.0, actual1.x(), tol);
  EXPECT_DOUBLES_EQUAL(2.0, actual1.y(), tol);
  EXPECT_DOUBLES_EQUAL(3.0, actual1.z(), tol);
  EXPECT_DOUBLES_EQUAL(0.1, actual1.theta(), tol);

  Pose3Upright actual2(1.0, 2.0, 3.0, 0.1);
  EXPECT_DOUBLES_EQUAL(1.0, actual2.x(), tol);
  EXPECT_DOUBLES_EQUAL(2.0, actual2.y(), tol);
  EXPECT_DOUBLES_EQUAL(3.0, actual2.z(), tol);
  EXPECT_DOUBLES_EQUAL(0.1, actual2.theta(), tol);
}

/* ************************************************************************* */
TEST( testPose3Upright, equals ) {
  Pose3Upright origin, actual1(1.0, 2.0, 3.0, 0.1),
      actual2(1.0, 2.0, 3.0, 0.1), actual3(4.0,-7.0, 3.0, 0.3);
  EXPECT(assert_equal(origin, origin, tol));
  EXPECT(assert_equal(actual1, actual1, tol));
  EXPECT(assert_equal(actual1, actual2, tol));
  EXPECT(assert_equal(actual2, actual1, tol));

  EXPECT(assert_inequal(actual1, actual3, tol));
  EXPECT(assert_inequal(actual3, actual1, tol));
  EXPECT(assert_inequal(actual1, origin, tol));
  EXPECT(assert_inequal(origin, actual1, tol));
}

/* ************************************************************************* */
TEST( testPose3Upright, conversions ) {
  Pose3Upright pose(1.0, 2.0, 3.0, 0.1);
  EXPECT(assert_equal(Point3(1.0, 2.0, 3.0), pose.translation(), tol));
  EXPECT(assert_equal(Point2(1.0, 2.0), pose.translation2(), tol));
  EXPECT(assert_equal(Rot2::fromAngle(0.1), pose.rotation2(), tol));
  EXPECT(assert_equal(Rot3::Yaw(0.1), pose.rotation(), tol));
  EXPECT(assert_equal(Pose2(1.0, 2.0, 0.1), pose.pose2(), tol));
  EXPECT(assert_equal(Pose3(Rot3::Yaw(0.1), Point3(1.0, 2.0, 3.0)), pose.pose(), tol));
}

/* ************************************************************************* */
TEST( testPose3Upright, manifold ) {
  Pose3Upright origin, x1(1.0, 2.0, 3.0, 0.0), x2(4.0, 2.0, 7.0, 0.0);
  EXPECT_LONGS_EQUAL(4, origin.dim());

  EXPECT(assert_equal(origin, origin.retract(Z_4x1), tol));
  EXPECT(assert_equal(x1, x1.retract(Z_4x1), tol));
  EXPECT(assert_equal(x2, x2.retract(Z_4x1), tol));

  Vector delta12 = (Vector(4) << 3.0, 0.0, 4.0, 0.0).finished(), delta21 = -delta12;
  EXPECT(assert_equal(x2, x1.retract(delta12), tol));
  EXPECT(assert_equal(x1, x2.retract(delta21), tol));

  EXPECT(assert_equal(delta12, x1.localCoordinates(x2), tol));
  EXPECT(assert_equal(delta21, x2.localCoordinates(x1), tol));
}

/* ************************************************************************* */
TEST( testPose3Upright, lie ) {
  Pose3Upright origin, x1(1.0, 2.0, 3.0, 0.1);
  EXPECT(assert_equal(Z_4x1, Pose3Upright::Logmap(origin), tol));
  EXPECT(assert_equal(origin, Pose3Upright::Expmap(Z_4x1), tol));

  EXPECT(assert_equal(x1, Pose3Upright::Expmap(Pose3Upright::Logmap(x1)), tol));
}

/* ************************************************************************* */
Pose3Upright between_proxy(const Pose3Upright& x1, const Pose3Upright& x2) { return x1.between(x2); }
TEST( testPose3Upright, between ) {
  Pose3Upright x1(1.0, 2.0, 3.0, 0.1), x2(4.0,-2.0, 7.0, 0.3);
  Pose3Upright expected(x1.pose2().between(x2.pose2()), x2.z() - x1.z());
  EXPECT(assert_equal(expected, x1.between(x2), tol));

  Matrix actualH1, actualH2, numericH1, numericH2;
  x1.between(x2, actualH1, actualH2);
  numericH1 = numericalDerivative21(between_proxy, x1, x2, 1e-5);
  numericH2 = numericalDerivative22(between_proxy, x1, x2, 1e-5);
  EXPECT(assert_equal(numericH1, actualH1, tol));
  EXPECT(assert_equal(numericH2, actualH2, tol));
}

/* ************************************************************************* */
Pose3Upright compose_proxy(const Pose3Upright& x1, const Pose3Upright& x2) { return x1.compose(x2); }
TEST( testPose3Upright, compose ) {
  Pose3Upright x1(1.0, 2.0, 3.0, 0.1), x2(4.0,-2.0, 7.0, 0.3);
  Pose3Upright expected(x1.pose2().between(x2.pose2()), x2.z() - x1.z());
  EXPECT(assert_equal(x2, x1.compose(expected), tol));

  Matrix actualH1, actualH2, numericH1, numericH2;
  x1.compose(expected, actualH1, actualH2);
  numericH1 = numericalDerivative21(compose_proxy, x1, expected, 1e-5);
  numericH2 = numericalDerivative22(compose_proxy, x1, expected, 1e-5);
  EXPECT(assert_equal(numericH1, actualH1, tol));
  EXPECT(assert_equal(numericH2, actualH2, tol));
}

/* ************************************************************************* */
Pose3Upright inverse_proxy(const Pose3Upright& x1) { return x1.inverse(); }
TEST( testPose3Upright, inverse ) {
  Pose3Upright x1(1.0, 2.0, 3.0, 0.1);
  Pose3Upright expected(x1.pose2().inverse(), - x1.z());
  EXPECT(assert_equal(expected, x1.inverse(), tol));

  Matrix actualH1, numericH1;
  x1.inverse(actualH1);
  numericH1 = numericalDerivative11(inverse_proxy, x1, 1e-5);
  EXPECT(assert_equal(numericH1, actualH1, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

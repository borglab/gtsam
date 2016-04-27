/**
 * @file testBearingS2.cpp
 *
 * @brief Tests for a bearing measurement on S2 for 3D bearing measurements
 *
 * @date Jan 26, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/TestableAssertions.h>

#include <gtsam_unstable/geometry/BearingS2.h>

using namespace gtsam;

static const double tol=1e-5;

/* ************************************************************************* */
TEST( testBearingS2, basics ) {
  BearingS2 origin;
  EXPECT(assert_equal(Rot2(), origin.elevation(), tol));
  EXPECT(assert_equal(Rot2(), origin.azimuth(), tol));

  double expAzi = 0.2, expEle = 0.3;
  BearingS2 actual1(expAzi, expEle);
  EXPECT(assert_equal(Rot2::fromAngle(expEle), actual1.elevation(), tol));
  EXPECT(assert_equal(Rot2::fromAngle(expAzi), actual1.azimuth(), tol));
}

/* ************************************************************************* */
TEST( testBearingS2, equals ) {
  BearingS2 origin, b1(0.2, 0.3), b2(b1), b3(0.1, 0.3), b4(0.2, 0.5);
  EXPECT(assert_equal(origin, origin, tol));
  EXPECT(assert_equal(b1, b1, tol));
  EXPECT(assert_equal(b1, b2, tol));
  EXPECT(assert_equal(b2, b1, tol));

  EXPECT(assert_inequal(b1, b3, tol));
  EXPECT(assert_inequal(b3, b1, tol));
  EXPECT(assert_inequal(b1, b4, tol));
  EXPECT(assert_inequal(b4, b1, tol));
}

/* ************************************************************************* */
TEST( testBearingS2, manifold ) {
  BearingS2 origin, b1(0.2, 0.3);
  EXPECT_LONGS_EQUAL(2, origin.dim());

  EXPECT(assert_equal(Z_2x1, origin.localCoordinates(origin), tol));
  EXPECT(assert_equal(origin, origin.retract(Z_2x1), tol));

  EXPECT(assert_equal(Z_2x1, b1.localCoordinates(b1), tol));
  EXPECT(assert_equal(b1, b1.retract(Z_2x1), tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testNavState.cpp
 * @brief   Unit tests for NavState
 * @author  Frank Dellaert
 * @date    July 2015
 */

#include <gtsam/navigation/NavState.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

static const Rot3 kRotation = Rot3::RzRyRx(0.1, 0.2, 0.3);
static const Point3 kPosition(1.0, 2.0, 3.0);
static const Velocity3 kVelocity(0.4, 0.5, 0.6);
static const NavState kIdentity;
static const NavState kState1(kRotation, kPosition, kVelocity);

const double tol = 1e-5;

/* ************************************************************************* */
TEST( NavState, MatrixGroup ) {
  // check roundtrip conversion to 7*7 matrix representation
  Matrix7 T = kState1.matrix();
  EXPECT(assert_equal(kState1, NavState(T), tol));

  // check group product agrees with matrix product
  NavState state2 = kState1 * kState1;
  Matrix T2 = T * T;
  EXPECT(assert_equal(state2, NavState(T2), tol));
  EXPECT(assert_equal(T2, state2.matrix(), tol));
}

/* ************************************************************************* */
TEST( NavState, Manifold ) {
  // Check zero xi
  EXPECT(assert_equal(kIdentity, kIdentity.retract(zero(9)), tol));
  EXPECT(assert_equal(zero(9), kIdentity.localCoordinates(kIdentity), tol));
  EXPECT(assert_equal(kState1, kState1.retract(zero(9)), tol));
  EXPECT(assert_equal(zero(9), kState1.localCoordinates(kState1), tol));

  // Check definition of retract as operating on components separately
  Vector xi(9);
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  Rot3 drot = Rot3::Expmap(xi.head<3>());
  Point3 dt = Point3(xi.segment < 3 > (3));
  Velocity3 dvel = Velocity3(-0.1, -0.2, -0.3);
  NavState state2 = kState1 * NavState(drot, dt, dvel);
  EXPECT(assert_equal(state2, kState1.retract(xi), tol));
  EXPECT(assert_equal(xi, kState1.localCoordinates(state2), tol));

  // roundtrip from state2 to state3 and back
  NavState state3 = state2.retract(xi);
  EXPECT(assert_equal(xi, state2.localCoordinates(state3), tol));

  // Check derivatives for ChartAtOrigin::Retract
  Matrix9 aH, eH;
  //  For zero xi
  boost::function<NavState(Vector9)> f = boost::bind(
      NavState::ChartAtOrigin::Retract, _1, boost::none);
  NavState::ChartAtOrigin::Retract(zero(9), aH);
  eH = numericalDerivative11<NavState, Vector9>(f, zero(9));
  EXPECT(assert_equal((Matrix )eH, aH));
  //  For non-zero xi
  NavState::ChartAtOrigin::Retract(xi, aH);
  eH = numericalDerivative11<NavState, Vector9>(f, xi);
  EXPECT(assert_equal((Matrix )eH, aH));

  // Check retract derivatives
  Matrix9 aH1, aH2;
  kState1.retract(xi, aH1, aH2);
  Matrix eH1 = numericalDerivative11<NavState, NavState>(
      boost::bind(&NavState::retract, _1, xi, boost::none, boost::none),
      kState1);
  EXPECT(assert_equal(eH1, aH1));
  Matrix eH2 = numericalDerivative11<NavState, Vector9>(
      boost::bind(&NavState::retract, kState1, _1, boost::none, boost::none),
      xi);
  EXPECT(assert_equal(eH2, aH2));
}

/* ************************************************************************* */
TEST( NavState, Lie ) {
  // Check zero xi
  EXPECT(assert_equal(kIdentity, kIdentity.expmap(zero(9)), tol));
  EXPECT(assert_equal(zero(9), kIdentity.logmap(kIdentity), tol));
  EXPECT(assert_equal(kState1, kState1.expmap(zero(9)), tol));
  EXPECT(assert_equal(zero(9), kState1.logmap(kState1), tol));

  // Expmap/Logmap roundtrip
  Vector xi(9);
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  NavState state2 = NavState::Expmap(xi);
  EXPECT(assert_equal(xi, NavState::Logmap(state2), tol));

  // roundtrip from state2 to state3 and back
  NavState state3 = state2.expmap(xi);
  EXPECT(assert_equal(xi, state2.logmap(state3), tol));

  // For the expmap/logmap (not necessarily expmap/local) -xi goes other way
  EXPECT(assert_equal(state2, state3.expmap(-xi), tol));
  EXPECT(assert_equal(xi, -state3.logmap(state2), tol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

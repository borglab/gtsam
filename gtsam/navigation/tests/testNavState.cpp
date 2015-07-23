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
static const Vector3 kOmegaCoriolis(0.02, 0.03, 0.04);
static const Vector3 kGravity(0, 0, 9.81);

/* ************************************************************************* */
TEST( NavState, Attitude) {
  Matrix39 aH, eH;
  Rot3 actual = kState1.attitude(aH);
  EXPECT(assert_equal(actual, kRotation));
  eH = numericalDerivative11<Rot3, NavState>(
      boost::bind(&NavState::attitude, _1, boost::none), kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}
/* ************************************************************************* */
TEST( NavState, Position) {
  Matrix39 aH, eH;
  Point3 actual = kState1.position(aH);
  EXPECT(assert_equal(actual, kPosition));
  eH = numericalDerivative11<Point3, NavState>(
      boost::bind(&NavState::position, _1, boost::none), kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}
/* ************************************************************************* */
TEST( NavState, Velocity) {
  Matrix39 aH, eH;
  Velocity3 actual = kState1.velocity(aH);
  EXPECT(assert_equal(actual, kVelocity));
  eH = numericalDerivative11<Velocity3, NavState>(
      boost::bind(&NavState::velocity, _1, boost::none), kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}
/* ************************************************************************* */
TEST( NavState, MatrixGroup ) {
  // check roundtrip conversion to 7*7 matrix representation
  Matrix7 T = kState1.matrix();
  EXPECT(assert_equal(kState1, NavState(T)));

  // check group product agrees with matrix product
  NavState state2 = kState1 * kState1;
  Matrix T2 = T * T;
  EXPECT(assert_equal(state2, NavState(T2)));
  EXPECT(assert_equal(T2, state2.matrix()));
}

/* ************************************************************************* */
TEST( NavState, Manifold ) {
  // Check zero xi
  EXPECT(assert_equal(kIdentity, kIdentity.retract(zero(9))));
  EXPECT(assert_equal(zero(9), kIdentity.localCoordinates(kIdentity)));
  EXPECT(assert_equal(kState1, kState1.retract(zero(9))));
  EXPECT(assert_equal(zero(9), kState1.localCoordinates(kState1)));

  // Check definition of retract as operating on components separately
  Vector xi(9);
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  Rot3 drot = Rot3::Expmap(xi.head<3>());
  Point3 dt = Point3(xi.segment < 3 > (3));
  Velocity3 dvel = Velocity3(-0.1, -0.2, -0.3);
  NavState state2 = kState1 * NavState(drot, dt, dvel);
  EXPECT(assert_equal(state2, kState1.retract(xi)));
  EXPECT(assert_equal(xi, kState1.localCoordinates(state2)));

  // roundtrip from state2 to state3 and back
  NavState state3 = state2.retract(xi);
  EXPECT(assert_equal(xi, state2.localCoordinates(state3)));

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
  EXPECT(assert_equal(kIdentity, kIdentity.expmap(zero(9))));
  EXPECT(assert_equal(zero(9), kIdentity.logmap(kIdentity)));
  EXPECT(assert_equal(kState1, kState1.expmap(zero(9))));
  EXPECT(assert_equal(zero(9), kState1.logmap(kState1)));

  // Expmap/Logmap roundtrip
  Vector xi(9);
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  NavState state2 = NavState::Expmap(xi);
  EXPECT(assert_equal(xi, NavState::Logmap(state2)));

  // roundtrip from state2 to state3 and back
  NavState state3 = state2.expmap(xi);
  EXPECT(assert_equal(xi, state2.logmap(state3)));

  // For the expmap/logmap (not necessarily expmap/local) -xi goes other way
  EXPECT(assert_equal(state2, state3.expmap(-xi)));
  EXPECT(assert_equal(xi, -state3.logmap(state2)));
}

/* ************************************************************************* */
static const double dt = 2.0;
boost::function<Vector9(const NavState&, const bool&)> coriolis = boost::bind(
    &NavState::coriolis, _1, dt, kOmegaCoriolis, _2, boost::none);

TEST(NavState, Coriolis) {
  Matrix9 actualH;

  // first-order
  kState1.coriolis(dt, kOmegaCoriolis, false, actualH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, kState1, false), actualH));
  // second-order
  kState1.coriolis(dt, kOmegaCoriolis, true, actualH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, kState1, true), actualH));
}

TEST(NavState, Coriolis2) {
  Matrix9 actualH;
  NavState state2(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
      Point3(5.0, 1.0, -50.0), Vector3(0.5, 0.0, 0.0));

  // first-order
  state2.coriolis(dt, kOmegaCoriolis, false, actualH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, state2, false), actualH));
  // second-order
  state2.coriolis(dt, kOmegaCoriolis, true, actualH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, state2, true), actualH));
}

/* ************************************************************************* */
TEST(NavState, PredictXi) {
  Vector9 xi;
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  double dt = 0.5;
  Matrix9 actualH1, actualH2;
  boost::function<Vector9(const NavState&, const Vector9&)> integrateTangent =
      boost::bind(&NavState::integrateTangent, _1, _2, dt, kGravity, kOmegaCoriolis,
          false, boost::none, boost::none);
  kState1.integrateTangent(xi, dt, kGravity, kOmegaCoriolis, false, actualH1, actualH2);
  EXPECT(assert_equal(numericalDerivative21(integrateTangent, kState1, xi), actualH1));
  EXPECT(assert_equal(numericalDerivative22(integrateTangent, kState1, xi), actualH2));
}

/* ************************************************************************* */
TEST(NavState, Predict) {
  Vector9 xi;
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  double dt = 0.5;
  Matrix9 actualH1, actualH2;
  boost::function<NavState(const NavState&, const Vector9&)> predict =
      boost::bind(&NavState::predict, _1, _2, dt, kGravity, kOmegaCoriolis,
          false, boost::none, boost::none);
  kState1.predict(xi, dt, kGravity, kOmegaCoriolis, false, actualH1, actualH2);
  EXPECT(assert_equal(numericalDerivative21(predict, kState1, xi), actualH1));
  EXPECT(assert_equal(numericalDerivative22(predict, kState1, xi), actualH2));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

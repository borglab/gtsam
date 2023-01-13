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

#include <boost/bind/bind.hpp>
#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

static const Rot3 kAttitude = Rot3::RzRyRx(0.1, 0.2, 0.3);
static const Point3 kPosition(1.0, 2.0, 3.0);
static const Pose3 kPose(kAttitude, kPosition);
static const Velocity3 kVelocity(0.4, 0.5, 0.6);
static const NavState kIdentity;
static const NavState kState1(kAttitude, kPosition, kVelocity);
static const Vector3 kOmegaCoriolis(0.02, 0.03, 0.04);
static const Vector3 kGravity(0, 0, 9.81);
static const Vector9 kZeroXi = Vector9::Zero();

/* ************************************************************************* */
TEST(NavState, Constructor) {
  std::function<NavState(const Rot3&, const Point3&, const Vector3&)> create =
      std::bind(&NavState::Create, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, nullptr, nullptr, nullptr);
  Matrix aH1, aH2, aH3;
  EXPECT(
      assert_equal(kState1,
          NavState::Create(kAttitude, kPosition, kVelocity, aH1, aH2, aH3)));
  EXPECT(
      assert_equal(
          numericalDerivative31(create, kAttitude, kPosition, kVelocity), aH1));
  EXPECT(
      assert_equal(
          numericalDerivative32(create, kAttitude, kPosition, kVelocity), aH2));
  EXPECT(
      assert_equal(
          numericalDerivative32(create, kAttitude, kPosition, kVelocity), aH2));
}

/* ************************************************************************* */
TEST(NavState, Constructor2) {
  std::function<NavState(const Pose3&, const Vector3&)> construct =
      std::bind(&NavState::FromPoseVelocity, std::placeholders::_1,
                std::placeholders::_2, nullptr, nullptr);
  Matrix aH1, aH2;
  EXPECT(
      assert_equal(kState1,
          NavState::FromPoseVelocity(kPose, kVelocity, aH1, aH2)));
  EXPECT(assert_equal(numericalDerivative21(construct, kPose, kVelocity), aH1));
  EXPECT(assert_equal(numericalDerivative22(construct, kPose, kVelocity), aH2));
}

/* ************************************************************************* */
TEST( NavState, Attitude) {
  Matrix39 aH, eH;
  Rot3 actual = kState1.attitude(aH);
  EXPECT(assert_equal(actual, kAttitude));
  eH = numericalDerivative11<Rot3, NavState>(
      std::bind(&NavState::attitude, std::placeholders::_1, nullptr), kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}

/* ************************************************************************* */
TEST( NavState, Position) {
  Matrix39 aH, eH;
  Point3 actual = kState1.position(aH);
  EXPECT(assert_equal(actual, kPosition));
  eH = numericalDerivative11<Point3, NavState>(
      std::bind(&NavState::position, std::placeholders::_1, nullptr),
      kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}

/* ************************************************************************* */
TEST( NavState, Velocity) {
  Matrix39 aH, eH;
  Velocity3 actual = kState1.velocity(aH);
  EXPECT(assert_equal(actual, kVelocity));
  eH = numericalDerivative11<Velocity3, NavState>(
      std::bind(&NavState::velocity, std::placeholders::_1, nullptr),
      kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}

/* ************************************************************************* */
TEST( NavState, BodyVelocity) {
  Matrix39 aH, eH;
  Velocity3 actual = kState1.bodyVelocity(aH);
  EXPECT(assert_equal<Velocity3>(actual, kAttitude.unrotate(kVelocity)));
  eH = numericalDerivative11<Velocity3, NavState>(
      std::bind(&NavState::bodyVelocity, std::placeholders::_1, nullptr),
      kState1);
  EXPECT(assert_equal((Matrix )eH, aH));
}

/* ************************************************************************* */
TEST( NavState, Manifold ) {
  // Check zero xi
  EXPECT(assert_equal(kIdentity, kIdentity.retract(kZeroXi)));
  EXPECT(assert_equal(kZeroXi, kIdentity.localCoordinates(kIdentity)));
  EXPECT(assert_equal(kState1, kState1.retract(kZeroXi)));
  EXPECT(assert_equal(kZeroXi, kState1.localCoordinates(kState1)));

  // Check definition of retract as operating on components separately
  Vector9 xi;
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  Rot3 drot = Rot3::Expmap(xi.head<3>());
  Point3 dt = Point3(xi.segment<3>(3));
  Velocity3 dvel = Velocity3(-0.1, -0.2, -0.3);
  NavState state2 = NavState(kState1.attitude() * drot,
      kState1.position() + kState1.attitude() * dt,
      kState1.velocity() + kState1.attitude() * dvel);
  EXPECT(assert_equal(state2, kState1.retract(xi)));
  EXPECT(assert_equal(xi, kState1.localCoordinates(state2)));

  // roundtrip from state2 to state3 and back
  NavState state3 = state2.retract(xi);
  EXPECT(assert_equal(xi, state2.localCoordinates(state3)));

  // Check retract derivatives
  Matrix9 aH1, aH2;
  kState1.retract(xi, aH1, aH2);
  std::function<NavState(const NavState&, const Vector9&)> retract =
      std::bind(&NavState::retract, std::placeholders::_1,
                std::placeholders::_2, nullptr, nullptr);
  EXPECT(assert_equal(numericalDerivative21(retract, kState1, xi), aH1));
  EXPECT(assert_equal(numericalDerivative22(retract, kState1, xi), aH2));

  // Check retract derivatives on state 2
  const Vector9 xi2 = -3.0*xi;
  state2.retract(xi2, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(retract, state2, xi2), aH1));
  EXPECT(assert_equal(numericalDerivative22(retract, state2, xi2), aH2));

  // Check localCoordinates derivatives
  std::function<Vector9(const NavState&, const NavState&)> local =
      std::bind(&NavState::localCoordinates, std::placeholders::_1,
                std::placeholders::_2, nullptr, nullptr);
  // from state1 to state2
  kState1.localCoordinates(state2, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(local, kState1, state2), aH1));
  EXPECT(assert_equal(numericalDerivative22(local, kState1, state2), aH2));
  // from identity to state2
  kIdentity.localCoordinates(state2, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(local, kIdentity, state2), aH1));
  EXPECT(assert_equal(numericalDerivative22(local, kIdentity, state2), aH2));
  // from state2 to identity
  state2.localCoordinates(kIdentity, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(local, state2, kIdentity), aH1));
  EXPECT(assert_equal(numericalDerivative22(local, state2, kIdentity), aH2));
}

/* ************************************************************************* */
static const double dt = 2.0;
std::function<Vector9(const NavState&, const bool&)> coriolis =
    std::bind(&NavState::coriolis, std::placeholders::_1, dt, kOmegaCoriolis,
              std::placeholders::_2, nullptr);

TEST(NavState, Coriolis) {
  Matrix9 aH;

  // first-order
  kState1.coriolis(dt, kOmegaCoriolis, false, aH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, kState1, false), aH));
  // second-order
  kState1.coriolis(dt, kOmegaCoriolis, true, aH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, kState1, true), aH));
}

TEST(NavState, Coriolis2) {
  Matrix9 aH;
  NavState state2(Rot3::RzRyRx(M_PI / 12.0, M_PI / 6.0, M_PI / 4.0),
      Point3(5.0, 1.0, -50.0), Vector3(0.5, 0.0, 0.0));

  // first-order
  state2.coriolis(dt, kOmegaCoriolis, false, aH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, state2, false), aH));
  // second-order
  state2.coriolis(dt, kOmegaCoriolis, true, aH);
  EXPECT(assert_equal(numericalDerivative21(coriolis, state2, true), aH));
}

TEST(NavState, Coriolis3) {
  /** Consider a massless planet with an attached nav frame at 
   *  n_omega = [0 0 1]', and a body at position n_t = [1 0 0]', travelling with 
   *  velocity n_v = [0 1 0]'. Orient the body so that it is not instantaneously
   *  aligned with the nav frame (i.e., nRb != I_3x3). Test that first and 
   *  second order Coriolis corrections are as expected.
   */

  // Get true first and second order coriolis accelerations
  double dt = 2.0, dt2 = dt * dt;
  Vector3 n_omega(0.0, 0.0, 1.0), n_t(1.0, 0.0, 0.0), n_v(0.0, 1.0, 0.0);
  Vector3 n_aCorr1 = -2.0 * n_omega.cross(n_v),
          n_aCorr2 = -n_omega.cross(n_omega.cross(n_t));
  Rot3 nRb = Rot3(-1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0),
       bRn = nRb.inverse();

  // Get expected first and second order corrections in the nav frame
  Vector3 n_dP1e = 0.5 * dt2 * n_aCorr1, 
          n_dP2e = 0.5 * dt2 * (n_aCorr1 + n_aCorr2),
          n_dV1e = dt * n_aCorr1, 
          n_dV2e = dt * (n_aCorr1 + n_aCorr2);

  // Get expected first and second order corrections in the body frame
  Vector3 dRe = -dt * (bRn * n_omega),
          b_dP1e = bRn * n_dP1e, b_dP2e = bRn * n_dP2e,
          b_dV1e = bRn * n_dV1e, b_dV2e = bRn * n_dV2e;

  // Get actual first and scond order corrections in body frame
  NavState kState2(nRb, n_t, n_v);
  Vector9 dXi1a = kState2.coriolis(dt, n_omega, false),
          dXi2a = kState2.coriolis(dt, n_omega, true);
  Vector3 dRa = NavState::dR(dXi1a),
          b_dP1a = NavState::dP(dXi1a), b_dV1a = NavState::dV(dXi1a),
          b_dP2a = NavState::dP(dXi2a), b_dV2a = NavState::dV(dXi2a);

  EXPECT(assert_equal(dRe, dRa));
  EXPECT(assert_equal(b_dP1e, b_dP1a));
  EXPECT(assert_equal(b_dV1e, b_dV1a));
  EXPECT(assert_equal(b_dP2e, b_dP2a));
  EXPECT(assert_equal(b_dV2e, b_dV2a));

}

/* ************************************************************************* */
TEST(NavState, CorrectPIM) {
  Vector9 xi;
  xi << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  double dt = 0.5;
  Matrix9 aH1, aH2;
  std::function<Vector9(const NavState&, const Vector9&)> correctPIM =
      std::bind(&NavState::correctPIM, std::placeholders::_1,
                std::placeholders::_2, dt, kGravity, kOmegaCoriolis, false,
                nullptr, nullptr);
  kState1.correctPIM(xi, dt, kGravity, kOmegaCoriolis, false, aH1, aH2);
  EXPECT(assert_equal(numericalDerivative21(correctPIM, kState1, xi), aH1));
  EXPECT(assert_equal(numericalDerivative22(correctPIM, kState1, xi), aH2));
}

/* ************************************************************************* */
TEST(NavState, Stream)
{
  NavState state;

  std::ostringstream os;
  os << state;

  string expected;
  expected = "R: [\n\t1, 0, 0;\n\t0, 1, 0;\n\t0, 0, 1\n]\np: 0 0 0\nv: 0 0 0";

  EXPECT(os.str() == expected);
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

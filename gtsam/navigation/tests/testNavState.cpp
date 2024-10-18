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

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/navigation/NavState.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;


GTSAM_CONCEPT_TESTABLE_INST(NavState)
GTSAM_CONCEPT_LIE_INST(NavState)

static const Rot3 kAttitude = Rot3::RzRyRx(0.1, 0.2, 0.3);
static const Point3 kPosition(1.0, 2.0, 3.0);
static const Pose3 kPose(kAttitude, kPosition);
static const Velocity3 kVelocity(0.4, 0.5, 0.6);
static const NavState kIdentity;
static const NavState kState1(kAttitude, kPosition, kVelocity);
static const Vector3 kOmegaCoriolis(0.02, 0.03, 0.04);
static const Vector3 kGravity(0, 0, 9.81);
static const Vector9 kZeroXi = Vector9::Zero();

static const Point3 V(3, 0.4, -2.2);
static const Point3 P(0.2, 0.7, -2);
static const Rot3 R = Rot3::Rodrigues(0.3, 0, 0);
static const Point3 V2(-6.5, 3.5, 6.2);
static const Point3 P2(3.5, -8.2, 4.2);
static const NavState T(R, P2, V2);
static const NavState T2(Rot3::Rodrigues(0.3, 0.2, 0.1), P2, V2);
static const NavState T3(Rot3::Rodrigues(-90, 0, 0), Point3(5, 6, 7),
                         Point3(1, 2, 3));

/* ************************************************************************* */
TEST(NavState, Constructor) {
  std::function<NavState(const Rot3&, const Point3&, const Vector3&)> create =
      std::bind(&NavState::Create, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, nullptr, nullptr, nullptr);
  Matrix aH1, aH2, aH3;
  EXPECT(assert_equal(kState1, NavState::Create(kAttitude, kPosition, kVelocity,
                                                aH1, aH2, aH3)));
  EXPECT(assert_equal(
      numericalDerivative31(create, kAttitude, kPosition, kVelocity), aH1));
  EXPECT(assert_equal(
      numericalDerivative32(create, kAttitude, kPosition, kVelocity), aH2));
  EXPECT(assert_equal(
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
TEST( NavState, Manifold) {
  // Check zero xi
  EXPECT(assert_equal(kIdentity, kIdentity.retract(kZeroXi)));
  EXPECT(assert_equal(kZeroXi, kIdentity.localCoordinates(kIdentity)));
  EXPECT(assert_equal(kState1, kState1.retract(kZeroXi)));
  EXPECT(assert_equal(kZeroXi, kState1.localCoordinates(kState1)));

  // Check definition of retract as operating on components separately
  Vector9 d;
  d << 0.1, 0.1, 0.1, 0.02, 0.03, 0.04, -0.01, -0.02, -0.03;
  Rot3 drot = Rot3::Expmap(d.head<3>());
  Point3 dt = Point3(d.segment<3>(3));
  Velocity3 dvel = Velocity3(d.tail<3>());
  NavState state2 = NavState(kState1.attitude() * drot,
      kState1.position() + kState1.attitude() * dt,
      kState1.velocity() + kState1.attitude() * dvel);

  Vector9 xi = kState1.localCoordinates(state2);

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
TEST(NavState, Equals) {
  NavState T3(Rot3::Rodrigues(-90, 0, 0), Point3(5, 6, 7), Point3(1, 2, 3));
  NavState pose2 = T3;
  EXPECT(T3.equals(pose2));
  NavState origin;
  EXPECT(!T3.equals(origin));
}

/* ************************************************************************* */
TEST(NavState, Compose) {
  NavState nav_state_a(Rot3::Identity(), {0.0, 1.0, 2.0}, {1.0, -1.0, 1.0});
  NavState nav_state_b(Rot3::Rx(M_PI_4), {0.0, 1.0, 3.0}, {1.0, -1.0, 2.0});
  NavState nav_state_c(Rot3::Ry(M_PI / 180.0), {1.0, 1.0, 2.0},
                       {3.0, -1.0, 1.0});

  auto ab_c = (nav_state_a * nav_state_b) * nav_state_c;
  auto a_bc = nav_state_a * (nav_state_b * nav_state_c);
  CHECK(assert_equal(ab_c, a_bc));

  Matrix actual = (T2 * T2).matrix();

  Matrix expected = T2.matrix() * T2.matrix();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix actualDcompose1, actualDcompose2;
  T2.compose(T2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 =
      numericalDerivative21(testing::compose<NavState>, T2, T2);

  EXPECT(assert_equal(numericalH1, actualDcompose1, 5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(), actualDcompose1, 5e-3));

  Matrix numericalH2 =
      numericalDerivative22(testing::compose<NavState>, T2, T2);
  EXPECT(assert_equal(numericalH2, actualDcompose2, 1e-4));
}

/* ************************************************************************* */
// Check compose and its pushforward, another case
TEST(NavState, Compose2) {
  const NavState& T1 = T;
  Matrix actual = (T1 * T2).matrix();
  Matrix expected = T1.matrix() * T2.matrix();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix actualDcompose1, actualDcompose2;
  T1.compose(T2, actualDcompose1, actualDcompose2);

  Matrix numericalH1 = numericalDerivative21(testing::compose<NavState>, T1, T2);
  EXPECT(assert_equal(numericalH1, actualDcompose1, 5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(), actualDcompose1, 5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::compose<NavState>, T1, T2);
  EXPECT(assert_equal(numericalH2, actualDcompose2, 1e-5));
}

/* ************************************************************************* */
TEST(NavState, Inverse) {
  NavState nav_state_a(Rot3::Identity(), {0.0, 1.0, 2.0}, {1.0, -1.0, 1.0});
  NavState nav_state_b(Rot3::Rx(M_PI_4), {0.0, 1.0, 3.0}, {1.0, -1.0, 2.0});
  NavState nav_state_c(Rot3::Ry(M_PI / 180.0), {1.0, 1.0, 2.0},
                       {3.0, -1.0, 1.0});

  auto a_inv = nav_state_a.inverse();
  auto a_a_inv = nav_state_a * a_inv;
  CHECK(assert_equal(a_a_inv, NavState()));

  auto b_inv = nav_state_b.inverse();
  auto b_b_inv = nav_state_b * b_inv;
  CHECK(assert_equal(b_b_inv, NavState()));

  Matrix actualDinverse;
  Matrix actual = T.inverse(actualDinverse).matrix();
  Matrix expected = T.matrix().inverse();
  EXPECT(assert_equal(actual, expected, 1e-8));

  Matrix numericalH = numericalDerivative11(testing::inverse<NavState>, T);
  EXPECT(assert_equal(numericalH, actualDinverse, 5e-3));
  EXPECT(assert_equal(-T.AdjointMap(), actualDinverse, 5e-3));
}

/* ************************************************************************* */
TEST(NavState, InverseDerivatives) {
  Rot3 R = Rot3::Rodrigues(0.3, 0.4, -0.5);
  Vector3 v(3.5, -8.2, 4.2);
  Point3 p(3.5, -8.2, 4.2);
  NavState T(R, p, v);

  Matrix numericalH = numericalDerivative11(testing::inverse<NavState>, T);
  Matrix actualDinverse;
  T.inverse(actualDinverse);
  EXPECT(assert_equal(numericalH, actualDinverse, 5e-3));
  EXPECT(assert_equal(-T.AdjointMap(), actualDinverse, 5e-3));
}

/* ************************************************************************* */
TEST(NavState, Compose_Inverse) {
  NavState actual = T * T.inverse();
  NavState expected;
  EXPECT(assert_equal(actual, expected, 1e-8));
}

/* ************************************************************************* */
TEST(NavState, Between) {
  NavState s1, s2(Rot3(), Point3(1, 2, 3), Velocity3(0, 0, 0));

  NavState actual = s1.compose(s2);
  EXPECT(assert_equal(s2, actual));

  NavState between = s2.between(s1);
  NavState expected_between(Rot3(), Point3(-1, -2, -3), Velocity3(0, 0, 0));
  EXPECT(assert_equal(expected_between, between));

  NavState expected = T2.inverse() * T3;
  Matrix actualDBetween1, actualDBetween2;
  actual = T2.between(T3, actualDBetween1, actualDBetween2);
  EXPECT(assert_equal(expected, actual));

  Matrix numericalH1 = numericalDerivative21(testing::between<NavState>, T2, T3);
  EXPECT(assert_equal(numericalH1, actualDBetween1, 5e-3));

  Matrix numericalH2 = numericalDerivative22(testing::between<NavState>, T2, T3);
  EXPECT(assert_equal(numericalH2, actualDBetween2, 1e-5));
}

/* ************************************************************************* */
TEST(NavState, interpolate) {
  EXPECT(assert_equal(T2, interpolate(T2, T3, 0.0)));
  EXPECT(assert_equal(T3, interpolate(T2, T3, 1.0)));
}

/* ************************************************************************* */
TEST(NavState, Lie) {
  NavState nav_state_a(Rot3::Identity(), {0.0, 1.0, 2.0}, {1.0, -1.0, 1.0});
  NavState nav_state_b(Rot3::Rx(M_PI_4), {0.0, 1.0, 3.0}, {1.0, -1.0, 2.0});
  NavState nav_state_c(Rot3::Ry(M_PI / 180.0), {1.0, 1.0, 2.0},
                       {3.0, -1.0, 1.0});

  // logmap
  Matrix9 H1, H2;
  auto logmap_b = NavState::Create(Rot3::Identity(),
                                          Vector3::Zero(), Vector3::Zero())
                      .localCoordinates(nav_state_b, H1, H2);

  // TODO(Varun)
  Matrix6 J1, J2;
  // auto logmap_pose_b = Pose3::Create(Rot3(), Vector3::Zero())
  //                          .localCoordinates(nav_state_b.pose(), J1, J2);

  // //  Check retraction
  // auto retraction_b = NavState().retract(logmap_b);
  // CHECK(assert_equal(retraction_b, nav_state_b));

  // // Test if the sum of the logmap is the same as the logmap of the product
  // auto logmap_c = NavState::Create(Rot3::Identity(),
  //                                         Vector3::Zero(), Vector3::Zero())
  //                     .localCoordinates(nav_state_c);

  // auto logmap_bc = NavState::Create(
  //                       gtsam::Rot3::Identity(), Eigen::Vector3d::Zero(),
  //                       Eigen::Vector3d::Zero(), {}, {}, {})
  //                       .localCoordinates(nav_state_b * nav_state_c);
  // Vector9 logmap_bc2 = NavState::Logmap(nav_state_b * nav_state_c);

  // Vector9 logmap_bc_sum = logmap_b + logmap_c;
  // std::cout << "logmap_bc = " << logmap_bc.transpose() << std::endl;
  // std::cout << "logmap_bc2 = " << logmap_bc2.transpose() << std::endl;

  // // std::cout << "logmap_bc + logmap_c = " << logmap_bc_sum.transpose() << std::endl;
  // // std::cout << "logmap_b + logmap_c = " << (NavState::Logmap(nav_state_b) + NavState::Logmap(nav_state_c)).transpose() << std::endl;
  // // std::cout << "logmap_bc = " << logmap_bc.transpose() << std::endl;
  // // CHECK(assert_equal(logmap_bc_sum, logmap_bc));
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
TEST(NavState, Print) {
  NavState state(Rot3::Identity(), Point3(1, 2, 3), Vector3(1, 2, 3));

  // Generate the expected output
  std::string R = "R: [\n\t1, 0, 0;\n\t0, 1, 0;\n\t0, 0, 1\n]\n";
  std::string p = "p: 1 2 3\n";
  std::string v = "v: 1 2 3\n";
  std::string expected = R + p + v;

  EXPECT(assert_print_equal(expected, state));
}

/* ************************************************************************* */
#ifndef GTSAM_POSE3_EXPMAP
TEST(NavState, Retract_first_order) {
  NavState id;
  Vector v = Z_9x1;
  v(0) = 0.3;
  EXPECT(assert_equal(NavState(R, Point3(0, 0, 0), Vector3(0, 0, 0)),
                      id.retract(v), 1e-2));
  v(3) = 0.2;
  v(4) = 0.7;
  v(5) = -2;
  v(6) = 3;
  v(7) = 0.4;
  v(8) = -2.2;
  EXPECT(assert_equal(NavState(R, P, V), id.retract(v), 1e-2));
}
#endif

/* ************************************************************************* */
TEST(NavState, RetractExpmap) {
  Vector xi = Z_9x1;
  xi(0) = 0.3;
  NavState pose = NavState::Expmap(xi),
           expected(R, Point3(0, 0, 0), Point3(0, 0, 0));
  EXPECT(assert_equal(expected, pose, 1e-2));
  EXPECT(assert_equal(xi, NavState::Logmap(pose), 1e-2));
}

/* ************************************************************************* */
TEST(NavState, Expmap_A_Full) {
  NavState id;
  Vector xi = Z_9x1;
  xi(0) = 0.3;
  EXPECT(assert_equal(expmap_default<NavState>(id, xi),
                      NavState(R, Point3(0, 0, 0), Point3(0, 0, 0))));
  xi(3) = -0.2;
  xi(4) = -0.394742;
  xi(5) = 2.08998;
  xi(6) = 0.2;
  xi(7) = 0.394742;
  xi(8) = -2.08998;

  NavState expected(R, -P, P);
  EXPECT(assert_equal(expected, expmap_default<NavState>(id, xi), 1e-5));
}

/* ************************************************************************* */
TEST(NavState, Expmap_b) {
  NavState p1(Rot3(), Point3(-100, 0, 0), Point3(100, 0, 0));
  NavState p2 = p1.retract(
      (Vector(9) << 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished());
  NavState expected(Rot3::Rodrigues(0.0, 0.0, 0.1), Point3(-100.0, 0.0, 0.0),
                    Point3(100.0, 0.0, 0.0));
  EXPECT(assert_equal(expected, p2, 1e-2));
}

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screwNavState {
double a = 0.3, c = cos(a), s = sin(a), w = 0.3;
Vector xi = (Vector(9) << 0.0, 0.0, w, w, 0.0, 1.0, w, 0.0, 1.0).finished();
Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
Point3 expectedV(0.29552, 0.0446635, 1);
Point3 expectedP(0.29552, 0.0446635, 1);
NavState expected(expectedR, expectedV, expectedP);
}  // namespace screwNavState

/* ************************************************************************* */
// Checks correct exponential map (Expmap) with brute force matrix exponential
TEST(NavState, Expmap_c_full) {
  //TODO(Varun)
  // EXPECT(assert_equal(screwNavState::expected,
  //                     expm<NavState>(screwNavState::xi), 1e-6));
  // EXPECT(assert_equal(screwNavState::expected,
  //                     NavState::Expmap(screwNavState::xi), 1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(NavState, Adjoint_full) {
  NavState expected = T * NavState::Expmap(screwNavState::xi) * T.inverse();
  Vector xiprime = T.Adjoint(screwNavState::xi);
  EXPECT(assert_equal(expected, NavState::Expmap(xiprime), 1e-6));

  NavState expected2 = T2 * NavState::Expmap(screwNavState::xi) * T2.inverse();
  Vector xiprime2 = T2.Adjoint(screwNavState::xi);
  EXPECT(assert_equal(expected2, NavState::Expmap(xiprime2), 1e-6));

  NavState expected3 = T3 * NavState::Expmap(screwNavState::xi) * T3.inverse();
  Vector xiprime3 = T3.Adjoint(screwNavState::xi);
  EXPECT(assert_equal(expected3, NavState::Expmap(xiprime3), 1e-6));
}

/* ************************************************************************* */
// assert that T*wedge(xi)*T^-1 is equal to wedge(Ad_T(xi))
TEST(NavState, Adjoint_hat) {
  //TODO(Varun)
  // auto hat = [](const Vector& xi) { return ::wedge<NavState>(xi); };
  // Matrix5 expected = T.matrix() * hat(screwNavState::xi) * T.matrix().inverse();
  // Matrix5 xiprime = hat(T.Adjoint(screwNavState::xi));

  // EXPECT(assert_equal(expected, xiprime, 1e-6));

  // Matrix5 expected2 =
  //     T2.matrix() * hat(screwNavState::xi) * T2.matrix().inverse();
  // Matrix5 xiprime2 = hat(T2.Adjoint(screwNavState::xi));
  // EXPECT(assert_equal(expected2, xiprime2, 1e-6));

  // Matrix5 expected3 =
  //     T3.matrix() * hat(screwNavState::xi) * T3.matrix().inverse();

  // Matrix5 xiprime3 = hat(T3.Adjoint(screwNavState::xi));
  // EXPECT(assert_equal(expected3, xiprime3, 1e-6));
}

/* ************************************************************************* */
TEST(NavState, Adjoint_compose_full) {
  // To debug derivatives of compose, assert that
  // T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
  const NavState& T1 = T;
  Vector x = (Vector(9) << 0.1, 0.1, 0.1, 0.4, 0.2, 0.8, 0.4, 0.2, 0.8).finished();
  NavState expected = T1 * NavState::Expmap(x) * T2;
  Vector y = T2.inverse().Adjoint(x);
  NavState actual = T1 * T2 * NavState::Expmap(y);
  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* ************************************************************************* */
TEST(NavState, expmaps_galore_full) {
  Vector xi;
  //TODO(Varun)
  // NavState actual;
  // xi = (Vector(9) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9).finished();
  // actual = NavState::Expmap(xi);
  // EXPECT(assert_equal(expm<NavState>(xi), actual, 1e-6));
  // EXPECT(assert_equal(xi, NavState::Logmap(actual), 1e-6));

  // xi = (Vector(9) << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, -0.7, -0.8, -0.9)
  //          .finished();
  // for (double theta = 1.0; 0.3 * theta <= M_PI; theta *= 2) {
  //   Vector txi = xi * theta;
  //   actual = NavState::Expmap(txi);
  //   EXPECT(assert_equal(expm<NavState>(txi, 30), actual, 1e-6));
  //   Vector log = NavState::Logmap(actual);
  //   EXPECT(assert_equal(actual, NavState::Expmap(log), 1e-6));
  //   EXPECT(assert_equal(txi, log, 1e-6));  // not true once wraps
  // }

  // // Works with large v as well, but expm needs 10 iterations!
  // xi =
  //     (Vector(9) << 0.2, 0.3, -0.8, 100.0, 120.0, -60.0, 12, 14, 45).finished();
  // actual = NavState::Expmap(xi);
  // EXPECT(assert_equal(expm<NavState>(xi, 10), actual, 1e-5));
  // EXPECT(assert_equal(xi, NavState::Logmap(actual), 1e-9));
}

/* ************************************************************************* */
TEST(NavState, Retract_LocalCoordinates) {
  Vector9 d;
  d << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  d /= 10;
  const Rot3 R = Rot3::Retract(d.head<3>());
  NavState t = NavState::Retract(d);
  EXPECT(assert_equal(d, NavState::LocalCoordinates(t)));
}
/* ************************************************************************* */
TEST(NavState, retract_localCoordinates) {
  Vector9 d12;
  d12 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  d12 /= 10;
  NavState t1 = T, t2 = t1.retract(d12);
  EXPECT(assert_equal(d12, t1.localCoordinates(t2)));
}
/* ************************************************************************* */
TEST(NavState, expmap_logmap) {
  Vector d12 = Vector9::Constant(0.1);
  NavState t1 = T, t2 = t1.expmap(d12);
  EXPECT(assert_equal(d12, t1.logmap(t2)));
}

/* ************************************************************************* */
TEST(NavState, retract_localCoordinates2) {
  NavState t1 = T;
  NavState t2 = T3;
  NavState origin;
  Vector d12 = t1.localCoordinates(t2);
  EXPECT(assert_equal(t2, t1.retract(d12)));
  Vector d21 = t2.localCoordinates(t1);
  EXPECT(assert_equal(t1, t2.retract(d21)));
  // EXPECT(assert_equal(d12, -d21));
}
/* ************************************************************************* */
TEST(NavState, manifold_expmap) {
  NavState t1 = T;
  NavState t2 = T3;
  NavState origin;
  Vector d12 = t1.logmap(t2);
  EXPECT(assert_equal(t2, t1.expmap(d12)));
  Vector d21 = t2.logmap(t1);
  EXPECT(assert_equal(t1, t2.expmap(d21)));

  // Check that log(t1,t2)=-log(t2,t1)
  EXPECT(assert_equal(d12, -d21));
}

/* ************************************************************************* */
TEST(NavState, subgroups) {
  // Frank - Below only works for correct "Agrawal06iros style expmap
  // lines in canonical coordinates correspond to Abelian subgroups in SE(3)
  Vector d = (Vector(9) << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9).finished();
  // exp(-d)=inverse(exp(d))
  EXPECT(assert_equal(NavState::Expmap(-d), NavState::Expmap(d).inverse()));
  // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
  NavState T2 = NavState::Expmap(2 * d);
  NavState T3 = NavState::Expmap(3 * d);
  NavState T5 = NavState::Expmap(5 * d);
  EXPECT(assert_equal(T5, T2 * T3));
  EXPECT(assert_equal(T5, T3 * T2));
}

/* ************************************************************************* */
TEST(NavState, adjointMap) {
  Matrix res = NavState::adjointMap(screwNavState::xi);
  Matrix wh = skewSymmetric(screwNavState::xi(0), screwNavState::xi(1), screwNavState::xi(2));
  Matrix vh = skewSymmetric(screwNavState::xi(3), screwNavState::xi(4), screwNavState::xi(5));
  Matrix rh = skewSymmetric(screwNavState::xi(6), screwNavState::xi(7), screwNavState::xi(8));
  Matrix9 expected;
  expected << wh, Z_3x3, Z_3x3, vh, wh, Z_3x3, rh, Z_3x3, wh;
  EXPECT(assert_equal(expected, res, 1e-5));
}

/* ************************************************************************* */

TEST(NavState, ExpmapDerivative1) {
  Matrix9 actualH;
  Vector9 w;
  w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  NavState::Expmap(w, actualH);

  std::function<NavState(const Vector9&)> f = [](const Vector9& w) { return NavState::Expmap(w); };
  Matrix expectedH =
      numericalDerivative21<NavState, Vector9, OptionalJacobian<9, 9> >(&NavState::Expmap, w, {});
  EXPECT(assert_equal(expectedH, actualH));
}

/* ************************************************************************* */
TEST(NavState, LogmapDerivative) {
  Matrix9 actualH;
  Vector9 w;
  w << 0.1, 0.2, 0.3, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
  NavState p = NavState::Expmap(w);
  EXPECT(assert_equal(w, NavState::Logmap(p, actualH), 1e-5));

  std::function<Vector9(const NavState&)> f = [](const NavState& p) { return NavState::Logmap(p); };
  Matrix expectedH =
      numericalDerivative21<Vector9, NavState, OptionalJacobian<9, 9> >(&NavState::Logmap, p, {});
  EXPECT(assert_equal(expectedH, actualH));
}

//******************************************************************************
TEST(NavState, Invariants) {
  NavState id;

  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, T3));
  EXPECT(check_group_invariants(T2, id));
  EXPECT(check_group_invariants(T2, T3));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, T3));
  EXPECT(check_manifold_invariants(T2, id));
  EXPECT(check_manifold_invariants(T2, T3));
}

//******************************************************************************
TEST(NavState, LieGroupDerivatives) {
  NavState id;

  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, T2);
  CHECK_LIE_GROUP_DERIVATIVES(T2, id);
  CHECK_LIE_GROUP_DERIVATIVES(T2, T3);
}

//******************************************************************************
TEST(NavState, ChartDerivatives) {
  NavState id;
  if (ROT3_DEFAULT_COORDINATES_MODE == Rot3::EXPMAP) {
    CHECK_CHART_DERIVATIVES(id, id);
    //    CHECK_CHART_DERIVATIVES(id,T2);
    //    CHECK_CHART_DERIVATIVES(T2,id);
    //    CHECK_CHART_DERIVATIVES(T2,T3);
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

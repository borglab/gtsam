/**
 * @file testPoseRTV
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam_unstable/dynamics/PoseRTV.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(PoseRTV)
GTSAM_CONCEPT_MATRIX_LIE_GROUP_INST(PoseRTV)

static const Rot3 rot = Rot3::RzRyRx(0.1, 0.2, 0.3);
static const Point3 pt(1.0, 2.0, 3.0);
static const Velocity3 vel(0.4, 0.5, 0.6);
static const Vector3 kZero3 = Vector3::Zero();

/* ************************************************************************* */
TEST( testPoseRTV, constructors ) {
  PoseRTV state1(pt, rot, vel);
  EXPECT(assert_equal(pt, state1.t()));
  EXPECT(assert_equal(rot, state1.R()));
  EXPECT(assert_equal(vel, state1.v()));
  EXPECT(assert_equal(Pose3(rot, pt), state1.pose()));

  PoseRTV state2;
  EXPECT(assert_equal(Point3(0,0,0),  state2.t()));
  EXPECT(assert_equal(Rot3(), state2.R()));
  EXPECT(assert_equal(kZero3, state2.v()));
  EXPECT(assert_equal(Pose3(), state2.pose()));

  PoseRTV state3(Pose3(rot, pt), vel);
  EXPECT(assert_equal(pt,  state3.t()));
  EXPECT(assert_equal(rot, state3.R()));
  EXPECT(assert_equal(vel, state3.v()));
  EXPECT(assert_equal(Pose3(rot, pt), state3.pose()));

  PoseRTV state4(Pose3(rot, pt));
  EXPECT(assert_equal(pt,  state4.t()));
  EXPECT(assert_equal(rot, state4.R()));
  EXPECT(assert_equal(kZero3, state4.v()));
  EXPECT(assert_equal(Pose3(rot, pt), state4.pose()));

  Vector vec_init = (Vector(9) << 0.1, 0.2, 0.3,  1.0, 2.0, 3.0,  0.4, 0.5, 0.6).finished();
  PoseRTV state5(vec_init);
  EXPECT(assert_equal(pt,  state5.t()));
  EXPECT(assert_equal(rot, state5.R()));
  EXPECT(assert_equal(vel, state5.v()));
  EXPECT(assert_equal(vec_init, state5.vector()));
}

/* ************************************************************************* */
TEST( testPoseRTV, dim ) {
  PoseRTV state1(pt, rot, vel);
  EXPECT_LONGS_EQUAL(9, state1.dim());
  EXPECT_LONGS_EQUAL(9, PoseRTV::Dim());
}

/* ************************************************************************* */
TEST( testPoseRTV, equals ) {
  PoseRTV state1, state2(pt, rot, vel), state3(state2), state4(Pose3(rot, pt));
  EXPECT(assert_equal(state1, state1));
  EXPECT(assert_equal(state2, state3));
  EXPECT(assert_equal(state3, state2));
  EXPECT(assert_inequal(state1, state2));
  EXPECT(assert_inequal(state2, state1));
  EXPECT(assert_inequal(state2, state4));
}

/* ************************************************************************* */
TEST( testPoseRTV, Lie ) {
  // origin and zero deltas
  PoseRTV identity;
  EXPECT(assert_equal(identity, (PoseRTV)identity.retract(Z_9x1)));
  EXPECT(assert_equal((Vector) Z_9x1, identity.localCoordinates(identity)));

  PoseRTV state1(pt, rot, vel);
  EXPECT(assert_equal(state1, (PoseRTV)state1.retract(Z_9x1)));
  EXPECT(assert_equal((Vector) Z_9x1, state1.localCoordinates(state1)));

  Vector delta(9);
  delta << 0.1, 0.1, 0.1, 0.2, 0.3, 0.4,-0.1,-0.2,-0.3;
  Pose3 pose2 = Pose3(rot, pt).retract(delta.head<6>());
  Velocity3 vel2 = vel + Velocity3(-0.1, -0.2, -0.3);
  PoseRTV state2(pose2.translation(), pose2.rotation(), vel2);
  EXPECT(assert_equal(state2, (PoseRTV)state1.retract(delta)));
  EXPECT(assert_equal(delta, state1.localCoordinates(state2)));

  // roundtrip from state2 to state3 and back
  PoseRTV state3 = state2.retract(delta);
  EXPECT(assert_equal(delta, state2.localCoordinates(state3)));

  // roundtrip from state3 to state4 and back, with expmap.
  PoseRTV state4 = state3.expmap(delta);
  EXPECT(assert_equal(delta, state3.logmap(state4)));

  // For the expmap/logmap (not necessarily retract/local) -delta goes other way
  EXPECT(assert_equal(state3, (PoseRTV)state4.expmap(-delta)));
  EXPECT(assert_equal(delta, -state4.logmap(state3)));
}

/* ************************************************************************* */
TEST( testPoseRTV, dynamics_identities ) {
  // general dynamics should produce the same measurements as the imuPrediction function
  PoseRTV x0, x1, x2, x3, x4;

  const double dt = 0.1;
  Vector accel = Vector3(0.2, 0.0, 0.0), gyro = Vector3(0.0, 0.0, 0.2);
  Vector imu01 = Z_6x1, imu12 = Z_6x1, imu23 = Z_6x1, imu34 = Z_6x1;

  x1 = x0.generalDynamics(accel, gyro, dt);
  x2 = x1.generalDynamics(accel, gyro, dt);
  x3 = x2.generalDynamics(accel, gyro, dt);
  x4 = x3.generalDynamics(accel, gyro, dt);

//  EXPECT(assert_equal(imu01, x0.imuPrediction(x1, dt).first));
//  EXPECT(assert_equal(imu12, x1.imuPrediction(x2, dt).first));
//  EXPECT(assert_equal(imu23, x2.imuPrediction(x3, dt).first));
//  EXPECT(assert_equal(imu34, x3.imuPrediction(x4, dt).first));
//
//  EXPECT(assert_equal(x1.translation(), x0.imuPrediction(x1, dt).second));
//  EXPECT(assert_equal(x2.translation(), x1.imuPrediction(x2, dt).second));
//  EXPECT(assert_equal(x3.translation(), x2.imuPrediction(x3, dt).second));
//  EXPECT(assert_equal(x4.translation(), x3.imuPrediction(x4, dt).second));
}


/* ************************************************************************* */
PoseRTV compose_proxy(const PoseRTV& A, const PoseRTV& B) { return A.compose(B); }
TEST( testPoseRTV, compose ) {
  PoseRTV state1(pt, rot, vel), state2 = state1;

  Matrix actH1, actH2;
  state1.compose(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(compose_proxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(compose_proxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1));
  EXPECT(assert_equal(numericH2, actH2));
}

/* ************************************************************************* */
PoseRTV between_proxy(const PoseRTV& A, const PoseRTV& B) { return A.between(B); }
TEST( testPoseRTV, between ) {
  PoseRTV state1(pt, rot, vel), state2 = state1;

  Matrix actH1, actH2;
  state1.between(state2, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(between_proxy, state1, state2);
  Matrix numericH2 = numericalDerivative22(between_proxy, state1, state2);
  EXPECT(assert_equal(numericH1, actH1));
  EXPECT(assert_equal(numericH2, actH2));
}

/* ************************************************************************* */
PoseRTV inverse_proxy(const PoseRTV& A) { return A.inverse(); }
TEST( testPoseRTV, inverse ) {
  PoseRTV state1(pt, rot, vel);

  Matrix actH1;
  state1.inverse(actH1);
  Matrix numericH1 = numericalDerivative11(inverse_proxy, state1);
  EXPECT(assert_equal(numericH1, actH1));
}

/* ************************************************************************* */
double range_proxy(const PoseRTV& A, const PoseRTV& B) { return A.range(B); }
TEST( testPoseRTV, range ) {
  Point3 tA(1.0, 2.0, 3.0), tB(3.0, 2.0, 3.0);
  PoseRTV rtvA(tA), rtvB(tB);
  EXPECT_DOUBLES_EQUAL(0.0, rtvA.range(rtvA), 1e-9);
  EXPECT_DOUBLES_EQUAL(2.0, rtvA.range(rtvB), 1e-9);
  EXPECT_DOUBLES_EQUAL(2.0, rtvB.range(rtvA), 1e-9);

  Matrix actH1, actH2;
  rtvA.range(rtvB, actH1, actH2);
  Matrix numericH1 = numericalDerivative21(range_proxy, rtvA, rtvB);
  Matrix numericH2 = numericalDerivative22(range_proxy, rtvA, rtvB);
  EXPECT(assert_equal(numericH1, actH1));
  EXPECT(assert_equal(numericH2, actH2));
}

/* ************************************************************************* */
PoseRTV transformed_from_proxy(const PoseRTV& a, const Pose3& trans) {
  return a.transformed_from(trans);
}
TEST( testPoseRTV, transformed_from_1 ) {
  Rot3 R = Rot3::Rodrigues(0.1, 0.2, 0.3);
  Point3 T(1.0, 2.0, 3.0);
  Velocity3 V(2.0, 3.0, 4.0);
  PoseRTV start(R, T, V);
  Pose3 transform(Rot3::Yaw(M_PI_2), Point3(1.0, 2.0, 3.0));

  Matrix actDTrans, actDGlobal;
  PoseRTV actual = start.transformed_from(transform, actDGlobal, actDTrans);
  PoseRTV expected(transform.compose(start.pose()), transform.rotation().matrix() * V);
  EXPECT(assert_equal(expected, actual));

  Matrix numDGlobal = numericalDerivative21(transformed_from_proxy, start, transform, 1e-5); // At 1e-8, fails
  Matrix numDTrans = numericalDerivative22(transformed_from_proxy, start, transform, 1e-8); // Sensitive to step size
  EXPECT(assert_equal(numDGlobal, actDGlobal));
  EXPECT(assert_equal(numDTrans, actDTrans, 1e-5)); // FIXME: still needs analytic derivative
}

/* ************************************************************************* */
TEST( testPoseRTV, transformed_from_2 ) {
  Rot3 R;
  Point3 T(1.0, 2.0, 3.0);
  Velocity3 V(2.0, 3.0, 4.0);
  PoseRTV start(R, T, V);
  Pose3 transform(Rot3::Yaw(M_PI_2), Point3(1.0, 2.0, 3.0));

  Matrix actDTrans, actDGlobal;
  PoseRTV actual = start.transformed_from(transform, actDGlobal, actDTrans);
  PoseRTV expected(transform.compose(start.pose()), transform.rotation().matrix() * V);
  EXPECT(assert_equal(expected, actual));

  Matrix numDGlobal = numericalDerivative21(transformed_from_proxy, start, transform, 1e-5); // At 1e-8, fails
  Matrix numDTrans = numericalDerivative22(transformed_from_proxy, start, transform, 1e-8); // Sensitive to step size
  EXPECT(assert_equal(numDGlobal, actDGlobal));
  EXPECT(assert_equal(numDTrans, actDTrans, 1e-5)); // FIXME: still needs analytic derivative
}

/* ************************************************************************* */
TEST(testPoseRTV, RRTMbn) {
  EXPECT(assert_equal(I_3x3, PoseRTV::RRTMbn(kZero3)));
  EXPECT(assert_equal(I_3x3, PoseRTV::RRTMbn(Rot3())));
  EXPECT(assert_equal(PoseRTV::RRTMbn(Vector3(0.3, 0.2, 0.1)), PoseRTV::RRTMbn(Rot3::Ypr(0.1, 0.2, 0.3))));
}

/* ************************************************************************* */
TEST(testPoseRTV, RRTMnb) {
  EXPECT(assert_equal(I_3x3, PoseRTV::RRTMnb(kZero3)));
  EXPECT(assert_equal(I_3x3, PoseRTV::RRTMnb(Rot3())));
  EXPECT(assert_equal(PoseRTV::RRTMnb(Vector3(0.3, 0.2, 0.1)), PoseRTV::RRTMnb(Rot3::Ypr(0.1, 0.2, 0.3))));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


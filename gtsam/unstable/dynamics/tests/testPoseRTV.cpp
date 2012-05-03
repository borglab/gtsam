/**
 * @file testPoseRTV
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>

#include <gtsam2/dynamics/PoseRTV.h>
#include <gtsam2/dynamics/imu_examples.h>

using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(PoseRTV)
GTSAM_CONCEPT_LIE_INST(PoseRTV)

const double tol=1e-5;

Rot3 rot = Rot3::RzRyRx(0.1, 0.2, 0.3);
Point3 pt(1.0, 2.0, 3.0);
Velocity3 vel(0.4, 0.5, 0.6);

/* ************************************************************************* */
TEST( testPoseRTV, constructors ) {
	PoseRTV state1(pt, rot, vel);
	EXPECT(assert_equal(pt, state1.t(), tol));
	EXPECT(assert_equal(rot, state1.R(), tol));
	EXPECT(assert_equal(vel, state1.v(), tol));
	EXPECT(assert_equal(Pose3(rot, pt), state1.pose(), tol));

	PoseRTV state2;
	EXPECT(assert_equal(Point3(),  state2.t(), tol));
	EXPECT(assert_equal(Rot3(), state2.R(), tol));
	EXPECT(assert_equal(Velocity3(), state2.v(), tol));
	EXPECT(assert_equal(Pose3(), state2.pose(), tol));

	PoseRTV state3(Pose3(rot, pt), vel);
	EXPECT(assert_equal(pt,  state3.t(), tol));
	EXPECT(assert_equal(rot, state3.R(), tol));
	EXPECT(assert_equal(vel, state3.v(), tol));
	EXPECT(assert_equal(Pose3(rot, pt), state3.pose(), tol));

	PoseRTV state4(Pose3(rot, pt));
	EXPECT(assert_equal(pt,  state4.t(), tol));
	EXPECT(assert_equal(rot, state4.R(), tol));
	EXPECT(assert_equal(Velocity3(), state4.v(), tol));
	EXPECT(assert_equal(Pose3(rot, pt), state4.pose(), tol));

	Vector vec_init = Vector_(9, 0.1, 0.2, 0.3,  1.0, 2.0, 3.0,  0.4, 0.5, 0.6);
	PoseRTV state5(vec_init);
	EXPECT(assert_equal(pt,  state5.t(), tol));
	EXPECT(assert_equal(rot, state5.R(), tol));
	EXPECT(assert_equal(vel, state5.v(), tol));
	EXPECT(assert_equal(vec_init, state5.vector(), tol));
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
	EXPECT(assert_equal(state1, state1, tol));
	EXPECT(assert_equal(state2, state3, tol));
	EXPECT(assert_equal(state3, state2, tol));
	EXPECT(assert_inequal(state1, state2, tol));
	EXPECT(assert_inequal(state2, state1, tol));
	EXPECT(assert_inequal(state2, state4, tol));
}

/* ************************************************************************* */
TEST( testPoseRTV, Lie ) {
	// origin and zero deltas
	EXPECT(assert_equal(PoseRTV(), PoseRTV().retract(zero(9)), tol));
	EXPECT(assert_equal(zero(9), PoseRTV().localCoordinates(PoseRTV()), tol));

	PoseRTV state1(pt, rot, vel);
	EXPECT(assert_equal(state1, state1.retract(zero(9)), tol));
	EXPECT(assert_equal(zero(9), state1.localCoordinates(state1), tol));

	Vector delta = Vector_(9, 0.1, 0.1, 0.1, 0.2, 0.3, 0.4,-0.1,-0.2,-0.3);
	Rot3 rot2 = rot.retract(repeat(3, 0.1));
	Point3 pt2 = pt + rot * Point3(0.2, 0.3, 0.4);
	Velocity3 vel2 = vel + rot * Velocity3(-0.1,-0.2,-0.3);
	PoseRTV state2(pt2, rot2, vel2);
	EXPECT(assert_equal(state2, state1.retract(delta), tol));
	EXPECT(assert_equal(delta, state1.localCoordinates(state2), tol));
	EXPECT(assert_equal(-delta, state2.localCoordinates(state1), 1e-1)); // loose tolerance due to retract approximation
}

/* ************************************************************************* */
TEST( testPoseRTV, dynamics_identities ) {
	// general dynamics should produce the same measurements as the imuPrediction function
	PoseRTV x0, x1, x2, x3, x4;

	const double dt = 0.1;
	Vector accel = Vector_(3, 0.2, 0.0, 0.0), gyro = Vector_(3, 0.0, 0.0, 0.2);
	Vector imu01 = zero(6), imu12 = zero(6), imu23 = zero(6), imu34 = zero(6);

	x1 = x0.generalDynamics(accel, gyro, dt);
	x2 = x1.generalDynamics(accel, gyro, dt);
	x3 = x2.generalDynamics(accel, gyro, dt);
	x4 = x3.generalDynamics(accel, gyro, dt);

//	EXPECT(assert_equal(imu01, x0.imuPrediction(x1, dt).first, tol));
//	EXPECT(assert_equal(imu12, x1.imuPrediction(x2, dt).first, tol));
//	EXPECT(assert_equal(imu23, x2.imuPrediction(x3, dt).first, tol));
//	EXPECT(assert_equal(imu34, x3.imuPrediction(x4, dt).first, tol));
//
//	EXPECT(assert_equal(x1.translation(), x0.imuPrediction(x1, dt).second, tol));
//	EXPECT(assert_equal(x2.translation(), x1.imuPrediction(x2, dt).second, tol));
//	EXPECT(assert_equal(x3.translation(), x2.imuPrediction(x3, dt).second, tol));
//	EXPECT(assert_equal(x4.translation(), x3.imuPrediction(x4, dt).second, tol));
}

///* ************************************************************************* */
//TEST( testPoseRTV, constant_velocity ) {
//	double dt = 1.0;
//	PoseRTV  init(Rot3(), Point3(1.0, 2.0, 3.0), Vector_(3, 0.5, 0.0, 0.0));
//	PoseRTV final(Rot3(), Point3(1.5, 2.0, 3.0), Vector_(3, 0.5, 0.0, 0.0));
//
//	// constant velocity, so gyro is zero, but accel includes gravity
//	Vector accel = delta(3, 2, -9.81), gyro = zero(3);
//
//  // perform integration
//  PoseRTV actFinal = init.integrate(accel, gyro, dt);
//  EXPECT(assert_equal(final, actFinal, tol));
//
//  // perform prediction
//  Vector actAccel, actGyro;
//  boost::tie(actAccel, actGyro) = init.predict(final, dt);
//  EXPECT(assert_equal(accel, actAccel, tol));
//  EXPECT(assert_equal(gyro, actGyro, tol));
//}
//
///* ************************************************************************* */
//TEST( testPoseRTV, frame10000_imu ) {
//	using namespace examples;
//
//  // perform integration
//  PoseRTV actFinal = frame10000::init.integrate(frame10000::accel, frame10000::gyro, frame10000::dt);
//  EXPECT(assert_equal(frame10000::final, actFinal, tol));
//
//  // perform prediction
//  Vector actAccel, actGyro;
//  boost::tie(actAccel, actGyro) = frame10000::init.predict(frame10000::final, frame10000::dt);
//  EXPECT(assert_equal(frame10000::accel, actAccel, tol));
//  EXPECT(assert_equal(frame10000::gyro, actGyro, tol));
//}
//
///* ************************************************************************* */
//TEST( testPoseRTV, frame5000_imu ) {
//	using namespace examples;
//
//  // perform integration
//  PoseRTV actFinal = frame5000::init.integrate(frame5000::accel, frame5000::gyro, frame5000::dt);
//  EXPECT(assert_equal(frame5000::final, actFinal, tol));
//
//  // perform prediction
//  Vector actAccel, actGyro;
//  boost::tie(actAccel, actGyro) = frame5000::init.predict(frame5000::final, frame5000::dt);
//  EXPECT(assert_equal(frame5000::accel, actAccel, tol));
//  EXPECT(assert_equal(frame5000::gyro, actGyro, tol));
//}
//
///* ************************************************************************* */
//TEST( testPoseRTV, time4_imu ) {
//	using namespace examples::flying400;
//
//  // perform integration
//  PoseRTV actFinal = init.integrate(accel, gyro, dt);
//  EXPECT(assert_equal(final, actFinal, tol));
//
//  // perform prediction
//  Vector actAccel, actGyro;
//  boost::tie(actAccel, actGyro) = init.predict(final, dt);
//  EXPECT(assert_equal(accel, actAccel, tol));
//  EXPECT(assert_equal(gyro, actGyro, tol));
//}
//
///* ************************************************************************* */
//TEST( testPoseRTV, time65_imu ) {
//	using namespace examples::flying650;
//
//  // perform integration
//  PoseRTV actFinal = init.integrate(accel, gyro, dt);
//  EXPECT(assert_equal(final, actFinal, tol));
//
//  // perform prediction
//  Vector actAccel, actGyro;
//  boost::tie(actAccel, actGyro) = init.predict(final, dt);
//  EXPECT(assert_equal(accel, actAccel, tol));
//  EXPECT(assert_equal(gyro, actGyro, tol));
//}

/* ************************************************************************* */
double range_proxy(const PoseRTV& A, const PoseRTV& B) { return A.range(B); }
TEST( testPoseRTV, range ) {
	Point3 tA(1.0, 2.0, 3.0), tB(3.0, 2.0, 3.0);
	PoseRTV rtvA(tA), rtvB(tB);
	EXPECT_DOUBLES_EQUAL(0.0, rtvA.range(rtvA), tol);
	EXPECT_DOUBLES_EQUAL(2.0, rtvA.range(rtvB), tol);
	EXPECT_DOUBLES_EQUAL(2.0, rtvB.range(rtvA), tol);

	Matrix actH1, actH2;
	rtvA.range(rtvB, actH1, actH2);
	Matrix numericH1 = numericalDerivative21(range_proxy, rtvA, rtvB);
	Matrix numericH2 = numericalDerivative22(range_proxy, rtvA, rtvB);
	EXPECT(assert_equal(numericH1, actH1, tol));
	EXPECT(assert_equal(numericH2, actH2, tol));
}

/* ************************************************************************* */
PoseRTV transformed_from_proxy(const PoseRTV& a, const Pose3& trans) {
	return a.transformed_from(trans);
}
TEST( testPoseRTV, transformed_from_1 ) {
	Rot3 R = Rot3::rodriguez(0.1, 0.2, 0.3);
	Point3 T(1.0, 2.0, 3.0);
	Velocity3 V(2.0, 3.0, 4.0);
	PoseRTV start(R, T, V);
	Pose3 transform(Rot3::yaw(M_PI_2), Point3(1.0, 2.0, 3.0));

	Matrix actDTrans, actDGlobal;
	PoseRTV actual = start.transformed_from(transform, actDGlobal, actDTrans);
	PoseRTV expected(transform.compose(start.pose()), transform.rotation().rotate(V));
	EXPECT(assert_equal(expected, actual, tol));

	Matrix numDGlobal = numericalDerivative21(transformed_from_proxy, start, transform, 1e-8);
	Matrix numDTrans = numericalDerivative22(transformed_from_proxy, start, transform, 1e-8);
	EXPECT(assert_equal(numDGlobal, actDGlobal, tol));
	EXPECT(assert_equal(numDTrans, actDTrans, tol)); // FIXME: still needs analytic derivative
}

/* ************************************************************************* */
TEST( testPoseRTV, transformed_from_2 ) {
	Rot3 R;
	Point3 T(1.0, 2.0, 3.0);
	Velocity3 V(2.0, 3.0, 4.0);
	PoseRTV start(R, T, V);
	Pose3 transform(Rot3::yaw(M_PI_2), Point3(1.0, 2.0, 3.0));

	Matrix actDTrans, actDGlobal;
	PoseRTV actual = start.transformed_from(transform, actDGlobal, actDTrans);
	PoseRTV expected(transform.compose(start.pose()), transform.rotation().rotate(V));
	EXPECT(assert_equal(expected, actual, tol));

	Matrix numDGlobal = numericalDerivative21(transformed_from_proxy, start, transform, 1e-8);
	Matrix numDTrans = numericalDerivative22(transformed_from_proxy, start, transform, 1e-8);
	EXPECT(assert_equal(numDGlobal, actDGlobal, tol));
	EXPECT(assert_equal(numDTrans, actDTrans, tol)); // FIXME: still needs analytic derivative
}

/* ************************************************************************* */
// ground robot maximums
//const static double ground_max_accel = 1.0; // m/s^2
//const static double ground_mag_vel =   5.0; // m/s - fixed in simulator

///* ************************************************************************* */
//TEST(testPoseRTV, flying_integration650) {
//	using namespace examples;
//	const PoseRTV &x1 = flying650::init, &x2 = flying650::final;
//	Vector accel = flying650::accel, gyro = flying650::gyro;
//	double dt = flying650::dt;
//
//	// control inputs
//	double pitch_rate = gyro(1),
//			   heading_rate = gyro(2),
//			   lift_control = 0.0; /// FIXME: need to find this value
//
//	PoseRTV actual_x2;
//	actual_x2 = x1.flyingDynamics(pitch_rate, heading_rate, lift_control, dt);
//
//	// FIXME: enable remaining components when there the lift control value is known
//	EXPECT(assert_equal(x2.R(), actual_x2.R(), tol));
////	EXPECT(assert_equal(x2.t(), actual_x2.t(), tol));
////	EXPECT(assert_equal(x2.v(), actual_x2.v(), tol));
//}

///* ************************************************************************* */
//TEST(testPoseRTV, imu_prediction650) {
//	using namespace examples;
//	const PoseRTV &x1 = flying650::init, &x2 = flying650::final;
//	Vector accel = flying650::accel, gyro = flying650::gyro;
//	double dt = flying650::dt;
//
//	// given states, predict the imu measurement and t2 (velocity constraint)
//	Vector actual_imu;
//	Point3 actual_t2;
//	boost::tie(actual_imu, actual_t2) = x1.imuPrediction(x2, dt);
//
//	EXPECT(assert_equal(x2.t(), actual_t2, tol));
//	EXPECT(assert_equal(accel, actual_imu.head(3), tol));
//	EXPECT(assert_equal(gyro, actual_imu.tail(3), tol));
//}
//
///* ************************************************************************* */
//TEST(testPoseRTV, imu_prediction39) {
//	// This case was a known failure case for gyro prediction, returning [9.39091; 0.204952; 625.63] using
//	// the general approach for reverse-engineering the gyro updates
//	using namespace examples;
//	const PoseRTV &x1 = flying39::init, &x2 = flying39::final;
//	Vector accel = flying39::accel, gyro = flying39::gyro;
//	double dt = flying39::dt;
//
//	// given states, predict the imu measurement and t2 (velocity constraint)
//	Vector actual_imu;
//	Point3 actual_t2;
//	boost::tie(actual_imu, actual_t2) = x1.imuPrediction(x2, dt);
//
//	EXPECT(assert_equal(x2.t(), actual_t2, tol));
//	EXPECT(assert_equal(accel, actual_imu.head(3), tol));
//	EXPECT(assert_equal(gyro, actual_imu.tail(3), tol));
//}
//
///* ************************************************************************* */
//TEST(testPoseRTV, ground_integration200) {
//	using namespace examples;
//	const PoseRTV &x1 = ground200::init, &x2 = ground200::final;
//	Vector accel = ground200::accel, gyro = ground200::gyro;
//	double dt = ground200::dt;
//
//	// integrates from one pose to the next with known measurements
//	// No heading change in this example
//	// Hits maximum accel bound in this example
//
//	PoseRTV actual_x2;
//	actual_x2 = x1.planarDynamics(ground_mag_vel, gyro(2), ground_max_accel, dt);
//
//	EXPECT(assert_equal(x2, actual_x2, tol));
//}
//
///* ************************************************************************* */
//TEST(testPoseRTV, ground_prediction200) {
//	using namespace examples;
//	const PoseRTV &x1 = ground200::init, &x2 = ground200::final;
//	Vector accel = ground200::accel, gyro = ground200::gyro;
//	double dt = ground200::dt;
//
//	// given states, predict the imu measurement and t2 (velocity constraint)
//	Vector actual_imu;
//	Point3 actual_t2;
//	boost::tie(actual_imu, actual_t2) = x1.imuPrediction(x2, dt);
//
//	EXPECT(assert_equal(x2.t(), actual_t2, tol));
//	EXPECT(assert_equal(accel, actual_imu.head(3), tol));
//	EXPECT(assert_equal(gyro, actual_imu.tail(3), tol));
//}
//
///* ************************************************************************* */
//TEST(testPoseRTV, ground_integration600) {
//	using namespace examples;
//	const PoseRTV &x1 = ground600::init, &x2 = ground600::final;
//	Vector accel = ground600::accel, gyro = ground600::gyro;
//	double dt = ground600::dt;
//
//	// integrates from one pose to the next with known measurements
//	PoseRTV actual_x2;
//	actual_x2 = x1.planarDynamics(ground_mag_vel, gyro(2), ground_max_accel, dt);
//
//	EXPECT(assert_equal(x2, actual_x2, tol));
//}
//
///* ************************************************************************* */
//TEST(testPoseRTV, ground_prediction600) {
//	using namespace examples;
//	const PoseRTV &x1 = ground600::init, &x2 = ground600::final;
//	Vector accel = ground600::accel, gyro = ground600::gyro;
//	double dt = ground600::dt;
//
//	// given states, predict the imu measurement and t2 (velocity constraint)
//	Vector actual_imu;
//	Point3 actual_t2;
//	boost::tie(actual_imu, actual_t2) = x1.imuPrediction(x2, dt);
//
//	EXPECT(assert_equal(x2.t(), actual_t2, tol));
//	EXPECT(assert_equal(accel, actual_imu.head(3), tol));
//	EXPECT(assert_equal(gyro, actual_imu.tail(3), tol));
//}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */


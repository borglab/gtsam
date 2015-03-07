/**
 * @file testPoseRotationPrior.cpp
 *
 * @brief Tests rotation priors on poses
 *
 * @date Jun 14, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam_unstable/slam/PoseRotationPrior.h>

using namespace gtsam;

const SharedNoiseModel model1 = noiseModel::Diagonal::Sigmas(Vector_(1, 0.1));
const SharedNoiseModel model3 = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.2, 0.3));

typedef PoseRotationPrior<Pose2> Pose2RotationPrior;
typedef PoseRotationPrior<Pose3> Pose3RotationPrior;

const double tol = 1e-5;

const gtsam::Key poseKey = 1;

// Pose3 examples
const Point3 point3A(1.0, 2.0, 3.0), point3B(4.0, 6.0, 8.0);
const Rot3 rot3A, rot3B = Rot3::pitch(-M_PI_2), rot3C = Rot3::Expmap(Vector_(3, 0.1, 0.2, 0.3));

// Pose2 examples
const Point2 point2A(1.0, 2.0), point2B(4.0, 6.0);
const Rot2 rot2A, rot2B = Rot2::fromAngle(M_PI_2);

/* ************************************************************************* */
LieVector evalFactorError3(const Pose3RotationPrior& factor, const Pose3& x) {
	return LieVector(factor.evaluateError(x));
}

/* ************************************************************************* */
LieVector evalFactorError2(const Pose2RotationPrior& factor, const Pose2& x) {
	return LieVector(factor.evaluateError(x));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level3_zero_error ) {
	Pose3 pose1(rot3A, point3A);
	Pose3RotationPrior factor(poseKey, rot3A, model3);
	Matrix actH1;
	EXPECT(assert_equal(zero(3), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level3_error ) {
	Pose3 pose1(rot3A, point3A);
	Pose3RotationPrior factor(poseKey, rot3C, model3);
	Matrix actH1;
	EXPECT(assert_equal(Vector_(3,-0.1,-0.2,-0.3), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level2_zero_error ) {
	Pose2 pose1(rot2A, point2A);
	Pose2RotationPrior factor(poseKey, rot2A, model1);
	Matrix actH1;
	EXPECT(assert_equal(zero(1), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose2>(
			boost::bind(evalFactorError2, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level2_error ) {
	Pose2 pose1(rot2A, point2A);
	Pose2RotationPrior factor(poseKey, rot2B, model1);
	Matrix actH1;
	EXPECT(assert_equal(Vector_(1,-M_PI_2), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose2>(
			boost::bind(evalFactorError2, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

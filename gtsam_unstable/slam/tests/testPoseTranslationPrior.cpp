/**
 * @file testPoseTranslationPrior.cpp
 *
 * @date Aug 19, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam_unstable/slam/PoseTranslationPrior.h>

using namespace gtsam;

const SharedNoiseModel model2 = noiseModel::Diagonal::Sigmas(Vector_(2, 0.1, 0.2));
const SharedNoiseModel model3 = noiseModel::Diagonal::Sigmas(Vector_(3, 0.1, 0.2, 0.3));

typedef PoseTranslationPrior<Pose2> Pose2TranslationPrior;
typedef PoseTranslationPrior<Pose3> Pose3TranslationPrior;

const double tol = 1e-5;

const gtsam::Key poseKey = 1;

// Pose3 examples
const Point3 point3A(1.0, 2.0, 3.0), point3B(4.0, 6.0, 8.0);
const Rot3 rot3A, rot3B = Rot3::pitch(-M_PI_2), rot3C = Rot3::RzRyRx(0.1, 0.2, 0.3);

// Pose2 examples
const Point2 point2A(1.0, 2.0), point2B(4.0, 6.0);
const Rot2 rot2A, rot2B = Rot2::fromAngle(M_PI_2);

/* ************************************************************************* */
LieVector evalFactorError3(const Pose3TranslationPrior& factor, const Pose3& x) {
	return LieVector(factor.evaluateError(x));
}

/* ************************************************************************* */
LieVector evalFactorError2(const Pose2TranslationPrior& factor, const Pose2& x) {
	return LieVector(factor.evaluateError(x));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, level3_zero_error ) {
	Pose3 pose1(rot3A, point3A);
	Pose3TranslationPrior factor(poseKey, point3A, model3);
	Matrix actH1;
	EXPECT(assert_equal(zero(3), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, level3_error ) {
	Pose3 pose1(rot3A, point3A);
	Pose3TranslationPrior factor(poseKey, point3B, model3);
	Matrix actH1;
	EXPECT(assert_equal(Vector_(3,-3.0,-4.0,-5.0), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, pitched3_zero_error ) {
	Pose3 pose1(rot3B, point3A);
	Pose3TranslationPrior factor(poseKey, point3A, model3);
	Matrix actH1;
	EXPECT(assert_equal(zero(3), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, pitched3_error ) {
	Pose3 pose1(rot3B, point3A);
	Pose3TranslationPrior factor(poseKey, point3B, model3);
	Matrix actH1;
	EXPECT(assert_equal(Vector_(3,-3.0,-4.0,-5.0), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, smallrot3_zero_error ) {
	Pose3 pose1(rot3C, point3A);
	Pose3TranslationPrior factor(poseKey, point3A, model3);
	Matrix actH1;
	EXPECT(assert_equal(zero(3), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, smallrot3_error ) {
	Pose3 pose1(rot3C, point3A);
	Pose3TranslationPrior factor(poseKey, point3B, model3);
	Matrix actH1;
	EXPECT(assert_equal(Vector_(3,-3.0,-4.0,-5.0), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose3>(
			boost::bind(evalFactorError3, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, level2_zero_error ) {
	Pose2 pose1(rot2A, point2A);
	Pose2TranslationPrior factor(poseKey, point2A, model2);
	Matrix actH1;
	EXPECT(assert_equal(zero(2), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose2>(
			boost::bind(evalFactorError2, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseTranslationFactor, level2_error ) {
	Pose2 pose1(rot2A, point2A);
	Pose2TranslationPrior factor(poseKey, point2B, model2);
	Matrix actH1;
	EXPECT(assert_equal(Vector_(2,-3.0,-4.0), factor.evaluateError(pose1, actH1)));
	Matrix expH1 = numericalDerivative11<LieVector,Pose2>(
			boost::bind(evalFactorError2, factor, _1), pose1, 1e-5);
	EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

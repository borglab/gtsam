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

#include <gtsam/slam/PoseRotationPrior.h>

using namespace gtsam;

const SharedNoiseModel model1 = noiseModel::Diagonal::Sigmas((Vector(1) << 0.1).finished());
const SharedNoiseModel model3 = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.2, 0.3));

typedef PoseRotationPrior<Pose2> Pose2RotationPrior;
typedef PoseRotationPrior<Pose3> Pose3RotationPrior;

const double tol = 1e-5;

const gtsam::Key poseKey = 1;

// Pose3 examples
const Point3 point3A(1.0, 2.0, 3.0), point3B(4.0, 6.0, 8.0);
const Rot3 rot3A, rot3B = Rot3::Pitch(-M_PI_2), rot3C = Rot3::Expmap(Vector3(0.1, 0.2, 0.3));

// Pose2 examples
const Point2 point2A(1.0, 2.0), point2B(4.0, 6.0);
const Rot2 rot2A, rot2B = Rot2::fromAngle(M_PI_2);
const Rot2 rot2C = Rot2::fromAngle(M_PI-0.01), rot2D = Rot2::fromAngle(M_PI+0.01);

/* ************************************************************************* */
Vector evalFactorError3(const Pose3RotationPrior& factor, const Pose3& x) {
  return factor.evaluateError(x);
}

/* ************************************************************************* */
Vector evalFactorError2(const Pose2RotationPrior& factor, const Pose2& x) {
  return factor.evaluateError(x);
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level3_zero_error ) {
  Pose3 pose1(rot3A, point3A);
  Pose3RotationPrior factor(poseKey, rot3A, model3);
  Matrix actH1;
  EXPECT(assert_equal(Z_3x1, factor.evaluateError(pose1, actH1)));
  Matrix expH1 = numericalDerivative22<Vector3,Pose3RotationPrior,Pose3>(evalFactorError3, factor, pose1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level3_error ) {
  Pose3 pose1(rot3A, point3A);
  Pose3RotationPrior factor(poseKey, rot3C, model3);
  Matrix actH1;
#if defined(GTSAM_ROT3_EXPMAP) || defined(GTSAM_USE_QUATERNIONS)
  EXPECT(assert_equal(Vector3(-0.1, -0.2,-0.3), factor.evaluateError(pose1, actH1)));
#else
  EXPECT(assert_equal(Vector3(-0.1, -0.2, -0.3), factor.evaluateError(pose1, actH1),1e-2));
#endif
  Matrix expH1 = numericalDerivative22<Vector3,Pose3RotationPrior,Pose3>(evalFactorError3, factor, pose1, 1e-5);
  // the derivative is more complex, but is close to the identity for Rot3 around the origin
  // If not using true expmap will be close, but not exact around the origin
  // EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level2_zero_error ) {
  Pose2 pose1(rot2A, point2A);
  Pose2RotationPrior factor(poseKey, rot2A, model1);
  Matrix actH1;
  EXPECT(assert_equal(Z_1x1, factor.evaluateError(pose1, actH1)));
  Matrix expH1 = numericalDerivative22<Vector1,Pose2RotationPrior,Pose2>(evalFactorError2, factor, pose1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level2_error ) {
  Pose2 pose1(rot2A, point2A);
  Pose2RotationPrior factor(poseKey, rot2B, model1);
  Matrix actH1;
  EXPECT(assert_equal((Vector(1) << -M_PI_2).finished(), factor.evaluateError(pose1, actH1)));
  Matrix expH1 = numericalDerivative22<Vector1,Pose2RotationPrior,Pose2>(evalFactorError2, factor, pose1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
TEST( testPoseRotationFactor, level2_error_wrap ) {
  Pose2 pose1(rot2C, point2A);
  Pose2RotationPrior factor(poseKey, rot2D, model1);
  Matrix actH1;
  EXPECT(assert_equal((Vector(1) << -0.02).finished(), factor.evaluateError(pose1, actH1)));
  Matrix expH1 = numericalDerivative22<Vector1,Pose2RotationPrior,Pose2>(evalFactorError2, factor, pose1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

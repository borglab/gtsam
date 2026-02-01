/**
 * @file testRelativeElevationFactor.cpp
 *
 * @date Aug 17, 2012
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#include <gtsam_unstable/slam/RelativeElevationFactor.h>

#include <gtsam/base/numericalDerivative.h>

using namespace std::placeholders;
using namespace gtsam;

SharedNoiseModel model1 = noiseModel::Unit::Create(1);

const double tol = 1e-5;

const Pose3 pose1(Rot3(), Point3(2.0, 3.0, 4.0));
const Pose3 pose2(Rot3::Pitch(-M_PI_2), Point3(2.0, 3.0, 4.0));
const Pose3 pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(2.0, 3.0, 4.0));
const Point3 point1(3.0, 4.0, 2.0);
const gtsam::Key poseKey = 1, pointKey = 2;

/* ************************************************************************* */
Vector evalFactorError(const RelativeElevationFactor& factor, const Pose3& x, const Point3& p) {
  return factor.evaluateError(x, p);
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, level_zero_error ) {
  // Zero error
  double measured = 2.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal(Z_1x1, factor.evaluateError(pose1, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose1, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose1, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, level_positive ) {
  // Positive meas
  double measured = 0.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal((Vector(1) << 2.0).finished(), factor.evaluateError(pose1, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose1, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose1, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, level_negative ) {
  // Negative meas
  double measured = -1.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal((Vector(1) << 3.0).finished(), factor.evaluateError(pose1, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose1, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose1, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, rotated_zero_error ) {
  // Zero error
  double measured = 2.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal(Z_1x1, factor.evaluateError(pose2, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose2, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose2, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, rotated_positive ) {
  // Positive meas
  double measured = 0.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal((Vector(1) << 2.0).finished(), factor.evaluateError(pose2, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose2, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose2, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, rotated_negative1 ) {
  // Negative meas
  double measured = -1.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal((Vector(1) << 3.0).finished(), factor.evaluateError(pose2, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose2, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose2, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
TEST( testRelativeElevationFactor, rotated_negative2 ) {
  // Negative meas
  double measured = -1.0;
  RelativeElevationFactor factor(poseKey, pointKey, measured, model1);
  Matrix actH1, actH2;
  EXPECT(assert_equal((Vector(1) << 3.0).finished(), factor.evaluateError(pose3, point1, actH1, actH2)));
  Matrix expH1 = numericalDerivative21<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose3, point1, 1e-5);
  Matrix expH2 = numericalDerivative22<Vector, Pose3, Point3>(
      std::bind(evalFactorError, factor, std::placeholders::_1,
                std::placeholders::_2),
      pose3, point1, 1e-5);
  EXPECT(assert_equal(expH1, actH1, tol));
  EXPECT(assert_equal(expH2, actH2, tol));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

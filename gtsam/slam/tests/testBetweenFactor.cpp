/**
 * @file testBetweenFactor.cpp
 * @brief
 * @author Duy-Nguyen Ta, Varun Agrawal
 * @date Aug 2, 2013
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std::placeholders;
using namespace gtsam;
using namespace gtsam::symbol_shorthand;
using namespace gtsam::noiseModel;

/**
 * This TEST should fail. If you want it to pass, change noise to 0.
 */
TEST(BetweenFactor, Rot3) {
  Rot3 R1 = Rot3::Rodrigues(0.1, 0.2, 0.3);
  Rot3 R2 = Rot3::Rodrigues(0.4, 0.5, 0.6);
  Rot3 noise = Rot3(); // Rot3::Rodrigues(0.01, 0.01, 0.01); // Uncomment to make unit test fail
  Rot3 measured = R1.between(R2)*noise  ;

  BetweenFactor<Rot3> factor(R(1), R(2), measured, Isotropic::Sigma(3, 0.05));
  Matrix actualH1, actualH2;
  Vector actual = factor.evaluateError(R1, R2, actualH1, actualH2);

  Vector expected = Rot3::Logmap(measured.inverse() * R1.between(R2));
  EXPECT(assert_equal(expected,actual/*, 1e-100*/)); // Uncomment to make unit test fail

  Matrix numericalH1 = numericalDerivative21<Vector3, Rot3, Rot3>(
      std::function<Vector(const Rot3&, const Rot3&)>(std::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, std::placeholders::_1,
          std::placeholders::_2, boost::none, boost::none)),
      R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH1,actualH1, 1E-5));

  Matrix numericalH2 = numericalDerivative22<Vector3,Rot3,Rot3>(
      std::function<Vector(const Rot3&, const Rot3&)>(std::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, std::placeholders::_1, std::placeholders::_2, boost::none,
          boost::none)), R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH2,actualH2, 1E-5));
}

/* ************************************************************************* */
// Constructor scalar
TEST(BetweenFactor, ConstructorScalar) {
  SharedNoiseModel model;
  double measured = 0.0;
  BetweenFactor<double> factor(1, 2, measured, model);
}

/* ************************************************************************* */
// Constructor vector3
TEST(BetweenFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Vector3 measured(1, 2, 3);
  BetweenFactor<Vector3> factor(1, 2, measured, model);
}

/* ************************************************************************* */
// Constructor dynamic sized vector
TEST(BetweenFactor, ConstructorDynamicSizeVector) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 1.0);
  Vector measured(5); measured << 1, 2, 3, 4, 5;
  BetweenFactor<Vector> factor(1, 2, measured, model);
}

/* ************************************************************************* */
TEST(BetweenFactor, Point3Jacobians) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Point3 measured(1, 2, 3);
  BetweenFactor<Point3> factor(1, 2, measured, model);
  
  Values values;
  values.insert(1, Point3(0, 0, 0));
  values.insert(2, Point3(1, 2, 3));
  Vector3 error = factor.evaluateError(Point3(0, 0, 0), Point3(1, 2, 3));
  EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BetweenFactor, Rot3Jacobians) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Rot3 measured = Rot3::Ry(M_PI_2);
  BetweenFactor<Rot3> factor(1, 2, measured, model);
  
  Values values;
  values.insert(1, Rot3());
  values.insert(2, Rot3::Ry(M_PI_2));
  Vector3 error = factor.evaluateError(Rot3(), Rot3::Ry(M_PI_2));
  EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
TEST(BetweenFactor, Pose3Jacobians) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(6, 1.0);
  Pose3 measured(Rot3(), Point3(1, 2, 3));
  BetweenFactor<Pose3> factor(1, 2, measured, model);

  Pose3 pose1, pose2(Rot3(), Point3(1, 2, 3));
  Values values;
  values.insert(1, pose1);
  values.insert(2, pose2);
  Vector6 error = factor.evaluateError(pose1, pose2);
  EXPECT(assert_equal<Vector6>(Vector6::Zero(), error, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

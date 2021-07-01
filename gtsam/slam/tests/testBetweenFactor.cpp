/**
 * @file    testBetweenFactor.cpp
 * @brief
 * @author Duy-Nguyen Ta
 * @date    Aug 2, 2013
 */

// #define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <CppUnitLite/TestHarness.h>

#include <chrono>

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

  Matrix numericalH1 = numericalDerivative21<Vector3,Rot3,Rot3>(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH1,actualH1, 1E-5));

  Matrix numericalH2 = numericalDerivative22<Vector3,Rot3,Rot3>(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);
  EXPECT(assert_equal(numericalH2,actualH2, 1E-5));
}

/* ************************************************************************* *

// Timing and error evaluation for evaluate error 
 
TEST(BetweenFactor, SlowButCorrectCompare) {
  Rot3 R1 = Rot3::Rodrigues(0.1, 0.2, 0.3);
  Rot3 R2 = Rot3::Rodrigues(0.4, 0.5, 0.6);
  Rot3 noise = Rot3::Rodrigues(0.01, 0.01, 0.01);
  Rot3 measured = R1.between(R2)*noise  ; // Some noisy measurement
  BetweenFactor<Rot3> factor(R(1), R(2), measured, Isotropic::Sigma(3, 0.05));

  Vector3 actual;
  Matrix actualH1 = Eigen::MatrixXd::Zero(3,3);
  Matrix actualH2 = Eigen::MatrixXd::Zero(3,3);
  double duration = 0.0;
  // Running samples to get the timing of evaluateError
  size_t sample_size = 100; 
  for (size_t i = 0; i < sample_size; i++) {
    Matrix H1, H2;
    auto start = std::chrono::high_resolution_clock::now();
    Vector3 error = factor.evaluateError(R1, R2, H1, H2);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    duration = duration + elapsed.count();
    actual = actual + error;
    actualH1 = actualH1 + H1; 
    actualH2 = actualH2 + H2;
  }
  
  actual = actual / sample_size;
  actualH1 = actualH1 / sample_size;
  actualH2 = actualH2 / sample_size;
  duration = duration / sample_size;
  // Comparing calculated Jacobian to numerically obtained Jacobian 
  Vector expected = Rot3::Logmap(measured.inverse() * R1.between(R2));
  Matrix numericalH1 = numericalDerivative21<Vector3,Rot3,Rot3>(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);

  Matrix numericalH2 = numericalDerivative22<Vector3,Rot3,Rot3>(
      boost::function<Vector(const Rot3&, const Rot3&)>(boost::bind(
          &BetweenFactor<Rot3>::evaluateError, factor, _1, _2, boost::none,
          boost::none)), R1, R2, 1e-5);

  std::cout << "measurement error: \n" << (expected - actual).array().abs() << std::endl;
  std::cout << "H1 error: \n" << (numericalH1 - actualH1).array().abs() << std::endl;
  std::cout << "H2 error: \n" << (numericalH2 - actualH2).array().abs() << std::endl;
  std::cout << "Average runtime for evaluaterError: " << duration << std::endl;
}

/* ************************************************************************* */
/*
// Constructor scalar
TEST(BetweenFactor, ConstructorScalar) {
  SharedNoiseModel model;
  double measured_value = 0.0;
  BetweenFactor<double> factor(1, 2, measured_value, model);
}

// Constructor vector3
TEST(BetweenFactor, ConstructorVector3) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(3, 1.0);
  Vector3 measured_value(1, 2, 3);
  BetweenFactor<Vector3> factor(1, 2, measured_value, model);
}

// Constructor dynamic sized vector
TEST(BetweenFactor, ConstructorDynamicSizeVector) {
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 1.0);
  Vector measured_value(5); measured_value << 1, 2, 3, 4, 5;
  BetweenFactor<Vector> factor(1, 2, measured_value, model);
}
*/

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

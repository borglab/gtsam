/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2020, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testAngleTriangulationFactor.cpp
 *  @brief Unit tests for AngleTriangulationFactor Class
 *  @author Varun Agrawal
 *  @date October 2020
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/AngleTriangulationFactor.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// Create a noise model for the ray triangulation
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(2, 1e-5));

// Keys are deliberately *not* in sorted order to test that case.
static const Key kKey1(2), kKey2(1);

auto L1 = AngleTriangulationFactor<Cal3_S2>::MinimizationType::L1;
auto L2 = AngleTriangulationFactor<Cal3_S2>::MinimizationType::L2;
auto LInfinity = AngleTriangulationFactor<Cal3_S2>::MinimizationType::Linfinity;

/* ********************************************************************** */
TEST(AngleTriangulationFactor, Constructor) {
  Cal3_S2 K;

  AngleTriangulationFactor<Cal3_S2> l1_factor(
      kKey1, kKey2, K, Point2(100, 20), Point2(20, 100), model,
      AngleTriangulationFactor<Cal3_S2>::MinimizationType::L1);

  AngleTriangulationFactor<Cal3_S2> l2_factor(
      kKey1, kKey2, K, Point2(100, 20), Point2(20, 100), model,
      AngleTriangulationFactor<Cal3_S2>::MinimizationType::L2);

  AngleTriangulationFactor<Cal3_S2> linf_factor(
      kKey1, kKey2, K, Point2(100, 20), Point2(20, 100), model,
      AngleTriangulationFactor<Cal3_S2>::MinimizationType::Linfinity);
}

/* ********************************************************************** */
TEST(AngleTriangulationFactor, L1ZeroError) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI_4, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI_4, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);

  Point3 landmark(20, 0, 20);

  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create the factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model,
  L1);

  Values values;
  values.insert<Pose3>(kKey1, wTc0);
  values.insert<Pose3>(kKey2, wTc1);
  double error = factor.error(values);

  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);
}

TEST(AngleTriangulationFactor, NanError) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3(0.866025, 0, 0.5, 0, 1, 0, -0.5, 0, 0.866025),
             Point3(0, 0, 0));
  Pose3 wTc1(
      Rot3(0.866025, 0, -0.500009, 0, 1, 0, 0.500009, 0, 0.866025),
      Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);
  Point3 landmark(20, 0, 40 * sin(M_PI / 3));
  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create a factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);
  std::cout << factor.evaluateError(wTc0, wTc1) << std::endl;
}

/* ************************************************************************* */
double vectorAngle(const Vector3& a, const Vector3& b,
                   const AngleTriangulationFactor<Cal3_S2>& factor) {
  return factor.vectorAngle(a, b);
}

TEST(AngleTriangulationFactor, VectorAngle) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI / 3, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, 2 * M_PI / 3, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);
  Point3 landmark(40 * 2 * cos(M_PI / 3), 40 * sin(M_PI / 3), 0);
  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create a factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model,
  L1);

  Vector3 a(1, 2, 3), b(7, 8, 9);

  Matrix H1Actual, H2Actual;
  double theta = factor.vectorAngle(a, b, H1Actual, H2Actual);

  EXPECT_DOUBLES_EQUAL(0.285886797, theta, 1e-7);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected = numericalDerivative11<double, Vector3>(
      std::bind(&vectorAngle, std::placeholders::_1, b, factor), a);

  Matrix H2Expected = numericalDerivative11<double, Vector3>(
      std::bind(&vectorAngle, a, std::placeholders::_1, factor), b);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
}

TEST(AngleTriangulationFactor, VectorAngle2) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI / 6, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI / 6, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);
  Point3 landmark(20, 0, 40 * sin(M_PI / 3));
  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create a factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model,
  L1);

  Vector3 a(-0.866025, 0, 0.5), b(-0.799408, -0.230769, 0.5);

  Matrix H1Actual, H2Actual;
  double theta = factor.vectorAngle(a, b, H1Actual, H2Actual);

  EXPECT_DOUBLES_EQUAL(0.24256363543194706, theta, 1e-7);

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected = numericalDerivative11<double, Vector3>(
      std::bind(&vectorAngle, std::placeholders::_1, b, factor), a);

  Matrix H2Expected = numericalDerivative11<double, Vector3>(
      std::bind(&vectorAngle, a, std::placeholders::_1, factor), b);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ********************************************************************** */
template <typename CAL>
Vector factorError(const Pose3& T1, const Pose3& T2,
                   const AngleTriangulationFactor<CAL>& factor) {
  return factor.evaluateError(T1, T2);
}

TEST(AngleTriangulationFactor, L1Jacobian) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  // Pose3 wTc0(Rot3::Ypr(0.0, M_PI / 3, 0.0), Point3(0, 0, 0));
  // Pose3 wTc1(Rot3::Ypr(0.0, 2 * M_PI / 3, 0.0), Point3(40, 0, 0));
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI / 6, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI / 6, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);
  // Point3 landmark(40 * 2 * cos(M_PI / 3), 40 * sin(M_PI / 3), 0);
  Point3 landmark(20, 0, 40 * sin(M_PI / 3));
  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create a factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);

  // Use the factor to calculate the Jacobians
  Matrix H1Actual, H2Actual;
  Vector2 error = factor.evaluateError(wTc0, wTc1, H1Actual, H2Actual);

  Vector2 expectedError = Vector2::Zero();
  EXPECT(assert_equal(expectedError, error, 1e-9));

  // Use numerical derivatives to calculate the Jacobians
  Matrix H1Expected, H2Expected;
  H1Expected = numericalDerivative11<Vector2, Pose3>(
      std::bind(&factorError<Cal3_S2>, std::placeholders::_1, wTc1, factor),
      wTc0);
  H2Expected = numericalDerivative11<Vector2, Pose3>(
      std::bind(&factorError<Cal3_S2>, wTc0, std::placeholders::_1, factor),
      wTc1);

  // Verify the Jacobians are correct
  EXPECT(assert_equal(H1Expected, H1Actual, 1e-9));
  EXPECT(assert_equal(H2Expected, H2Actual, 1e-9));
}

/* ********************************************************************** */
TEST(AngleTriangulationFactor, L2ZeroError) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI_4, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI_4, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);

  Point3 landmark(20, 0, 20);

  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create the factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model,
  L2);

  Values values;
  values.insert<Pose3>(kKey1, wTc0);
  values.insert<Pose3>(kKey2, wTc1);
  double error = factor.error(values);

  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);
}

/* ********************************************************************** */
TEST(AngleTriangulationFactor, LInfinityZeroError) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI_4, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI_4, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);

  Point3 landmark(20, 0, 20);

  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create the factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model,
                                           LInfinity);

  Values values;
  values.insert<Pose3>(kKey1, wTc0);
  values.insert<Pose3>(kKey2, wTc1);
  double error = factor.error(values);

  EXPECT_DOUBLES_EQUAL(0.0, error, 1e-9);
}

/* ********************************************************************** */
TEST(AngleTriangulationFactor, NonZeroError) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI / 6, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI / 6, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);

  Point3 landmark(20, 0, 40 * sin(M_PI / 3));

  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);
  // create a factor
  SharedNoiseModel model(noiseModel::Isotropic::Sigma(2, 1e-10));
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model,
  L1);

  Pose3 wTc1_noisy = wTc1 * Pose3(Rot3(), Point3(0, 10, 0));

  std::cout << "\n\n\nnon zero error" << std::endl;
  // use the factor to calculate the error
  Vector actualError(factor.evaluateError(wTc0, wTc1_noisy));

  // verify we get the expected error
  Vector2 expected(0.242563874, 0);
  EXPECT(assert_equal(expected, actualError, 1e-8));

  NonlinearFactorGraph graph;
  graph.addPrior<Pose3>(kKey1, wTc0, noiseModel::Isotropic::Sigma(6, 1e-6));
  graph.push_back(factor);

  Values values;
  values.insert<Pose3>(kKey1, wTc0);
  values.insert<Pose3>(kKey2, wTc1_noisy);

  LevenbergMarquardtParams params;
  params.setVerbosity("SILENT");
  params.setVerbosityLM("SILENT");
  LevenbergMarquardtOptimizer optimizer(graph, values, params);
  Values result = optimizer.optimize();
  std::cout << wTc1_noisy << std::endl;
  std::cout << result.at<Pose3>(kKey2) << std::endl;

  std::cout << wTc1 << std::endl;

  PinholeCamera<Cal3_S2> C1_noisy(wTc1_noisy, K);
  PinholeCamera<Cal3_S2> C1_prime(result.at<Pose3>(kKey2), K);

  std::cout << "original: " << u1.transpose() << std::endl;
  std::cout << "noisy: " << C1_noisy.project2(landmark).transpose() << std::endl;
  std::cout << "result: " << C1_prime.project2(landmark).transpose() << std::endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

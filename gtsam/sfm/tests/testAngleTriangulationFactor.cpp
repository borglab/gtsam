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
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/AngleTriangulationFactor.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

// Create a noise model for the ray triangulation
static SharedNoiseModel model(noiseModel::Isotropic::Sigma(2, 1e-5));

static const Key kKey1(1), kKey2(2);

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
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);

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
  Pose3 wTc1(Rot3(0.866025, 0, -0.500009, 0, 1, 0, 0.500009, 0, 0.866025),
             Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);
  Point3 landmark(20, 0, 40 * sin(M_PI / 3));
  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create a factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);
  Vector2 actualError = factor.evaluateError(wTc0, wTc1);
  Vector2 expectedError = Vector2::Zero();
  EXPECT(assert_equal(expectedError, actualError));
}

/* ************************************************************************* */
double vectorAngle(const Vector3& a, const Vector3& b,
                   const AngleTriangulationFactor<Cal3_S2>& factor) {
  return factor.vectorAngle(a, b);
}

/* ************************************************************************* */
TEST(AngleTriangulationFactor, VectorAngle) {
  Cal3_S2 K(500, 500, 0.1, 640 / 2, 480 / 2);
  Pose3 wTc0(Rot3::Ypr(0.0, M_PI_4, 0.0), Point3(0, 0, 0));
  Pose3 wTc1(Rot3::Ypr(0.0, -M_PI_4, 0.0), Point3(40, 0, 0));
  PinholeCamera<Cal3_S2> C0(wTc0, K), C1(wTc1, K);

  Point3 landmark(20, 0, 20);
  Point2 u0 = C0.project2(landmark), u1 = C1.project2(landmark);

  // Create a factor
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);

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
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);

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
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L2);

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
  AngleTriangulationFactor<Cal3_S2> factor(kKey1, kKey2, K, u0, u1, model, L1);

  Pose3 wTc1_noisy = wTc1 * Pose3(Rot3(), Point3(0, 10, 0));

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

  // Final error should be zero
  EXPECT_DOUBLES_EQUAL(0.0, graph.error(result), 1e-9);

  std::cout << wTc1 << std::endl;
  std::cout << wTc1_noisy << std::endl;
  // Result is a rotated matrix (instead of translated) but that's because we
  // only have 2 constraints for 6 unknowns.
  std::cout << result.at<Pose3>(kKey2) << std::endl;

  PinholeCamera<Cal3_S2> C1_noisy(wTc1_noisy, K);
  PinholeCamera<Cal3_S2> C1_prime(result.at<Pose3>(kKey2), K);

  std::cout << "original projection: " << u1.transpose() << std::endl;
  std::cout << "noisy projection: " << C1_noisy.project2(landmark).transpose()
            << std::endl;
  std::cout << "result projection: " << C1_prime.project2(landmark).transpose()
            << std::endl;
}

/* ************************************************************************* */
std::vector<gtsam::Point3> createPoints() {
  // Create the set of ground-truth landmarks
  std::vector<gtsam::Point3> points;
  points.push_back(gtsam::Point3(10.0, 10.0, 10.0));
  points.push_back(gtsam::Point3(-10.0, 10.0, 10.0));
  points.push_back(gtsam::Point3(-10.0, -10.0, 10.0));
  points.push_back(gtsam::Point3(10.0, -10.0, 10.0));
  points.push_back(gtsam::Point3(10.0, 10.0, -10.0));
  points.push_back(gtsam::Point3(-10.0, 10.0, -10.0));
  points.push_back(gtsam::Point3(-10.0, -10.0, -10.0));
  points.push_back(gtsam::Point3(10.0, -10.0, -10.0));

  return points;
}

/* ************************************************************************* */
std::vector<gtsam::Pose3> createPoses(
    const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0,
                                                             -M_PI / 2),
                                            gtsam::Point3(30, 0, 0)),
    const gtsam::Pose3& delta = gtsam::Pose3(
        gtsam::Rot3::Ypr(0, -M_PI / 4, 0),
        gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4)))),
    int steps = 8, ) {
  // Create the set of ground-truth poses
  // Default values give a circular trajectory, radius 30 at pi/4 intervals,
  // always facing the circle center
  std::vector<gtsam::Pose3> poses;
  int i = 1;
  poses.push_back(init);
  for (; i < steps; ++i) {
    poses.push_back(poses[i - 1].compose(delta));
  }

  return poses;
}

TEST(AngleTriangulationFactor, SfmExample) {
  using symbol_shorthand::X;

  // Define the camera calibration parameters
  Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

  // Define the camera observation noise model
  auto measurementNoise =
      noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v

  // Create the set of ground-truth landmarks
  vector<Point3> points = createPoints();

  // Create the set of ground-truth poses
  vector<Pose3> poses = createPoses();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // Add a prior on pose x1. This indirectly specifies where the origin is.
  auto poseNoise = noiseModel::Diagonal::Sigmas(
      (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.0003))
          .finished());  // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  graph.addPrior(X(0), poses[0], poseNoise);  // add directly to graph

  // Simulated measurements from each camera pose, adding them to the factor
  // graph
  Point2 prev_measurement, measurement;
  for (size_t j = 0; j < points.size(); ++j) {
    for (size_t i = 0; i < poses.size(); ++i) {
      PinholeCamera<Cal3_S2> camera(poses[i], *K);
      if (i == 0) {
        prev_measurement = camera.project2(points[j]);
      } else {
        measurement = camera.project2(points[j]);
        graph.emplace_shared<AngleTriangulationFactor<Cal3_S2>>(
            X(i - 1), X(i), *K, prev_measurement, measurement, measurementNoise,
            L1);

        prev_measurement = measurement;
      }
    }
  }

  std::cout << "===================\n\n" << std::endl;

  // Intentionally initialize the variables off from the ground truth
  Values initialEstimate;
  for (size_t i = 0; i < poses.size(); ++i) {
    std::cout << poses[i] << std::endl;
    auto corrupted_pose = poses[i].compose(
        Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20)));
    initialEstimate.insert(X(i), corrupted_pose);
  }
  /* Optimize the graph and print results */
  Values result =
      LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

  // for (size_t i = 0; i < poses.size(); i++) {
  //   std::cout << poses[i] << std::endl;
  // }
  std::cout << "===================\n" << std::endl;
  initialEstimate.print("Initial Estimate:\n");
  std::cout << "===================\n" << std::endl;
  result.print("Final results:\n");

  std::cout << "Initial Error: " << graph.error(initialEstimate) << std::endl;
  std::cout << "  Final Error: " << graph.error(result) << std::endl;
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

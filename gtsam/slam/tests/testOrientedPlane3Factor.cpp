/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testOrientedPlane3Factor.cpp
 * @date Dec 19, 2012
 * @author Alex Trevor
 * @brief Tests the OrientedPlane3Factor class
 */

#include <gtsam/slam/OrientedPlane3Factor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(OrientedPlane3)
GTSAM_CONCEPT_MANIFOLD_INST(OrientedPlane3)

using symbol_shorthand::P;  //< Planes
using symbol_shorthand::X;  //< Pose3

// *************************************************************************
TEST(OrientedPlane3Factor, lm_translation_error) {
  // Tests one pose, two measurements of the landmark that differ in range only.
  // Normal along -x, 3m away
  OrientedPlane3 test_lm0(-1.0, 0.0, 0.0, 3.0);

  NonlinearFactorGraph graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement
  // does not fully constrain the pose
  Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  Vector6 sigmas;
  sigmas << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
  graph.addPrior(X(0), init_pose, noiseModel::Diagonal::Sigmas(sigmas));

  // Add two landmark measurements, differing in range
  Vector3 sigmas3 {0.1, 0.1, 0.1};
  Vector4 measurement0 {-1.0, 0.0, 0.0, 3.0};
  OrientedPlane3Factor factor0(
      measurement0, noiseModel::Diagonal::Sigmas(sigmas3), X(0), P(0));
  graph.add(factor0);
  Vector4 measurement1 {-1.0, 0.0, 0.0, 1.0};
  OrientedPlane3Factor factor1(
      measurement1, noiseModel::Diagonal::Sigmas(sigmas3), X(0), P(0));
  graph.add(factor1);

  // Initial Estimate
  Values values;
  values.insert(X(0), init_pose);
  values.insert(P(0), test_lm0);

  // Optimize
  ISAM2 isam2;
  ISAM2Result result = isam2.update(graph, values);
  Values result_values = isam2.calculateEstimate();
  OrientedPlane3 optimized_plane_landmark =
      result_values.at<OrientedPlane3>(P(0));

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-1.0, 0.0, 0.0, 2.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}

// *************************************************************************
// TODO As described in PR #564 after correcting the derivatives in
// OrientedPlane3Factor this test fails. It should be debugged and re-enabled.
/*
TEST (OrientedPlane3Factor, lm_rotation_error) {
  // Tests one pose, two measurements of the landmark that differ in angle only.
  // Normal along -x, 3m away
  OrientedPlane3 test_lm0(-1.0/sqrt(1.01), 0.1/sqrt(1.01), 0.0, 3.0);

  NonlinearFactorGraph graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement does not fully constrain the pose
  Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  graph.addPrior(X(0), init_pose,
      noiseModel::Diagonal::Sigmas(
          (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished()));

  // Add two landmark measurements, differing in angle
  Vector4 measurement0 {-1.0, 0.0, 0.0, 3.0};
  OrientedPlane3Factor factor0(measurement0,
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1)), X(0), P(0));
  graph.add(factor0);
  Vector4 measurement1 {0.0, -1.0, 0.0, 3.0};
  OrientedPlane3Factor factor1(measurement1,
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1)), X(0), P(0));
  graph.add(factor1);

  // Initial Estimate
  Values values;
  values.insert(X(0), init_pose);
  values.insert(P(0), test_lm0);

  // Optimize
  ISAM2 isam2;
  ISAM2Result result = isam2.update(graph, values);
  Values result_values = isam2.calculateEstimate();
  auto optimized_plane_landmark = result_values.at<OrientedPlane3>(P(0));

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0,
      0.0, 3.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}
*/

// *************************************************************************
TEST( OrientedPlane3Factor, Derivatives ) {
  // Measurement
  OrientedPlane3 p(sqrt(2)/2, -sqrt(2)/2, 0, 5);

  // Linearisation point
  OrientedPlane3 pLin(sqrt(3)/3, -sqrt(3)/3, sqrt(3)/3, 7);
  gtsam::Point3 pointLin  = gtsam::Point3(1, 2, -4);
  gtsam::Rot3 rotationLin = gtsam::Rot3::RzRyRx(0.5*M_PI, -0.3*M_PI, 1.4*M_PI);
  Pose3 poseLin(rotationLin, pointLin);

  // Factor
  Key planeKey(1), poseKey(2);
  SharedGaussian noise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  OrientedPlane3Factor factor(p.planeCoefficients(), noise, poseKey, planeKey);

  // Calculate numerical derivatives
  auto f = [&factor](const Pose3& p, const OrientedPlane3& o) {
    return factor.evaluateError(p, o);
  };
  Matrix numericalH1 = numericalDerivative21<Vector, Pose3, OrientedPlane3>(f, poseLin, pLin);
  Matrix numericalH2 = numericalDerivative22<Vector, Pose3, OrientedPlane3>(f, poseLin, pLin);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2;
  factor.evaluateError(poseLin, pLin, actualH1, actualH2);

  // Verify we get the expected error
  EXPECT(assert_equal(numericalH1, actualH1, 1e-8));
  EXPECT(assert_equal(numericalH2, actualH2, 1e-8));
}

// *************************************************************************
TEST( OrientedPlane3DirectionPrior, Constructor ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)

  Vector4 planeOrientation = (Vector(4) << 0.0, 0.0, -1.0, 10.0).finished(); // all vertical planes directly facing the origin

  // Factor
  Key key(1);
  SharedGaussian model = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 10.0));
  OrientedPlane3DirectionPrior factor(key, planeOrientation, model);

  // Create a linearization point at the zero-error point
  Vector4 theta1 {0.0, 0.02, -1.2, 10.0};
  Vector4 theta2 {0.0, 0.1, -0.8, 10.0};
  Vector4 theta3 {0.0, 0.2, -0.9, 10.0};

  OrientedPlane3 T1(theta1);
  OrientedPlane3 T2(theta2);
  OrientedPlane3 T3(theta3);

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, OrientedPlane3>(
		  [&factor](const OrientedPlane3& o) {return factor.evaluateError(o);}, T1);

  Matrix expectedH2 = numericalDerivative11<Vector, OrientedPlane3>(
      [&factor](const OrientedPlane3& o) {return factor.evaluateError(o);}, T2);

  Matrix expectedH3 = numericalDerivative11<Vector, OrientedPlane3>(
      [&factor](const OrientedPlane3& o) { return factor.evaluateError(o); }, T3);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2, actualH3;
  factor.evaluateError(T1, actualH1);
  factor.evaluateError(T2, actualH2);
  factor.evaluateError(T3, actualH3);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH1, actualH1, 1e-8));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-8));
  EXPECT(assert_equal(expectedH3, actualH3, 1e-8));
}

/* ************************************************************************* */
// Simplified version of the test by Marco Camurri to debug issue #561
TEST(OrientedPlane3Factor, Issue561Simplified) {
  // Typedefs
  using Plane = OrientedPlane3;

  NonlinearFactorGraph graph;

  // Setup prior factors
  // Note: If x0 is too far away from the origin (e.g. x=100) this test can fail.
  Pose3 x0(Rot3::Identity(), Vector3(10, -1, 1));
  auto x0_noise = noiseModel::Isotropic::Sigma(6, 0.01);
  graph.addPrior<Pose3>(X(0), x0, x0_noise);

  // Two horizontal planes with different heights, in the world frame.
  const Plane p1(0,0,1,1), p2(0,0,1,2);
  auto p1_noise = noiseModel::Diagonal::Sigmas(Vector3{1, 1, 5});
  auto p2_noise = noiseModel::Diagonal::Sigmas(Vector3{1, 1, 5});
  graph.addPrior<Plane>(P(1), p1, p1_noise);
  graph.addPrior<Plane>(P(2), p2, p2_noise);

  // Plane factors
  auto p1_measured_from_x0 = p1.transform(x0); // transform p1 to pose x0 as a measurement
  auto p2_measured_from_x0 = p2.transform(x0); // transform p2 to pose x0 as a measurement
  const auto x0_p1_noise = noiseModel::Isotropic::Sigma(3, 0.05);
  const auto x0_p2_noise = noiseModel::Isotropic::Sigma(3, 0.05);
  graph.emplace_shared<OrientedPlane3Factor>(
      p1_measured_from_x0.planeCoefficients(), x0_p1_noise, X(0), P(1));
  graph.emplace_shared<OrientedPlane3Factor>(
      p2_measured_from_x0.planeCoefficients(), x0_p2_noise, X(0), P(2));

  // Initial values
  // Just offset the initial pose by 1m. This is what we are trying to optimize.
  Values initialEstimate;
  Pose3 x0_initial = x0.compose(Pose3(Rot3::Identity(), Vector3(1,0,0)));
  initialEstimate.insert(P(1), p1);
  initialEstimate.insert(P(2), p2);
  initialEstimate.insert(X(0), x0_initial);

  // Optimize
  try {
    GaussNewtonOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    EXPECT_DOUBLES_EQUAL(0, graph.error(result), 0.1);
    EXPECT(x0.equals(result.at<Pose3>(X(0))));
    EXPECT(p1.equals(result.at<Plane>(P(1))));
    EXPECT(p2.equals(result.at<Plane>(P(2))));
  } catch (const IndeterminantLinearSystemException &e) {
    std::cerr << "CAPTURED THE EXCEPTION: " << DefaultKeyFormatter(e.nearbyVariable()) << std::endl;
    EXPECT(false); // fail if this happens
  }
}

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testLocalOrientedPlane3Factor.cpp
 * @date Feb 12, 2021
 * @author David Wisth
 * @brief Tests the LocalOrientedPlane3Factor class
 */

#include <gtsam_unstable/slam/LocalOrientedPlane3Factor.h>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
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
TEST(LocalOrientedPlane3Factor, lm_translation_error) {
  // Tests one pose, two measurements of the landmark that differ in range only.
  // Normal along -x, 3m away
  OrientedPlane3 test_lm0(-1.0, 0.0, 0.0, 3.0);

  NonlinearFactorGraph graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement
  // does not fully constrain the pose
  Pose3 init_pose = Pose3::identity();
  Pose3 anchor_pose = Pose3::identity();
  graph.addPrior(X(0), init_pose, noiseModel::Isotropic::Sigma(6, 0.001));
  graph.addPrior(X(1), anchor_pose, noiseModel::Isotropic::Sigma(6, 0.001));

  // Add two landmark measurements, differing in range
  Vector4 measurement0(-1.0, 0.0, 0.0, 3.0);
  Vector4 measurement1(-1.0, 0.0, 0.0, 1.0);
  LocalOrientedPlane3Factor factor0(
      measurement0, noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(1), P(0));
  LocalOrientedPlane3Factor factor1(
      measurement1, noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(1), P(0));
  graph.add(factor0);
  graph.add(factor1);

  // Initial Estimate
  Values values;
  values.insert(X(0), init_pose);
  values.insert(X(1), anchor_pose);
  values.insert(P(0), test_lm0);

  // Optimize
  ISAM2 isam2;
  isam2.update(graph, values);
  Values result_values = isam2.calculateEstimate();
  auto optimized_plane_landmark = result_values.at<OrientedPlane3>(P(0));

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-1.0, 0.0, 0.0, 2.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}

// *************************************************************************
// TODO As described in PR #564 after correcting the derivatives in
// OrientedPlane3Factor this test fails. It should be debugged and re-enabled.
/*
TEST (LocalOrientedPlane3Factor, lm_rotation_error) {
  // Tests one pose, two measurements of the landmark that differ in angle only.
  // Normal along -x, 3m away
  OrientedPlane3 test_lm0(-1.0/sqrt(1.01), -0.1/sqrt(1.01), 0.0, 3.0);

  NonlinearFactorGraph graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement
  // does not fully constrain the pose
  Pose3 init_pose = Pose3::identity();
  graph.addPrior(X(0), init_pose, noiseModel::Isotropic::Sigma(6, 0.001));

  // Add two landmark measurements, differing in angle
  Vector4 measurement0(-1.0, 0.0, 0.0, 3.0);
  Vector4 measurement1(0.0, -1.0, 0.0, 3.0);
  LocalOrientedPlane3Factor factor0(measurement0,
      noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(0), P(0));
  LocalOrientedPlane3Factor factor1(measurement1,
      noiseModel::Isotropic::Sigma(3, 0.1), X(0), X(0), P(0));
  graph.add(factor0);
  graph.add(factor1);

  // Initial Estimate
  Values values;
  values.insert(X(0), init_pose);
  values.insert(P(0), test_lm0);

  // Optimize
  ISAM2 isam2;
  isam2.update(graph, values);
  Values result_values = isam2.calculateEstimate();
  isam2.getDelta().print();

  auto optimized_plane_landmark = result_values.at<OrientedPlane3>(P(0));

  values.print();
  result_values.print();

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0,
      0.0, 3.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}
*/

// *************************************************************************
TEST(LocalOrientedPlane3Factor, Derivatives) {
  // Measurement
  OrientedPlane3 p(sqrt(2)/2, -sqrt(2)/2, 0, 5);

  // Linearisation point
  OrientedPlane3 pLin(sqrt(3)/3, -sqrt(3)/3, sqrt(3)/3, 7);
  Pose3 poseLin(Rot3::RzRyRx(0.5*M_PI, -0.3*M_PI, 1.4*M_PI), Point3(1, 2, -4));
  Pose3 anchorPoseLin(Rot3::RzRyRx(-0.1*M_PI, 0.2*M_PI, 1.0), Point3(-5, 0, 1));

  // Factor
  Key planeKey(1), poseKey(2), anchorPoseKey(3);
  SharedGaussian noise = noiseModel::Isotropic::Sigma(3, 0.1);
  LocalOrientedPlane3Factor factor(p, noise, poseKey, anchorPoseKey, planeKey);

  // Calculate numerical derivatives
  auto f =
      std::bind(&LocalOrientedPlane3Factor::evaluateError, factor,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, boost::none, boost::none, boost::none);
  Matrix numericalH1 = numericalDerivative31<Vector3, Pose3, Pose3,
    OrientedPlane3>(f, poseLin, anchorPoseLin, pLin);
  Matrix numericalH2 = numericalDerivative32<Vector3, Pose3, Pose3,
    OrientedPlane3>(f, poseLin, anchorPoseLin, pLin);
  Matrix numericalH3 = numericalDerivative33<Vector3, Pose3, Pose3,
    OrientedPlane3>(f, poseLin, anchorPoseLin, pLin);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2, actualH3;
  factor.evaluateError(poseLin, anchorPoseLin, pLin, actualH1, actualH2,
      actualH3);

  // Verify we get the expected error
  EXPECT(assert_equal(numericalH1, actualH1, 1e-8));
  EXPECT(assert_equal(numericalH2, actualH2, 1e-8));
  EXPECT(assert_equal(numericalH3, actualH3, 1e-8));
}


/* ************************************************************************* */
// Simplified version of the test by Marco Camurri to debug issue #561
//
// This test provides an example of how LocalOrientedPlane3Factor works.
// x0 is the current sensor pose, and x1 is the local "anchor pose" - i.e.
// a local linearisation point for the plane. The plane is representated and
// optimized in x1 frame in the optimization. This greatly improves numerical
// stability when the pose is far from the origin.
//
TEST(LocalOrientedPlane3Factor, Issue561Simplified) {
  // Typedefs
  using Plane = OrientedPlane3;

  NonlinearFactorGraph graph;

  // Setup prior factors
  Pose3 x0(Rot3::identity(), Vector3(100, 30, 10));  // the "sensor pose"
  Pose3 x1(Rot3::identity(), Vector3(90, 40,  5) );  // the "anchor pose"

  auto x0_noise = noiseModel::Isotropic::Sigma(6, 0.01);
  auto x1_noise = noiseModel::Isotropic::Sigma(6, 0.01);
  graph.addPrior<Pose3>(X(0), x0, x0_noise);
  graph.addPrior<Pose3>(X(1), x1, x1_noise);

  // Two horizontal planes with different heights, in the world frame.
  const Plane p1(0, 0, 1, 1), p2(0, 0, 1, 2);
  // Transform to x1, the "anchor frame" (i.e. local frame)
  auto p1_in_x1 = p1.transform(x1);
  auto p2_in_x1 = p2.transform(x1);
  auto p1_noise = noiseModel::Diagonal::Sigmas(Vector3{1, 1, 5});
  auto p2_noise = noiseModel::Diagonal::Sigmas(Vector3{1, 1, 5});
  graph.addPrior<Plane>(P(1), p1_in_x1, p1_noise);
  graph.addPrior<Plane>(P(2), p2_in_x1, p2_noise);

  // Add plane factors, with a local linearization point.
  // transform p1 to pose x0 as a measurement
  auto p1_measured_from_x0 = p1.transform(x0);
  // transform p2 to pose x0 as a measurement
  auto p2_measured_from_x0 = p2.transform(x0);
  const auto x0_p1_noise = noiseModel::Isotropic::Sigma(3, 0.05);
  const auto x0_p2_noise = noiseModel::Isotropic::Sigma(3, 0.05);
  graph.emplace_shared<LocalOrientedPlane3Factor>(
      p1_measured_from_x0.planeCoefficients(), x0_p1_noise, X(0), X(1), P(1));
  graph.emplace_shared<LocalOrientedPlane3Factor>(
      p2_measured_from_x0.planeCoefficients(), x0_p2_noise, X(0), X(1), P(2));

  // Initial values
  // Just offset the initial pose by 1m. This is what we are trying to optimize.
  Values initialEstimate;
  Pose3 x0_initial = x0.compose(Pose3(Rot3::identity(), Vector3(1, 0, 0)));
  initialEstimate.insert(P(1), p1_in_x1);
  initialEstimate.insert(P(2), p2_in_x1);
  initialEstimate.insert(X(0), x0_initial);
  initialEstimate.insert(X(1), x1);

  // Optimize
  try {
    ISAM2 isam2;
    isam2.update(graph, initialEstimate);
    Values result = isam2.calculateEstimate();
    EXPECT_DOUBLES_EQUAL(0, graph.error(result), 0.1);
    EXPECT(x0.equals(result.at<Pose3>(X(0))));
    EXPECT(p1_in_x1.equals(result.at<Plane>(P(1))));
    EXPECT(p2_in_x1.equals(result.at<Plane>(P(2))));
  } catch (const IndeterminantLinearSystemException &e) {
    cerr << "CAPTURED THE EXCEPTION: "
      << DefaultKeyFormatter(e.nearbyVariable()) << endl;
    EXPECT(false);  // fail if this happens
  }
}

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

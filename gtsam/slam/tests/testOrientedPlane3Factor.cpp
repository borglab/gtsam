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

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std.hpp>
#include <boost/bind.hpp>

using namespace boost::assign;
using namespace gtsam;
using namespace std;

GTSAM_CONCEPT_TESTABLE_INST(OrientedPlane3)
GTSAM_CONCEPT_MANIFOLD_INST(OrientedPlane3)

// *************************************************************************
TEST (OrientedPlane3Factor, lm_translation_error) {
  // Tests one pose, two measurements of the landmark that differ in range only.
  // Normal along -x, 3m away
  Symbol lm_sym('p', 0);
  OrientedPlane3 test_lm0(-1.0, 0.0, 0.0, 3.0);

  ISAM2 isam2;
  Values new_values;
  NonlinearFactorGraph new_graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement does not fully constrain the pose
  Symbol init_sym('x', 0);
  Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  Vector sigmas(6);
  sigmas << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
  new_graph.addPrior(
      init_sym, init_pose, noiseModel::Diagonal::Sigmas(sigmas));
  new_values.insert(init_sym, init_pose);

  // Add two landmark measurements, differing in range
  new_values.insert(lm_sym, test_lm0);
  Vector sigmas3(3);
  sigmas3 << 0.1, 0.1, 0.1;
  Vector test_meas0_mean(4);
  test_meas0_mean << -1.0, 0.0, 0.0, 3.0;
  OrientedPlane3Factor test_meas0(test_meas0_mean,
      noiseModel::Diagonal::Sigmas(sigmas3), init_sym, lm_sym);
  new_graph.add(test_meas0);
  Vector test_meas1_mean(4);
  test_meas1_mean << -1.0, 0.0, 0.0, 1.0;
  OrientedPlane3Factor test_meas1(test_meas1_mean,
      noiseModel::Diagonal::Sigmas(sigmas3), init_sym, lm_sym);
  new_graph.add(test_meas1);

  // Optimize
  ISAM2Result result = isam2.update(new_graph, new_values);
  Values result_values = isam2.calculateEstimate();
  OrientedPlane3 optimized_plane_landmark = result_values.at<OrientedPlane3>(
      lm_sym);

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-1.0, 0.0, 0.0, 2.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}

// *************************************************************************
TEST (OrientedPlane3Factor, lm_rotation_error) {
  // Tests one pose, two measurements of the landmark that differ in angle only.
  // Normal along -x, 3m away
  Symbol lm_sym('p', 0);
  OrientedPlane3 test_lm0(-1.0, 0.0, 0.0, 3.0);

  ISAM2 isam2;
  Values new_values;
  NonlinearFactorGraph new_graph;

  // Init pose and prior.  Pose Prior is needed since a single plane measurement does not fully constrain the pose
  Symbol init_sym('x', 0);
  Pose3 init_pose(Rot3::Ypr(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  new_graph.addPrior(init_sym, init_pose,
      noiseModel::Diagonal::Sigmas(
          (Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001).finished()));
  new_values.insert(init_sym, init_pose);

//  // Add two landmark measurements, differing in angle
  new_values.insert(lm_sym, test_lm0);
  Vector test_meas0_mean(4);
  test_meas0_mean << -1.0, 0.0, 0.0, 3.0;
  OrientedPlane3Factor test_meas0(test_meas0_mean,
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1)), init_sym, lm_sym);
  new_graph.add(test_meas0);
  Vector test_meas1_mean(4);
  test_meas1_mean << 0.0, -1.0, 0.0, 3.0;
  OrientedPlane3Factor test_meas1(test_meas1_mean,
      noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1)), init_sym, lm_sym);
  new_graph.add(test_meas1);

  // Optimize
  ISAM2Result result = isam2.update(new_graph, new_values);
  Values result_values = isam2.calculateEstimate();
  OrientedPlane3 optimized_plane_landmark = result_values.at<OrientedPlane3>(
      lm_sym);

  // Given two noisy measurements of equal weight, expect result between the two
  OrientedPlane3 expected_plane_landmark(-sqrt(2.0) / 2.0, -sqrt(2.0) / 2.0,
      0.0, 3.0);
  EXPECT(assert_equal(optimized_plane_landmark, expected_plane_landmark));
}

// *************************************************************************
TEST( OrientedPlane3DirectionPrior, Constructor ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)

  Vector planeOrientation = (Vector(4) << 0.0, 0.0, -1.0, 10.0).finished(); // all vertical planes directly facing the origin

  // Factor
  Key key(1);
  SharedGaussian model = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 10.0));
  OrientedPlane3DirectionPrior factor(key, planeOrientation, model);

  // Create a linearization point at the zero-error point
  Vector theta1 = Vector4(0.0, 0.02, -1.2, 10.0);
  Vector theta2 = Vector4(0.0, 0.1, -0.8, 10.0);
  Vector theta3 = Vector4(0.0, 0.2, -0.9, 10.0);

  OrientedPlane3 T1(theta1);
  OrientedPlane3 T2(theta2);
  OrientedPlane3 T3(theta3);

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector, OrientedPlane3>(
      boost::bind(&OrientedPlane3DirectionPrior::evaluateError, &factor, _1,
          boost::none), T1);

  Matrix expectedH2 = numericalDerivative11<Vector, OrientedPlane3>(
      boost::bind(&OrientedPlane3DirectionPrior::evaluateError, &factor, _1,
          boost::none), T2);

  Matrix expectedH3 = numericalDerivative11<Vector, OrientedPlane3>(
      boost::bind(&OrientedPlane3DirectionPrior::evaluateError, &factor, _1,
          boost::none), T3);

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
// Test by Marco Camurri to debug issue #561
TEST(OrientedPlane3Factor, Issue561) {
  // Typedefs
  using symbol_shorthand::P;  //< Planes
  using symbol_shorthand::X;  //< Pose3 (x,y,z,r,p,y)
  using Plane = OrientedPlane3;

  NonlinearFactorGraph graph;

  // Setup prior factors
  Pose3 x0_prior(
      Rot3(0.799903913, -0.564527097, 0.203624376, 0.552537226, 0.82520195,
           0.117236322, -0.234214312, 0.0187322547, 0.972004505),
      Vector3{-91.7500013, -0.47569366, -2.2});
  auto x0_noise = noiseModel::Isotropic::Sigma(6, 0.01);
  graph.addPrior<Pose3>(X(0), x0_prior, x0_noise);

//   Plane p1_prior(0.211098835, 0.214292752, 0.95368543, 26.4269514);
//   auto p1_noise =
//       noiseModel::Diagonal::Sigmas(Vector3{0.785398163, 0.785398163, 5});
//   graph.addPrior<Plane>(P(1), p1_prior, p1_noise);

// ADDING THIS PRIOR MAKES OPTIMIZATION FAIL
//   Plane p2_prior(0.301901811, 0.151751467, 0.941183717, 33.4388229);
//   auto p2_noise =
//       noiseModel::Diagonal::Sigmas(Vector3{0.785398163, 0.785398163, 5});
//   graph.addPrior<Plane>(P(2), p2_prior, p2_noise);

  // First plane factor
  Plane p1_meas = Plane(0.0638967294, 0.0755284627, 0.995094297, 2.55956073);
  const auto x0_p1_noise = noiseModel::Isotropic::Sigma(3, 0.0451801);
  graph.emplace_shared<OrientedPlane3Factor>(p1_meas.planeCoefficients(),
                                             x0_p1_noise, X(0), P(1));

  // Second plane factor
  Plane p2_meas = Plane(0.104902077, -0.0275756528, 0.994100165, 1.32765088);
  const auto x0_p2_noise = noiseModel::Isotropic::Sigma(3, 0.0322889);
  graph.emplace_shared<OrientedPlane3Factor>(p2_meas.planeCoefficients(),
                                             x0_p2_noise, X(0), P(2));

  // Optimize
  try {
    // Initial values
    Values initialEstimate;
    Plane p1(0.211098835, 0.214292752, 0.95368543, 26.4269514);
    Plane p2(0.301901811, 0.151751467, 0.941183717, 33.4388229);
    Pose3 x0(
        Rot3(0.799903913, -0.564527097, 0.203624376, 0.552537226, 0.82520195,
             0.117236322, -0.234214312, 0.0187322547, 0.972004505),
        Vector3{-91.7500013, -0.47569366, -2.2});
    initialEstimate.insert(P(1), p1);
    initialEstimate.insert(P(2), p2);
    initialEstimate.insert(X(0), x0);

    GaussNewtonParams params;
    GTSAM_PRINT(graph);
    Ordering ordering = list_of(P(1))(P(2))(X(0));  // make sure P1 eliminated first
    params.setOrdering(ordering);
    params.setLinearSolverType("SEQUENTIAL_QR");  // abundance of caution
    params.setVerbosity("TERMINATION");  // show info about stopping conditions
    GaussNewtonOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    EXPECT_DOUBLES_EQUAL(0, graph.error(result), 0.1);
  } catch (IndeterminantLinearSystemException e) {
    std::cerr << "CAPTURED THE EXCEPTION: " << e.nearbyVariable() << std::endl;
  }
}

/* ************************************************************************* */
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

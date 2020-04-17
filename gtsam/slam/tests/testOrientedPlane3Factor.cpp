/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file testOrientedPlane3.cpp
 * @date Dec 19, 2012
 * @author Alex Trevor
 * @brief Tests the OrientedPlane3Factor class
 */

#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>

#include <CppUnitLite/TestHarness.h>
#include <boost/bind.hpp>
#include <boost/assign/std/vector.hpp>

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
int main() {
  srand(time(nullptr));
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

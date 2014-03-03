/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testAttitudeFactor.cpp
 * @brief   Unit test for Rot3AttitudeFactor
 * @author  Frank Dellaert
 * @date   January 22, 2014
 */

#include <gtsam/navigation/AttitudeFactor.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( Rot3AttitudeFactor, Constructor ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)
  Unit3 bZ(0, 0, 1); // reference direction is body Z axis
  Unit3 nDown(0, 0, -1); // down, in ENU navigation frame, is "measurement"

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Rot3AttitudeFactor factor0(key, nDown, model);
  Rot3AttitudeFactor factor(key, nDown, model, bZ);
  EXPECT(assert_equal(factor0,factor,1e-5));

  // Create a linearization point at the zero-error point
  Rot3 nRb;
  EXPECT(assert_equal(zero(2),factor.evaluateError(nRb),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Rot3>(
      boost::bind(&Rot3AttitudeFactor::evaluateError, &factor, _1, boost::none),
      nRb);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(nRb, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3AttitudeFactor, Constructor ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)
  Unit3 bZ(0, 0, 1); // reference direction is body Z axis
  Unit3 nDown(0, 0, -1); // down, in ENU navigation frame, is "measurement"

  // Factor
  Key key(1);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Pose3AttitudeFactor factor0(key, nDown, model);
  Pose3AttitudeFactor factor(key, nDown, model, bZ);
  EXPECT(assert_equal(factor0,factor,1e-5));

  // Create a linearization point at the zero-error point
  Pose3 T(Rot3(), Point3(-5.0, 8.0, -11.0));
  EXPECT(assert_equal(zero(2),factor.evaluateError(T),1e-5));

  // Calculate numerical derivatives
  Matrix expectedH = numericalDerivative11<Pose3>(
      boost::bind(&Pose3AttitudeFactor::evaluateError, &factor, _1,
          boost::none), T);

  // Use the factor to calculate the derivative
  Matrix actualH;
  factor.evaluateError(T, actualH);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH, actualH, 1e-8));
}

/* ************************************************************************* */
TEST( Pose3AttitudeFactor, optimize ) {

  // Example: pitch and roll of aircraft in an ENU Cartesian frame.
  // If pitch and roll are zero for an aerospace frame,
  // that means Z is pointing down, i.e., direction of Z = (0,0,-1)
  Unit3 bZ(0, 0, 1); // reference direction is body Z axis
  Unit3 nDown(0, 0, 1); // down, in ENU navigation frame, is "measurement"

  SharedNoiseModel model = noiseModel::Isotropic::Sigma(2, 0.25);
  Pose3AttitudeFactor factor(0, nDown, model, bZ);

  // Create a linearization point at the zero-error point
  Pose3 Ptrue(Rot3(), Point3(-5.0, 8.0, -11.0));
  Pose3 noisePose = Pose3(Rot3::rodriguez(0.5, 0.5, 0.3),Point3(1,-1,1));
  Pose3 P = Ptrue.compose(noisePose);

  NonlinearFactorGraph graph;
  graph.push_back(factor);

  Values initial;
  initial.insert(0,P);

  LevenbergMarquardtParams params;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  Values actual = lm.optimize();

  Rot3 actualRotation = actual.at<Pose3>(0).rotation();
  Vector actualRotationAxis = Rot3::Logmap(actualRotation);
  actualRotationAxis = actualRotationAxis / actualRotationAxis.norm();
  Vector expectedRotationAxis(3); expectedRotationAxis << 0, 0, 1; // solution has to be a pure rotation around the z axis

  double actualError = graph.error(actual);
  double expectedError = 0.0;

  EXPECT(assert_equal(expectedRotationAxis,actualRotationAxis,1e-4));
  DOUBLES_EQUAL(expectedError, actualError, 1e-4);
}

// *************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *************************************************************************

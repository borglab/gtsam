/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testEssentialMatrixConstraint.cpp
 *  @brief Unit tests for EssentialMatrixConstraint Class
 *  @author Frank Dellaert
 *  @author Pablo Alcantarilla
 *  @date Jan 5, 2014
 */

#include <gtsam/slam/EssentialMatrixConstraint.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( EssentialMatrixConstraint, test ) {
  // Create a factor
  Key poseKey1(1);
  Key poseKey2(2);
  Rot3 trueRotation = Rot3::RzRyRx(0.15, 0.15, -0.20);
  Point3 trueTranslation(+0.5, -1.0, +1.0);
  Unit3 trueDirection(trueTranslation);
  EssentialMatrix measurement(trueRotation, trueDirection);
  SharedNoiseModel model = noiseModel::Isotropic::Sigma(5, 0.25);
  EssentialMatrixConstraint factor(poseKey1, poseKey2, measurement, model);

  // Create a linearization point at the zero-error point
  Pose3 pose1(Rot3::RzRyRx(0.00, -0.15, 0.30), Point3(-4.0, 7.0, -10.0));
  Pose3 pose2(
      Rot3::RzRyRx(0.179693265735950, 0.002945368776519, 0.102274823253840),
      Point3(-3.37493895, 6.14660244, -8.93650986));

  Vector expected = zero(5);
  Vector actual = factor.evaluateError(pose1,pose2);
  CHECK(assert_equal(expected, actual, 1e-8));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, _1, pose2,
          boost::none, boost::none), pose1);
  Matrix expectedH2 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, pose1, _1,
          boost::none, boost::none), pose2);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  Matrix actualH2;
  factor.evaluateError(pose1, pose2, actualH1, actualH2);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-9));
  CHECK(assert_equal(expectedH2, actualH2, 1e-9));
}

/* ************************************************************************* */
TEST(EssentialMatrixConstraint, optimization) {
  Pose3 P1 = Pose3();
  Pose3 P2 = Pose3(Rot3(),Point3(1,1,1));

  NonlinearFactorGraph graph;

  // prior
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Isotropic::Sigma(6, 1.0);
  graph.push_back(PriorFactor<Pose3>(0, P1, priorNoise)); // add directly to graph

  // scaleless between factor
  Unit3 relDirection = Unit3(1,1,1);
  Rot3 relRot = Rot3();
  EssentialMatrix E(relRot,relDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);
  EssentialMatrixConstraint factor(0, 1, E, noise);
  graph.push_back(factor);

  Values expected;
  expected.insert(0,P1);
  expected.insert(1,P2);

  Values initial;
  initial.insert(0,P1);
  Pose3 noisePose = Pose3(Rot3::rodriguez(0.5, 0.5, 0.3),Point3(1,-1,1));
  initial.insert(1,P2.compose(noisePose));

  LevenbergMarquardtParams params;
  params.relativeErrorTol = 0.0;
  params.absoluteErrorTol = 0.0;
  //params.setVerbosityLM("TRYDELTA");
  //params.setVerbosity("DELTA");
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  Values actual = lm.optimize();
  EXPECT(assert_equal(expected,actual,0.1));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


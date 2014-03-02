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
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <cmath>

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

EssentialMatrix EssentialFrom2Poses(const Pose3& p1, const Pose3& p2)
{
  Pose3 _1P2_ = p1.between(p2);
  return EssentialMatrix::FromPose3(_1P2_);
}

/* ************************************************************************* */
TEST(EssentialMatrixConstraint, optimization) {
  Pose3 P1true = Pose3();
  Pose3 P2true = Pose3(Rot3(),Point3(1,1,1));

  //Pose3 noisePose = Pose3(Rot3::rodriguez(0.5, 0.5, 0.3),Point3(1,-1,1));
  Pose3 noisePose = Pose3(Rot3::rodriguez(0.0, 0.0, 0.0),Point3(0.01,-0.01,0.01));
  Pose3 P1(P1true);
  Pose3 P2 = P2true.compose(noisePose);

  // test between
  Pose3 expectedBetween = P1.inverse() * P2;
  Matrix actualDBetween1,actualDBetween2;
  Pose3 actualBetween = P1.between(P2, actualDBetween1,actualDBetween2);
  EXPECT(assert_equal(expectedBetween,actualBetween));

  Matrix numericalH1 = numericalDerivative21(testing::between<Pose3> , P1, P2);
  EXPECT(assert_equal(numericalH1,actualDBetween1,5e-3));
  Matrix numericalH2 = numericalDerivative22(testing::between<Pose3> , P1, P2);
  EXPECT(assert_equal(numericalH2,actualDBetween2,1e-5));

  // test fromPose3
  Matrix actualH;
  EssentialMatrix hx = EssentialMatrix::FromPose3(expectedBetween, actualH);
  Matrix expectedH = numericalDerivative11<EssentialMatrix, Pose3>(
      boost::bind(EssentialMatrix::FromPose3, _1, boost::none), expectedBetween);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));

  // check chain rule:
  Matrix expectedHp1 = numericalDerivative11<EssentialMatrix, Pose3>(
        boost::bind(&EssentialFrom2Poses, _1, P2), P1);
  Matrix expectedHp2 = numericalDerivative11<EssentialMatrix, Pose3>(
        boost::bind(&EssentialFrom2Poses, P1, _1), P2);
  EXPECT(assert_equal(expectedHp1, actualH * actualDBetween1, 1e-7));
  EXPECT(assert_equal(expectedHp2, actualH * actualDBetween2, 1e-7));

  // EssentialMatrixConstraint
  Pose3 betweenTrue = P1true.between(P2true);
  Unit3 expectedDirection = Unit3(betweenTrue.translation());
  Rot3 expectedRotation = betweenTrue.rotation();

  EssentialMatrix E(expectedRotation,expectedDirection);
  noiseModel::Isotropic::shared_ptr noise = noiseModel::Isotropic::Sigma(5, 1.0);
  EssentialMatrixConstraint factor(1, 2, E, noise);

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, _1, P2,
          boost::none, boost::none), P1);
  Matrix expectedH2 = numericalDerivative11<Pose3>(
      boost::bind(&EssentialMatrixConstraint::evaluateError, &factor, P1, _1,
          boost::none, boost::none), P2);

  // Use the factor to calculate the derivative
  Matrix actualH1, actualH2;
  factor.evaluateError(P1, P2, actualH1, actualH2);

  // Verify we get the expected error
  EXPECT(assert_equal(expectedH1, actualH1, 1e-9));
  EXPECT(assert_equal(expectedH2, actualH2, 1e-9));

  // OPTIMIZATION
  NonlinearFactorGraph graph;
  graph.push_back(factor);

  // prior
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Isotropic::Sigma(6, 1.0);
  graph.push_back(PriorFactor<Pose3>(1, P1, priorNoise));

  Values initial;
  initial.insert(1,P1);
  initial.insert(2,P2);

  LevenbergMarquardtParams params;
  //params.relativeErrorTol = 0.0;
  //params.absoluteErrorTol = 0.0;
  //params.setVerbosityLM("TRYDELTA");
  //params.setVerbosity("DELTA");
  LevenbergMarquardtOptimizer lm(graph, initial, params);
  Values actual = lm.optimize();

  Rot3 actualRotation = actual.at<Pose3>(2).rotation();
  Unit3 actualDirection(actual.at<Pose3>(2).translation());

  EXPECT(assert_equal(P1true,actual.at<Pose3>(1),1e-4));
  EXPECT(assert_equal(expectedRotation,actualRotation,1e-4));
  EXPECT(assert_equal(expectedDirection,actualDirection,1e-4));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


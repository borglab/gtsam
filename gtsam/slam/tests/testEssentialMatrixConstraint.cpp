/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  TestEssentialMatrixConstraint.cpp
 *  @brief Unit tests for EssentialMatrixConstraint Class
 *  @author Frank Dellaert
 *  @author Pablo Alcantarilla
 *  @date Jan 5, 2014
 */

#include <gtsam/slam/EssentialMatrixConstraint.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace std::placeholders;
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

  Vector expected = Z_5x1;
  Vector actual = factor.evaluateError(pose1,pose2);
  CHECK(assert_equal(expected, actual, 1e-8));

  // Calculate numerical derivatives
  Matrix expectedH1 = numericalDerivative11<Vector5, Pose3>(
      std::bind(&EssentialMatrixConstraint::evaluateError, &factor,
                std::placeholders::_1, pose2, boost::none, boost::none),
      pose1);
  Matrix expectedH2 = numericalDerivative11<Vector5, Pose3>(
      std::bind(&EssentialMatrixConstraint::evaluateError, &factor, pose1,
                std::placeholders::_1, boost::none, boost::none),
      pose2);

  // Use the factor to calculate the derivative
  Matrix actualH1;
  Matrix actualH2;
  factor.evaluateError(pose1, pose2, actualH1, actualH2);

  // Verify we get the expected error
  CHECK(assert_equal(expectedH1, actualH1, 1e-5));
  CHECK(assert_equal(expectedH2, actualH2, 1e-5));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */


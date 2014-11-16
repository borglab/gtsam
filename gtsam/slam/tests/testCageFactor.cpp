/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testCageFactor.cpp
 * @brief  Unit tests CageFactor class
 * @author Krunal Chande
 */


#include <CppUnitLite/TestHarness.h>
#include <gtsam/slam/CageFactor.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <boost/bind.hpp>

using namespace std;
using namespace gtsam;

// Create a noise model
static SharedNoiseModel model(noiseModel::Unit::Create(6));

LieVector factorError(const Pose3& pose, const CageFactor& factor) {
  return factor.evaluateError(pose);
}


/* ************************************************************************* */
TEST(CageFactor, Inside) {
  Key poseKey(1);
  Pose3 pose(Rot3::ypr(0,0,0),Point3(0,0,0));
  double cageBoundary = 10; // in m
  CageFactor factor(poseKey, pose, cageBoundary, model);

  // Set the linearization point
  Pose3 poseLin;
  Matrix H;
  Vector actualError(factor.evaluateError(poseLin, H));
  Vector expectedError = zero(1);
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // use numerical derivatives to calculate the jacobians
  Matrix HExpected;
  HExpected = numericalDerivative11<Pose3>(boost::bind(&factorError, _1, factor), pose);
  CHECK(assert_equal(HExpected, H, 1e-9));
}

/* ************************************************************************* */
TEST(CageFactor, Outside) {
  Key poseKey(1);
  Point3 translation = Point3(15,0,0);
  Pose3 pose(Rot3::ypr(0,0,0),translation);
  double cageBoundary = 10; // in m
  CageFactor factor(poseKey, pose, cageBoundary, model);

  // Set the linearization point
  Pose3 poseLin;
  Matrix H;
  Vector actualError(factor.evaluateError(pose, H));
  Vector expectedError(Vector(1)<<5);
  CHECK(assert_equal(expectedError, actualError, 1e-9));

  // use numerical derivatives to calculate the jacobians
  Matrix HExpected;
  HExpected = numericalDerivative11<Pose3>(boost::bind(&factorError, _1, factor), pose);
  CHECK(assert_equal(HExpected, H, 1e-9));
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */

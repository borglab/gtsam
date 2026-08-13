/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExtendedPriorFactor.cpp
 * @date September 30, 2025
 * @author Frank Dellaert
 * @brief unit tests for ExtendedPriorFactor factor
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/nonlinear/ExtendedPriorFactor.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
TEST(ExtendedPriorFactor, Constructor) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  auto model = noiseModel::Isotropic::Sigma(3, 0.5);
  ExtendedPriorFactor<Pose2> factor(key, origin, model);
}

//******************************************************************************
TEST(ExtendedPriorFactor, ConstructorWithMean) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  auto model = noiseModel::Isotropic::Sigma(3, 0.5);
  Vector mean{{0.1, 0.2, 0.3}};
  ExtendedPriorFactor<Pose2> factor(key, origin, mean, model);
}

//******************************************************************************
TEST(ExtendedPriorFactor, Error) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  auto model = noiseModel::Isotropic::Sigma(3, 0.5);
  ExtendedPriorFactor<Pose2> factor(key, origin, model);

  Pose2 x(1.1, 2.2, 0.3);
  Vector expected_error{{0.15463769, 0.161515277, 0.0}};
  Vector actual_error = factor.evaluateError(x);
  EXPECT(assert_equal(expected_error, actual_error, 1e-8));
}

//******************************************************************************
TEST(ExtendedPriorFactor, ErrorWithMean) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  auto model = noiseModel::Isotropic::Sigma(3, 0.5);
  Vector mean{{0.1, 0.2, 0.05}};
  ExtendedPriorFactor<Pose2> factor(key, origin, mean, model);

  Pose2 x(1.1, 2.2, 0.3);
  Vector expected_error{{0.05463769, -0.038484723, -0.05}};
  Vector actual_error = factor.evaluateError(x);
  EXPECT(assert_equal(expected_error, actual_error, 1e-8));
}

//******************************************************************************
// Verifies the fixed-size Rot2 Local Jacobian is written to the dynamic output.
TEST(ExtendedPriorFactor, Rot2Jacobian) {
  const Rot2 origin = Rot2::fromAngle(0.3);
  const auto model = noiseModel::Isotropic::Sigma(1, 0.5);
  const ExtendedPriorFactor<Rot2> factor(1, origin, model);

  Matrix actualH;
  const Vector actualError =
      factor.evaluateError(Rot2::fromAngle(0.5), actualH);

  EXPECT(assert_equal(Vector1{0.2}, actualError, 1e-9));
  EXPECT(assert_equal(Matrix1::Identity(), actualH, 1e-9));
}

//******************************************************************************
TEST(ExtendedPriorFactor, Likelihood) {
  Key key(1);
  Pose2 origin(1, 2, 0.3);
  auto model = noiseModel::Isotropic::Sigma(3, 1.0);
  Vector mean{{0.1, 0.2, 0.05}};
  ExtendedPriorFactor<Pose2> factor(key, origin, mean, model);

  Pose2 x = origin.retract(mean);
  double expected_likelihood = 1.0;
  double actual_likelihood = factor.likelihood(x);
  EXPECT_DOUBLES_EQUAL(expected_likelihood, actual_likelihood, 1e-4);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

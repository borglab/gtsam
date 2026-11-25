/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStagDynamicsFactor.cpp
 * @brief   Unit tests for the STAG dynamics factor
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsFactor.h>
#include <gtsam_unstable/planning/dynamics/models/Unicycle.h>

#include <memory>
#include <stdexcept>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace unicycle_factors {

using DynamicsFactor = StagDynamicsFactor<Pose2, Vector2>;

const auto model = std::make_shared<const Unicycle>(0.1);  // default dt 0.1 s
const SharedNoiseModel noise =
    noiseModel::Diagonal::Sigmas(Vector3{0.1, 0.1, 0.05});

const Pose2 state(1.0, 2.0, 0.3);
const Vector2 control{2.0, 0.4};
const Pose2 nextState(1.3, 2.1, 0.4);  // not the prediction: nonzero error

// Checks that the factor has zero error when x_{k+1} is the prediction.
TEST(StagDynamicsFactor, ZeroErrorAtPrediction) {
  const Pose2 predicted = model->predict(state, control);
  EXPECT(assert_equal(Vector3::Zero().eval(),
                      Vector3(DynamicsFactor(X(0), U(0), X(1), model, noise)
                                  .evaluateError(state, control, predicted)),
                      1e-9));
}

// Checks that the error is Local(prediction, x_{k+1}), over the default dt and
// over a given dt.
TEST(StagDynamicsFactor, ErrorIsLocalOfPrediction) {
  const Vector3 expectedDefault =
      traits<Pose2>::Local(model->predict(state, control, 0.1), nextState);
  const Vector3 expectedGiven =
      traits<Pose2>::Local(model->predict(state, control, 0.5), nextState);

  EXPECT(assert_equal(expectedDefault,
                      Vector3(DynamicsFactor(X(0), U(0), X(1), model, noise)
                                  .evaluateError(state, control, nextState))));
  EXPECT(assert_equal(
      expectedGiven,
      Vector3(DynamicsFactor(X(0), U(0), X(1), model, 0.5, noise)
                  .evaluateError(state, control, nextState))));
}

// Checks that the factor keeps the default or given dt and the shared model.
TEST(StagDynamicsFactor, Accessors) {
  EXPECT_DOUBLES_EQUAL(0.1, DynamicsFactor(X(0), U(0), X(1), model, noise).dt(),
                       0.0);
  const DynamicsFactor given(X(0), U(0), X(1), model, 0.5, noise);
  EXPECT_DOUBLES_EQUAL(0.5, given.dt(), 0.0);
  EXPECT(given.dynamics() == model);
}

// Checks the Jacobians against numerical derivatives, at a point with nonzero
// error, over the default dt and over a given dt.
TEST(StagDynamicsFactor, Jacobians) {
  Values values;
  values.insert(X(0), state);
  values.insert(U(0), control);
  values.insert(X(1), nextState);

  EXPECT_CORRECT_FACTOR_JACOBIANS(
      DynamicsFactor(X(0), U(0), X(1), model, noise), values, 1e-5, 1e-5);
  EXPECT_CORRECT_FACTOR_JACOBIANS(
      DynamicsFactor(X(0), U(0), X(1), model, 0.5, noise), values, 1e-5, 1e-5);
}

// Checks that equals tells factors apart by dt, model and keys.
TEST(StagDynamicsFactor, Equals) {
  const auto otherModel = std::make_shared<const Unicycle>(0.1);

  const DynamicsFactor dynamics(X(0), U(0), X(1), model, noise);
  EXPECT(dynamics.equals(DynamicsFactor(X(0), U(0), X(1), model, noise)));
  EXPECT(!dynamics.equals(DynamicsFactor(X(0), U(0), X(1), model, 0.5, noise)));
  EXPECT(!dynamics.equals(DynamicsFactor(X(0), U(0), X(1), otherModel, noise)));
  EXPECT(!dynamics.equals(DynamicsFactor(X(0), U(0), X(2), model, noise)));
}

// Checks that a clone equals the original.
TEST(StagDynamicsFactor, Clone) {
  const DynamicsFactor dynamics(X(0), U(0), X(1), model, 0.5, noise);
  EXPECT(dynamics.equals(*dynamics.clone()));
}

// Checks that a null dynamics model is rejected by both constructors.
TEST(StagDynamicsFactor, NullModelThrows) {
  const std::shared_ptr<const Unicycle> nullModel;
  CHECK_EXCEPTION(DynamicsFactor(X(0), U(0), X(1), nullModel, noise),
                  std::invalid_argument);
  CHECK_EXCEPTION(DynamicsFactor(X(0), U(0), X(1), nullModel, 0.5, noise),
                  std::invalid_argument);
}

}  // namespace unicycle_factors
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testUnicycle.cpp
 * @brief   Unit tests for the Unicycle dynamics model and its dynamics factor
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam_unstable/planning/dynamics/models/Unicycle.h>

#include <memory>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace unicycle {

const Unicycle model(0.1);  // default time step 0.1 s
const Pose2 state(1.0, 2.0, 0.3);
const Vector2 control{2.0, 0.4};

// Checks that the state has dimension 3 and the control dimension 2.
TEST(Unicycle, Dimensions) {
  EXPECT_LONGS_EQUAL(3, Unicycle::StateDim);
  EXPECT_LONGS_EQUAL(2, Unicycle::ControlDim);
}

// Checks one Euler step over a given dt: from the identity, the unicycle
// moves v dt forward and turns by omega dt, and from any pose it composes
// that increment.
TEST(Unicycle, PredictGivenTimeStep) {
  EXPECT(assert_equal(Pose2(1.0, 0.0, 0.2),
                      model.predict(Pose2(), control, 0.5)));
  EXPECT(assert_equal(state.compose(Pose2(1.0, 0.0, 0.2)),
                      model.predict(state, control, 0.5)));
}

// Checks that predict without a time step uses the default dt.
TEST(Unicycle, PredictDefaultTimeStep) {
  EXPECT(assert_equal(Pose2(0.2, 0.0, 0.04), model.predict(Pose2(), control)));
  EXPECT(assert_equal(model.predict(state, control, 0.1),
                      model.predict(state, control)));
}

// Checks the Jacobians of both predict overloads against numerical
// derivatives.
TEST(Unicycle, PredictJacobians) {
  Matrix3 H_state;
  Matrix32 H_control;

  const auto predictDefault = [](const Pose2& x, const Vector2& u) {
    return model.predict(x, u);
  };
  model.predict(state, control, H_state, H_control);
  EXPECT(assert_equal(numericalDerivative21<Pose2, Pose2, Vector2>(
                          predictDefault, state, control),
                      H_state, 1e-8));
  EXPECT(assert_equal(numericalDerivative22<Pose2, Pose2, Vector2>(
                          predictDefault, state, control),
                      H_control, 1e-8));

  const auto predictGiven = [](const Pose2& x, const Vector2& u) {
    return model.predict(x, u, 0.5);
  };
  model.predict(state, control, 0.5, H_state, H_control);
  EXPECT(assert_equal(numericalDerivative21<Pose2, Pose2, Vector2>(
                          predictGiven, state, control),
                      H_state, 1e-8));
  EXPECT(assert_equal(numericalDerivative22<Pose2, Pose2, Vector2>(
                          predictGiven, state, control),
                      H_control, 1e-8));
}

// Checks that the unicycle's dynamics factor has zero error at the
// prediction, and correct Jacobians at a point with nonzero error.
TEST(Unicycle, DynamicsFactor) {
  const auto shared = std::make_shared<const Unicycle>(0.1);
  const UnicycleDynamicsFactor factor(
      X(0), U(0), X(1), shared,
      noiseModel::Diagonal::Sigmas(Vector3{0.1, 0.1, 0.05}));
  EXPECT(assert_equal(
      Vector3::Zero().eval(),
      Vector3(factor.evaluateError(state, control,
                                   shared->predict(state, control))),
      1e-9));

  Values values;
  values.insert(X(0), state);
  values.insert(U(0), control);
  values.insert(X(1), Pose2(1.3, 2.1, 0.4));  // not the prediction
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

}  // namespace unicycle
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

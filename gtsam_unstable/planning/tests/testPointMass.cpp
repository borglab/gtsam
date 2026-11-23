/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPointMass.cpp
 * @brief   Unit tests for the PointMass dynamics model and its dynamics factor
 * @date    November 17, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/planning/SamplingPlanner.h>
#include <gtsam_unstable/planning/dynamics/models/PointMass.h>

#include <memory>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace point_mass {

const PointMass model(0.1);  // default time step 0.1 s
const Vector4 state{1.0, 2.0, 3.0, 4.0};
const Vector2 control{0.5, -1.0};

// Checks that the state has dimension 4 and the control dimension 2.
TEST(PointMass, Dimensions) {
  EXPECT_LONGS_EQUAL(4, PointMass::StateDim);
  EXPECT_LONGS_EQUAL(2, PointMass::ControlDim);
}

// Checks one forward Euler step: the position moves by v dt with the old
// velocity, and the velocity by a dt.
TEST(PointMass, PredictGivenTimeStep) {
  EXPECT(assert_equal(Vector4{1.3, 2.4, 3.05, 3.9},
                      model.predict(state, control, 0.1)));
  EXPECT(assert_equal(Vector4{2.5, 4.0, 3.25, 3.5},
                      model.predict(state, control, 0.5)));
}

// Integrating the field must reproduce the model's discrete prediction.
TEST(PointMass, PlannerIntegratesModelField) {
  const double dt = 0.5;
  SamplingPlanner<Vector4, Vector2> planner({X(0), X(1)}, {U(0)}, dt);
  Values initial;
  initial.insert(X(0), state);
  const Values trajectory =
      planner.rollout(initial, control, PointMass::dynamics);
  EXPECT(assert_equal(Vector4{3.0, 4.0, 0.5, -1.0},
                      PointMass::dynamics(state, control)));
  EXPECT(assert_equal(model.predict(state, control, dt),
                      trajectory.at<Vector4>(X(1))));
}

// Checks that predict without a time step uses the default dt.
TEST(PointMass, PredictDefaultTimeStep) {
  EXPECT(assert_equal(model.predict(state, control, 0.1),
                      model.predict(state, control)));
}

// Checks the Jacobians of both predict overloads against numerical
// derivatives.
TEST(PointMass, PredictJacobians) {
  Matrix4 H_state;
  Matrix42 H_control;

  const auto predictDefault = [](const Vector4& x, const Vector2& u) {
    return model.predict(x, u);
  };
  model.predict(state, control, H_state, H_control);
  EXPECT(assert_equal(numericalDerivative21<Vector4, Vector4, Vector2>(
                          predictDefault, state, control),
                      H_state, 1e-8));
  EXPECT(assert_equal(numericalDerivative22<Vector4, Vector4, Vector2>(
                          predictDefault, state, control),
                      H_control, 1e-8));

  const auto predictGiven = [](const Vector4& x, const Vector2& u) {
    return model.predict(x, u, 0.5);
  };
  model.predict(state, control, 0.5, H_state, H_control);
  EXPECT(assert_equal(numericalDerivative21<Vector4, Vector4, Vector2>(
                          predictGiven, state, control),
                      H_state, 1e-8));
  EXPECT(assert_equal(numericalDerivative22<Vector4, Vector4, Vector2>(
                          predictGiven, state, control),
                      H_control, 1e-8));
}

// Checks that the point mass's dynamics factor has zero error at the
// prediction, and correct Jacobians at a point with nonzero error.
TEST(PointMass, DynamicsFactor) {
  const auto shared = std::make_shared<const PointMass>(0.1);
  const PointMassDynamicsFactor factor(X(0), U(0), X(1), shared,
                                       noiseModel::Isotropic::Sigma(4, 0.1));
  EXPECT(assert_equal(
      Vector4::Zero().eval(),
      Vector4(factor.evaluateError(state, control,
                                   shared->predict(state, control))),
      1e-9));

  Values values;
  values.insert(X(0), state);
  values.insert(U(0), control);
  values.insert(X(1), Vector4{1.5, 2.5, 3.0, 3.5});  // not the prediction
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

}  // namespace point_mass
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

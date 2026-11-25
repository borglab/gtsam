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
const Vector state = Vector4{1.0, 2.0, 3.0, 4.0};
const Vector control = Vector2{0.5, -1.0};

// Checks that the state has dimension 4 and the control dimension 2.
TEST(PointMass, Dimensions) {
  CHECK_EXCEPTION(model.predict(Vector::Zero(3), control),
                  std::invalid_argument);
  CHECK_EXCEPTION(model.predict(state, Vector::Zero(3)), std::invalid_argument);
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
  SamplingPlanner<Vector, Vector> planner({X(0), X(1)}, {U(0)}, dt);
  Values initial;
  initial.insert(X(0), state);
  const Values trajectory =
      planner.rollout(initial, control, PointMass::dynamics);
  EXPECT(assert_equal(Vector4{3.0, 4.0, 0.5, -1.0},
                      PointMass::dynamics(state, control)));
  EXPECT(assert_equal(model.predict(state, control, dt),
                      trajectory.at<Vector>(X(1))));
}

// A planner with dynamic controls reads their width from the trajectory.
TEST(PointMass, NominalControlDimensions) {
  SamplingPlanner<Vector, Vector> planner({X(0), X(1), X(2)}, {U(0), U(1)},
                                          0.1);
  Values values;
  values.insert(U(0), control);
  values.insert(U(1), control);
  const Matrix nominal = planner.nominalControls(values);
  EXPECT(assert_equal(control.replicate(1, 2).eval(), nominal));
  values.update(U(1), Vector::Zero(3).eval());
  CHECK_EXCEPTION(planner.nominalControls(values), std::invalid_argument);
}

// Checks that predict without a time step uses the default dt.
TEST(PointMass, PredictDefaultTimeStep) {
  EXPECT(assert_equal(model.predict(state, control, 0.1),
                      model.predict(state, control)));
}

// Checks the Jacobians of both predict overloads against numerical
// derivatives.
TEST(PointMass, PredictJacobians) {
  Matrix H_state;
  Matrix H_control;

  const auto predictDefault = [](const Vector& x, const Vector& u) {
    return model.predict(x, u);
  };
  model.predict(state, control, H_state, H_control);
  EXPECT(assert_equal(numericalDerivative21<Vector, Vector, Vector, 4>(
                          predictDefault, state, control),
                      H_state, 1e-8));
  EXPECT(assert_equal(numericalDerivative22<Vector, Vector, Vector, 2>(
                          predictDefault, state, control),
                      H_control, 1e-8));

  const auto predictGiven = [](const Vector& x, const Vector& u) {
    return model.predict(x, u, 0.5);
  };
  model.predict(state, control, 0.5, H_state, H_control);
  EXPECT(assert_equal(numericalDerivative21<Vector, Vector, Vector, 4>(
                          predictGiven, state, control),
                      H_state, 1e-8));
  EXPECT(assert_equal(numericalDerivative22<Vector, Vector, Vector, 2>(
                          predictGiven, state, control),
                      H_control, 1e-8));
}

// Checks that the point mass's dynamics factor has zero error at the
// prediction, and correct Jacobians at a point with nonzero error.
TEST(PointMass, DynamicsFactor) {
  const auto shared = std::make_shared<const PointMass>(0.1);
  const PointMassDynamicsFactor factor(X(0), U(0), X(1), shared,
                                       noiseModel::Isotropic::Sigma(4, 0.1));
  EXPECT(assert_equal(Vector4::Zero().eval(),
                      Vector4(factor.evaluateError(
                          state, control, shared->predict(state, control))),
                      1e-9));

  CHECK_EXCEPTION(factor.evaluateError(state, control, Vector::Zero(3)),
                  std::invalid_argument);

  Values values;
  values.insert(X(0), state);
  values.insert(U(0), control);
  values.insert(X(1),
                Vector(Vector4{1.5, 2.5, 3.0, 3.5}));  // not the prediction
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

}  // namespace point_mass
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

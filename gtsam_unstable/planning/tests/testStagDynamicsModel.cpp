/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testStagDynamicsModel.cpp
 * @brief   Unit tests for the StagDynamicsModel base class
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam_unstable/planning/dynamics/StagDynamicsModel.h>
#include <gtsam_unstable/planning/dynamics/models/Unicycle.h>

using namespace gtsam;

/* ************************************************************************* */
namespace unicycle {

const Unicycle model(0.1);  // default time step 0.1 s
const Pose2 state(1.0, 2.0, 0.3);
const Vector2 control{2.0, 0.4};

// Checks that the dimensions come from the state and control types.
TEST(StagDynamicsModel, Dimensions) {
  EXPECT_LONGS_EQUAL(3, Unicycle::StateDim);
  EXPECT_LONGS_EQUAL(2, Unicycle::ControlDim);
}

// Checks that dt() returns the time step given to the constructor.
TEST(StagDynamicsModel, DefaultTimeStep) {
  EXPECT_DOUBLES_EQUAL(0.1, model.dt(), 0.0);
}

// Checks that the base predict without a time step uses the default dt.
TEST(StagDynamicsModel, PredictDefaultTimeStep) {
  // From the identity, one step moves v dt forward and turns by omega dt.
  EXPECT(assert_equal(Pose2(0.2, 0.0, 0.04), model.predict(Pose2(), control)));
  EXPECT(assert_equal(model.predict(state, control, 0.1),
                      model.predict(state, control)));
}

// Checks that predict with a time step steps over that dt.
TEST(StagDynamicsModel, PredictGivenTimeStep) {
  EXPECT(assert_equal(Pose2(1.0, 0.0, 0.2),
                      model.predict(Pose2(), control, 0.5)));
  EXPECT(assert_equal(state.compose(Pose2(1.0, 0.0, 0.2)),
                      model.predict(state, control, 0.5)));
}

// Checks the Jacobians of both predict overloads against numerical
// derivatives.
TEST(StagDynamicsModel, PredictJacobians) {
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

// Checks that both predict overloads dispatch through a base-class reference.
TEST(StagDynamicsModel, BaseClassReference) {
  const StagDynamicsModel<Pose2, Vector2>& base = model;
  EXPECT(assert_equal(model.predict(state, control, 0.5),
                      base.predict(state, control, 0.5)));
  EXPECT(assert_equal(model.predict(state, control),
                      base.predict(state, control)));
}

/**
 * A unicycle that overrides the default-dt predict to step over 2 dt(), to
 * check that a derived model's own version of that overload is used.
 */
class DoubleStepUnicycle : public Unicycle {
 public:
  using Unicycle::Unicycle;
  using Unicycle::predict;  // keep the explicit-dt overload visible

  /// Steps over twice the default time step.
  Pose2 predict(const Pose2& state, const Vector2& control,
                OptionalJacobian<3, 3> H_state = {},
                OptionalJacobian<3, 2> H_control = {}) const override {
    return predict(state, control, 2.0 * dt(), H_state, H_control);
  }
};

// Checks that a derived model's default-dt predict replaces the base one,
// while the explicit-dt predict is unchanged.
TEST(StagDynamicsModel, OverrideDefaultTimeStep) {
  const DoubleStepUnicycle doubleStep(0.1);
  const StagDynamicsModel<Pose2, Vector2>& base = doubleStep;
  EXPECT(assert_equal(Pose2(0.4, 0.0, 0.08), base.predict(Pose2(), control)));
  EXPECT(assert_equal(Pose2(0.2, 0.0, 0.04),
                      base.predict(Pose2(), control, 0.1)));
}

}  // namespace unicycle
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

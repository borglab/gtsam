/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testVanillaMPPIController.cpp
 * @brief   Unit tests for the vanilla MPPI parameters and controller. The
 *          noise sampling itself is tested with the sampler.
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/planning/VanillaMPPIController.h>

#include "LinearPlanningFactor.h"

#include <stdexcept>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace vanilla_mppi_params {

// Checks the default vanilla MPPI parameters.
TEST(VanillaMPPIParams, Defaults) {
  const VanillaMPPIParams params;
  EXPECT_LONGS_EQUAL(1024, params.numSamples);
  EXPECT_LONGS_EQUAL(1, params.iterations);
  EXPECT_DOUBLES_EQUAL(1.0, params.inverseTemperature, 0.0);
  EXPECT_LONGS_EQUAL(0, params.controlCovariance.size());
  EXPECT_LONGS_EQUAL(42, params.seed);
  EXPECT(!params.verbose);
}

// Checks that the vanilla MPPI parameters are usable as controller params.
TEST(VanillaMPPIParams, AreControllerParams) {
  VanillaMPPIParams params;
  params.verbose = true;
  const MPCControllerParams& base = params;
  EXPECT(base.verbose);
}

}  // namespace vanilla_mppi_params
/* ************************************************************************* */

/* ************************************************************************* */
namespace vanilla_mppi_controller {

using Controller = VanillaMPPIController<planning_test::LinearPlanningFactor>;

/// Parameters with a 2x2 control covariance, 8 samples and seed 7.
VanillaMPPIParams testParams() {
  VanillaMPPIParams params;
  params.numSamples = 8;
  params.controlCovariance = Matrix{{0.04, 0.01}, {0.01, 0.09}};
  params.seed = 7;
  return params;
}

// Checks that construction accepts an empty covariance and rejects one that
// is not square, not positive definite, or not the size of CONTROL.
TEST(VanillaMPPIController, Construction) {
  const Controller controller;
  EXPECT_LONGS_EQUAL(0, controller.stepCounter());
  EXPECT_LONGS_EQUAL(2, Controller::ControlDim);

  VanillaMPPIParams nonSquare;
  nonSquare.controlCovariance = Matrix::Ones(2, 3);
  CHECK_EXCEPTION(Controller{nonSquare}, std::invalid_argument);

  VanillaMPPIParams notPositiveDefinite;
  notPositiveDefinite.controlCovariance = Matrix{{1.0, 2.0}, {2.0, 1.0}};
  CHECK_EXCEPTION(Controller{notPositiveDefinite}, std::invalid_argument);

  VanillaMPPIParams threeD;
  threeD.controlCovariance = Matrix::Identity(3, 3);
  CHECK_EXCEPTION(Controller{threeD}, std::invalid_argument);
}

// Checks that the controller's sampler is built from its parameters and that
// sampling through the controller gives the sampler's noise and controls.
TEST(VanillaMPPIController, SamplingDelegatesToSampler) {
  const VanillaMPPIParams params = testParams();
  const Controller controller(params);
  EXPECT_LONGS_EQUAL(2, controller.sampler().dim());
  EXPECT_LONGS_EQUAL(8, controller.sampler().numSamples());
  EXPECT_LONGS_EQUAL(7, controller.sampler().seed());

  const GaussianNoiseSampler sampler(params.controlCovariance, 8, 7);
  const size_t horizon = 3;
  EXPECT(assert_equal(Matrix(sampler.sampleNoise(horizon)),
                      Matrix(controller.sampleNoise(horizon)), 0.0));
  const Matrix nominal = Matrix::Ones(2, horizon);
  EXPECT(assert_equal(Matrix(sampler.sampleControls(nominal)),
                      Matrix(controller.sampleControls(nominal)), 0.0));
  EXPECT_LONGS_EQUAL(2, controller.stepCounter());
}

// With one sample, its control has unit weight and determines the next state.
TEST(VanillaMPPIController, OptimizeLinearDynamics) {
  auto params = testParams();
  params.numSamples = 1;
  Controller controller(params);
  NonlinearFactorGraph graph;
  graph.emplace_shared<planning_test::LinearPlanningFactor>(X(0), U(0), X(1));
  const Vector2 start{2.0, -1.0}, nominal{1.0, 0.5};
  Values initial;
  initial.insert(X(0), start);
  initial.insert(U(0), nominal);
  initial.insert(X(1), Vector2{0.0, 0.0});

  const GaussianNoiseSampler sampler(params.controlCovariance, 1, params.seed);
  const Vector2 expectedControl = sampler.sampleControls(nominal).col(0);
  const Values plan = controller.optimize(graph, initial, {U(0)});
  EXPECT(assert_equal(expectedControl, plan.at<Vector2>(U(0))));
  EXPECT(assert_equal(Vector2(start + expectedControl), plan.at<Vector2>(X(1))));
}

}  // namespace vanilla_mppi_controller
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

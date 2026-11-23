/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testGenericMPPIController.cpp
 * @brief   Unit tests for the generic MPPI controller with a custom,
 *          deterministic sampler. The vanilla (Gaussian) controller is tested
 *          in testVanillaMPPIController.cpp.
 * @date    November 8, 2026
 * @author  Akash Ratheesh
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/planning/GenericMPPIController.h>

#include "LinearPlanningFactor.h"

#include <algorithm>
#include <cmath>
#include <thread>
#include <utility>
#include <vector>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace generic_mppi_params {

// Checks the default generic MPPI parameters.
TEST(GenericMPPIParams, Defaults) {
  const GenericMPPIParams params;
  EXPECT_LONGS_EQUAL(1, params.iterations);
  EXPECT_DOUBLES_EQUAL(1.0, params.inverseTemperature, 0.0);
  EXPECT(!params.verbose);
  EXPECT(!params.parallelCandidateScoring);
}

}  // namespace generic_mppi_params
/* ************************************************************************* */

/* ************************************************************************* */
namespace generic_mppi_controller {

/**
 * A deterministic control sampler: sample i is the nominal controls plus a
 * fixed offset, the same at every step. It counts its calls.
 */
class OffsetSampler {
 public:
  explicit OffsetSampler(std::vector<Vector2> offsets)
      : offsets_(std::move(offsets)) {}

  /// Sample i at step k: nominalControls.col(k) + offsets[i].
  const Matrix& sampleControls(const Matrix& nominalControls) const {
    const size_t H = nominalControls.cols();
    samples_.resize(2, offsets_.size() * H);
    for (size_t i = 0; i < offsets_.size(); ++i) {
      for (size_t k = 0; k < H; ++k) {
        samples_.col(i * H + k) = nominalControls.col(k) + offsets_[i];
      }
    }
    ++calls_;
    return samples_;
  }

  /// The number of sampleControls calls so far.
  size_t calls() const { return calls_; }

 private:
  std::vector<Vector2> offsets_;  ///< The offset of each sample.
  mutable Matrix samples_;        ///< The last samples.
  mutable size_t calls_ = 0;      ///< sampleControls calls so far.
};

using planning_test::LinearPlanningFactor;
using Controller = GenericMPPIController<LinearPlanningFactor, OffsetSampler>;

const Vector2 nominal{1.0, 0.1};
const std::vector<Vector2> offsets{Vector2{0.5, 0.0}, Vector2{-0.5, 0.0},
                                   Vector2{0.0, 0.2}};

/// Future graph of three steps: a dynamics factor on X(k), U(k), X(k+1).
NonlinearFactorGraph futureGraph() {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k < 3; ++k) {
    graph.emplace_shared<LinearPlanningFactor>(X(k), U(k), X(k + 1));
  }
  return graph;
}

/// X(0), the nominal control at every step and zero guesses for X(k+1).
Values initialValues() {
  Values initial;
  initial.insert(X(0), Vector2{1.0, 2.0});
  for (size_t k = 0; k < 3; ++k) {
    initial.insert(U(k), nominal);
    initial.insert(X(k + 1), Vector2{0.0, 0.0});
  }
  return initial;
}

/// initial with the control u at every step and the states rolled out.
Values rolledOut(const Values& initial, const Vector2& u) {
  Values values = initial;
  Vector2 state = values.at<Vector2>(X(0));
  for (size_t k = 0; k < 3; ++k) {
    values.update(U(k), u);
    state += u;
    values.update(X(k + 1), state);
  }
  return values;
}

// A callback cost must stay on the caller even if parallel scoring is requested.
TEST(GenericMPPIController, NonSendableCostOnCaller) {
  auto graph = futureGraph();
  const auto caller = std::this_thread::get_id();
  graph.emplace_shared<CustomFactor>(
      noiseModel::Unit::Create(2), KeyVector{X(3)},
      [caller](const CustomFactor&, const Values& values,
               OptionalJacobianVector) {
        if (std::this_thread::get_id() != caller)
          throw std::runtime_error("Evaluated non-sendable cost on a worker.");
        return values.at<Vector2>(X(3));
      });
  auto solve = [&](bool optIn) {
    GenericMPPIParams params;
    params.parallelCandidateScoring = optIn;
    Controller controller(params, OffsetSampler(offsets));
    return controller.optimize(graph, initialValues(), {U(0), U(1), U(2)});
  };
  EXPECT(assert_equal(solve(false), solve(true)));
}

class NonSendableDynamics : public LinearPlanningFactor {
 public:
  using LinearPlanningFactor::LinearPlanningFactor;
  bool sendable() const override { return false; }
  Vector2 predict(const Vector2& state, const Vector2& control) const {
    if (std::this_thread::get_id() != caller_)
      throw std::runtime_error("Predicted non-sendable dynamics on a worker.");
    return LinearPlanningFactor::predict(state, control);
  }

 private:
  std::thread::id caller_ = std::this_thread::get_id();
};

// A non-sendable dynamics factor also keeps predict() calls on the caller.
TEST(GenericMPPIController, NonSendableDynamicsOnCaller) {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k < 3; ++k)
    graph.emplace_shared<NonSendableDynamics>(X(k), U(k), X(k + 1));
  GenericMPPIParams params;
  params.parallelCandidateScoring = true;
  GenericMPPIController<NonSendableDynamics, OffsetSampler> controller(
      params, OffsetSampler(offsets));
  const auto result =
      controller.optimize(graph, initialValues(), {U(0), U(1), U(2)});
  EXPECT_DOUBLES_EQUAL(0.0, graph.error(result), 1e-12);
}

// A repeated or disconnected step is invalid even when no samples are drawn.
TEST(GenericMPPIController, ValidateChainBeforeSampling) {
  const KeyVector keys{U(0), U(1), U(2)};
  auto duplicate = futureGraph();
  duplicate.emplace_shared<LinearPlanningFactor>(X(0), U(0), X(1));
  auto disconnected = futureGraph();
  disconnected[1] = std::make_shared<LinearPlanningFactor>(X(0), U(1), X(2));
  auto cyclic = futureGraph();
  cyclic[2] = std::make_shared<LinearPlanningFactor>(X(2), U(2), X(0));
  for (size_t iterations : {0, 1}) {
    GenericMPPIParams params;
    params.iterations = iterations;
    Controller controller(params, OffsetSampler(offsets));
    for (const auto* invalid : {&duplicate, &disconnected, &cyclic}) {
      CHECK_EXCEPTION(controller.optimize(*invalid, initialValues(), keys),
                      std::invalid_argument);
      CHECK_EXCEPTION(
          controller.initialValues(*invalid, keys, nominal, Matrix::Zero(2, 3)),
          std::invalid_argument);
    }
    EXPECT_LONGS_EQUAL(0, controller.sampler().calls());
  }
}

// Malformed control matrices fail before a direct rollout writes any values.
TEST(GenericMPPIController, CheckRolloutShape) {
  const auto graph = futureGraph();
  const Controller controller(GenericMPPIParams{}, OffsetSampler(offsets));
  Controller::Chain chain;
  controller.updateDynamicsFactors(graph, {U(0), U(1), U(2)}, chain);
  Values initial = initialValues();
  CHECK_EXCEPTION(controller.rollout(chain, Matrix::Zero(2, 2), initial),
                  std::invalid_argument);
  CHECK_EXCEPTION(controller.rollout(chain, Matrix::Zero(3, 3), initial),
                  std::invalid_argument);
  EXPECT(assert_equal(initialValues(), initial));
}

// Checks that the generic controller keeps its parameters and sampler.
TEST(GenericMPPIController, Construction) {
  GenericMPPIParams params;
  params.iterations = 4;
  const Controller controller(params, OffsetSampler(offsets));
  EXPECT_LONGS_EQUAL(4, controller.params().iterations);
  EXPECT_LONGS_EQUAL(0, controller.sampler().calls());
  EXPECT_LONGS_EQUAL(2, Controller::ControlDim);
}

// Construct correctly typed states and controls directly from a warm start.
TEST(GenericMPPIController, InitialRollout) {
  const Controller controller(GenericMPPIParams{}, OffsetSampler(offsets));
  const Values initial = initialValues();
  const Values plan = controller.initialValues(futureGraph(), {U(0), U(1), U(2)},
      initial.at<Vector2>(X(0)), nominal.replicate(1, 3));
  EXPECT(assert_equal(rolledOut(initial, nominal), plan));
}

// Checks optimize with a custom sampler on the dynamics-only graph: every
// sample costs 0, so the K = 3 samples (taken from the sampler) are weighted
// equally and each iteration adds the mean offset to ubar. The sampler is
// called once per iteration, and the plan satisfies the dynamics.
TEST(GenericMPPIController, OptimizeWithCustomSampler) {
  GenericMPPIParams params;
  params.iterations = 2;
  Controller controller(params, OffsetSampler(offsets));
  const Values initial = initialValues();
  const Values result =
      controller.optimize(futureGraph(), initial, {U(0), U(1), U(2)});

  const Vector2 meanOffset = (offsets[0] + offsets[1] + offsets[2]) / 3.0;
  const Values expected = rolledOut(initial, nominal + 2.0 * meanOffset);
  EXPECT(assert_equal(expected, result, 1e-12));
  EXPECT_DOUBLES_EQUAL(0.0, futureGraph().error(result), 1e-12);
  EXPECT_LONGS_EQUAL(2, controller.sampler().calls());
}

// Checks the weighting with a custom sampler: with a unit prior on every
// U(k) at target, sample i costs S_i = 3 * 0.5 |nominal + offset_i - target|^2,
// and the plan's control is sum_i w_i (nominal + offset_i) with
// w_i = exp(-(S_i - min S) * inverseTemperature), normalized.
TEST(GenericMPPIController, OptimizeWeightsByCost) {
  const Vector2 target{1.5, 0.1};
  NonlinearFactorGraph graph = futureGraph();
  for (size_t k = 0; k < 3; ++k) {
    graph.addPrior(U(k), target, noiseModel::Unit::Create(2));
  }
  GenericMPPIParams params;
  params.inverseTemperature = 2.0;
  Controller controller(params, OffsetSampler(offsets));
  const Values initial = initialValues();
  const Values result = controller.optimize(graph, initial, {U(0), U(1), U(2)});

  std::vector<double> costs, weights;
  for (const Vector2& offset : offsets) {
    costs.push_back(3.0 * 0.5 * (nominal + offset - target).squaredNorm());
  }
  const double minCost = std::min({costs[0], costs[1], costs[2]});
  double sum = 0.0;
  for (double cost : costs) {
    weights.push_back(std::exp(-(cost - minCost) * params.inverseTemperature));
    sum += weights.back();
  }
  Vector2 control = Vector2::Zero();
  for (size_t i = 0; i < offsets.size(); ++i) {
    control += (weights[i] / sum) * (nominal + offsets[i]);
  }
  EXPECT(assert_equal(rolledOut(initial, control), result, 1e-9));
}

}  // namespace generic_mppi_controller
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

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
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/config.h>  // for GTSAM_USE_TBB
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam_unstable/mpc_stag/controller/VanillaMPPIController.h>
#include <gtsam_unstable/mpc_stag/dynamics/models/Unicycle.h>

#ifdef GTSAM_USE_TBB
#include <tbb/global_control.h>
#endif

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

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
  const StagMPCControllerParams& base = params;
  EXPECT(base.verbose);
}

}  // namespace vanilla_mppi_params
/* ************************************************************************* */

/* ************************************************************************* */
namespace vanilla_mppi_controller {

using Controller = VanillaMPPIController<Pose2, Vector2>;

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

}  // namespace vanilla_mppi_controller
/* ************************************************************************* */

/* ************************************************************************* */
namespace vanilla_mppi_optimize {

using Controller = VanillaMPPIController<Pose2, Vector2>;

/// A Unicycle that counts its predict calls, from any thread.
class CountingUnicycle : public Unicycle {
 public:
  explicit CountingUnicycle(double dt) : Unicycle(dt) {}
  using Unicycle::predict;  // keep the default-dt overload visible

  /// Count the call, then predict as Unicycle does.
  Pose2 predict(const Pose2& state, const Vector2& control, double dt,
                OptionalJacobian<3, 3> H_state = {},
                OptionalJacobian<3, 2> H_control = {}) const override {
    ++calls_;
    return Unicycle::predict(state, control, dt, H_state, H_control);
  }

  /// The number of predict calls so far.
  size_t calls() const { return calls_; }

 private:
  mutable std::atomic<size_t> calls_{0};  ///< predict calls so far.
};

const auto model = std::make_shared<const Unicycle>(0.1);  // default dt 0.1 s
const double dt = 0.2;  // the factors' dt, not the model's default
const SharedNoiseModel noise =
    noiseModel::Diagonal::Sigmas(Vector3{0.1, 0.1, 0.05});

/// Parameters with a 2x2 control covariance, 8 samples, 3 iterations.
VanillaMPPIParams testParams() {
  VanillaMPPIParams params;
  params.numSamples = 8;
  params.iterations = 3;
  params.controlCovariance = Matrix{{0.04, 0.01}, {0.01, 0.09}};
  return params;
}

/// Future graph of three steps: a dynamics factor on X(k), U(k), X(k+1).
NonlinearFactorGraph futureGraph() {
  NonlinearFactorGraph graph;
  for (size_t k = 0; k < 3; ++k) {
    graph.emplace_shared<UnicycleDynamicsFactor>(X(k), U(k), X(k + 1), model,
                                                 dt, noise);
  }
  return graph;
}

/// Three Vector2 controls U(0), U(1), U(2).
Values threeControls() {
  Values initial;
  initial.insert(U(0), Vector2{1.0, -1.0});
  initial.insert(U(1), Vector2{2.0, 0.0});
  initial.insert(U(2), Vector2{3.0, 1.0});
  return initial;
}

/// The three controls, the current state X(0) and guesses for X(1) to X(3).
Values initialValues() {
  Values initial = threeControls();
  initial.insert(X(0), Pose2(1.0, 2.0, 0.3));
  for (size_t k = 1; k <= 3; ++k) initial.insert(X(k), Pose2());
  return initial;
}

const Pose2 goal(2.0, 2.5, 0.5);

/**
 * The future graph plus a goal cost: a prior on each X(k+1) at the goal and
 * a unit prior on each U(k) at zero.
 */
NonlinearFactorGraph goalGraph() {
  NonlinearFactorGraph graph = futureGraph();
  for (size_t k = 0; k < 3; ++k) {
    graph.addPrior(X(k + 1), goal, noise);
    graph.addPrior(U(k), Vector2{0.0, 0.0}, noiseModel::Unit::Create(2));
  }
  return graph;
}

/**
 * initial with the controls U(k) = controls.col(k) and the states X(k+1)
 * rolled out from X(0) through the model at the factors' dt.
 */
Values rolledOut(const Values& initial, const Matrix& controls) {
  Values values = initial;
  Pose2 state = values.at<Pose2>(X(0));
  for (size_t k = 0; k < 3; ++k) {
    const Vector2 control = controls.col(k);
    values.update(U(k), control);
    state = model->predict(state, control, dt);
    values.update(X(k + 1), state);
  }
  return values;
}

/// The controls U(0) to U(2) of values, one column per step.
Matrix controlsOf(const Values& values) {
  Matrix controls(2, 3);
  for (size_t k = 0; k < 3; ++k) controls.col(k) = values.at<Vector2>(U(k));
  return controls;
}

/**
 * Reference vanilla MPPI on the three-step graphs, with plain loops. Each
 * iteration samples around ubar with a sampler of the same settings, scores
 * every rolled-out sample with the graph error S_i, weights it with
 * exp(-(S_i - min S) * inverseTemperature), normalized, and sets ubar to the
 * weighted mean of the samples. It returns initial rolled out with the final
 * ubar.
 */
Values referenceOptimize(const NonlinearFactorGraph& graph,
                         const Values& initial,
                         const VanillaMPPIParams& params) {
  const size_t H = 3, K = params.numSamples;
  const GaussianNoiseSampler sampler(params.controlCovariance, K, params.seed);
  Matrix ubar = controlsOf(initial);
  for (size_t iteration = 0; iteration < params.iterations; ++iteration) {
    const Matrix sampled = sampler.sampleControls(ubar);
    std::vector<double> costs(K);
    for (size_t i = 0; i < K; ++i) {
      costs[i] = graph.error(rolledOut(initial, sampled.middleCols(i * H, H)));
    }
    const double minCost = *std::min_element(costs.begin(), costs.end());
    std::vector<double> weights(K);
    double sum = 0.0;
    for (size_t i = 0; i < K; ++i) {
      weights[i] =
          std::exp(-(costs[i] - minCost) * params.inverseTemperature);
      sum += weights[i];
    }
    ubar.setZero();
    for (size_t i = 0; i < K; ++i) {
      ubar += (weights[i] / sum) * sampled.middleCols(i * H, H);
    }
  }
  return rolledOut(initial, ubar);
}

// Checks that rollout writes each control and x_{k+1} = f(x_k, u_k, dt) with
// each factor's dt, after which the dynamics-only graph has zero error.
TEST(VanillaMPPIController, Rollout) {
  const NonlinearFactorGraph graph = futureGraph();
  Controller::Chain chain;
  for (const auto& factor : graph) {
    chain.push_back(
        std::dynamic_pointer_cast<const Controller::DynamicsFactor>(factor));
  }
  const Matrix controls{{0.5, 1.0, 1.5}, {0.2, -0.1, 0.0}};
  Values values = initialValues();
  EXPECT(graph.error(values) > 0.0);

  const Controller controller(testParams());
  controller.rollout(chain, controls, values);

  Pose2 expected = values.at<Pose2>(X(0));
  for (size_t k = 0; k < 3; ++k) {
    const Vector2 control = controls.col(k);
    EXPECT(assert_equal(control, values.at<Vector2>(U(k)), 0.0));
    expected = model->predict(expected, control, dt);
    EXPECT(assert_equal(expected, values.at<Pose2>(X(k + 1)), 0.0));
  }
  EXPECT_DOUBLES_EQUAL(0.0, graph.error(values), 1e-12);
}

/// The dynamics factors of goalGraph(), in step order.
Controller::Chain goalChain(const NonlinearFactorGraph& graph) {
  Controller::Chain chain;
  for (const auto& factor : graph) {
    const auto dynamics =
        std::dynamic_pointer_cast<const Controller::DynamicsFactor>(factor);
    if (dynamics) chain.push_back(dynamics);
  }
  return chain;
}

// Checks that rolloutAndScore gives each sample the error of its rollout from
// initial over the factors other than the chain's (the priors, factors 3 and
// up of goalGraph()), which is also its graph error since the rollout
// satisfies the chain, and the same costs on one thread.
TEST(VanillaMPPIController, RolloutAndScore) {
  const NonlinearFactorGraph graph = goalGraph();
  const Controller::Chain chain = goalChain(graph);
  const Values initial = initialValues();
  const Controller controller(testParams());
  const Matrix sampled = controller.sampleControls(controlsOf(initial));

  const Vector costs =
      controller.rolloutAndScore(graph, chain, sampled, initial);
  EXPECT_LONGS_EQUAL(8, costs.size());
  for (size_t i = 0; i < 8; ++i) {
    const Values values = rolledOut(initial, sampled.middleCols(i * 3, 3));
    double priors = 0.0;
    for (size_t j = 3; j < graph.size(); ++j) {
      priors += graph.at(j)->error(values);
    }
    EXPECT_DOUBLES_EQUAL(priors, costs(i), 0.0);
    EXPECT_DOUBLES_EQUAL(graph.error(values), costs(i), 0.0);
  }
#ifdef GTSAM_USE_TBB
  const tbb::global_control oneThread(
      tbb::global_control::max_allowed_parallelism, 1);
  EXPECT(assert_equal(
      costs, controller.rolloutAndScore(graph, chain, sampled, initial), 0.0));
#endif
}

// Checks that rolloutAndScore rejects an empty chain, and sampled controls
// with the wrong number of rows or a column count that is not a multiple of
// the chain length.
TEST(VanillaMPPIController, RolloutAndScoreChecksShapes) {
  const NonlinearFactorGraph graph = goalGraph();
  const Controller::Chain chain = goalChain(graph);
  const Values initial = initialValues();
  const Controller controller(testParams());
  CHECK_EXCEPTION(controller.rolloutAndScore(graph, Controller::Chain(),
                                             Matrix::Zero(2, 6), initial),
                  std::invalid_argument);
  CHECK_EXCEPTION(
      controller.rolloutAndScore(graph, chain, Matrix::Zero(3, 6), initial),
      std::invalid_argument);
  CHECK_EXCEPTION(
      controller.rolloutAndScore(graph, chain, Matrix::Zero(2, 7), initial),
      std::invalid_argument);
}

// Checks that rolloutAndScore does not evaluate the chain's dynamics factors:
// the model predicts once per step and sample, in the rollout only.
TEST(VanillaMPPIController, RolloutAndScoreSkipsChain) {
  const auto counting = std::make_shared<const CountingUnicycle>(0.1);
  NonlinearFactorGraph graph;
  for (size_t k = 0; k < 3; ++k) {
    graph.emplace_shared<UnicycleDynamicsFactor>(X(k), U(k), X(k + 1),
                                                 counting, dt, noise);
    graph.addPrior(X(k + 1), goal, noise);
  }
  const Values initial = initialValues();
  const Controller controller(testParams());
  const Matrix sampled = controller.sampleControls(controlsOf(initial));
  controller.rolloutAndScore(graph, goalChain(graph), sampled, initial);
  EXPECT_LONGS_EQUAL(8 * 3, counting->calls());
}

// Checks that rolloutAndScore scores a dynamics factor that is not in the
// chain: one from X(1) with the control U(5) to X(3), which the rollout does
// not satisfy.
TEST(VanillaMPPIController, RolloutAndScoreScoresOtherDynamics) {
  NonlinearFactorGraph graph = futureGraph();
  const auto other = std::make_shared<UnicycleDynamicsFactor>(X(1), U(5), X(3),
                                                              model, dt, noise);
  graph.push_back(other);
  Values initial = initialValues();
  initial.insert(U(5), Vector2{0.5, 0.0});
  const Controller controller(testParams());
  Controller::Chain chain;
  controller.updateDynamicsFactors(graph, {U(0), U(1), U(2)}, chain);
  const Matrix sampled = controller.sampleControls(controlsOf(initial));

  const Vector costs =
      controller.rolloutAndScore(graph, chain, sampled, initial);
  for (size_t i = 0; i < 8; ++i) {
    const double expected =
        other->error(rolledOut(initial, sampled.middleCols(i * 3, 3)));
    EXPECT(expected > 0.0);
    EXPECT_DOUBLES_EQUAL(expected, costs(i), 0.0);
  }
}

// Checks that rolloutAndScore rejects a chain that is not in step order,
// whose rollout would not satisfy the skipped dynamics factors.
TEST(VanillaMPPIController, RolloutAndScoreChecksOrder) {
  const NonlinearFactorGraph graph = goalGraph();
  const Controller controller(testParams());
  Controller::Chain chain;
  controller.updateDynamicsFactors(graph, {U(1), U(0), U(2)}, chain);
  CHECK_EXCEPTION(controller.rolloutAndScore(graph, chain, Matrix::Zero(2, 6),
                                             initialValues()),
                  std::invalid_argument);
}

// Checks that rolloutAndScore rejects a chain whose keys are not distinct: one
// in step order that returns to its first state, X(0) to X(1) to X(0).
TEST(VanillaMPPIController, RolloutAndScoreChecksDistinctKeys) {
  NonlinearFactorGraph graph;
  graph.emplace_shared<UnicycleDynamicsFactor>(X(0), U(0), X(1), model, dt,
                                               noise);
  graph.emplace_shared<UnicycleDynamicsFactor>(X(1), U(1), X(0), model, dt,
                                               noise);
  const Controller controller(testParams());
  Controller::Chain chain;
  controller.updateDynamicsFactors(graph, {U(0), U(1)}, chain);
  CHECK_EXCEPTION(controller.rolloutAndScore(graph, chain, Matrix::Zero(2, 4),
                                             initialValues()),
                  std::invalid_argument);
}

/**
 * The future graph with the goal priors of goalGraph() and, between them in
 * graph order, factors that rolloutAndScore scores through Values: a
 * BetweenFactor on the controls U(0) and U(2), a dynamics factor from X(1)
 * with the control U(5) to X(3), and a BetweenFactor on X(1) and X(3).
 */
NonlinearFactorGraph mixedGraph() {
  NonlinearFactorGraph graph = futureGraph();
  graph.emplace_shared<BetweenFactor<Vector2>>(U(0), U(2), Vector2{1.0, 0.5},
                                               noiseModel::Unit::Create(2));
  for (size_t k = 0; k < 3; ++k) {
    graph.addPrior(X(k + 1), goal, noise);
    if (k == 1) {
      graph.emplace_shared<UnicycleDynamicsFactor>(X(1), U(5), X(3), model, dt,
                                                   noise);
    }
    graph.addPrior(U(k), Vector2{0.0, 0.0}, noiseModel::Unit::Create(2));
  }
  graph.emplace_shared<BetweenFactor<Pose2>>(X(1), X(3), Pose2(0.5, 0.0, 0.1),
                                             noise);
  return graph;
}

// Checks that rolloutAndScore gives each sample exactly its graph error when
// priors, scored on the rolled-out states and controls, are mixed in graph
// order with factors scored through Values, which must see the same rollout;
// and the same costs on one thread.
TEST(VanillaMPPIController, RolloutAndScoreMixedFactors) {
  const NonlinearFactorGraph graph = mixedGraph();
  Values initial = initialValues();
  initial.insert(U(5), Vector2{0.5, 0.0});
  const Controller controller(testParams());
  Controller::Chain chain;
  controller.updateDynamicsFactors(graph, {U(0), U(1), U(2)}, chain);
  const Matrix sampled = controller.sampleControls(controlsOf(initial));

  const Vector costs =
      controller.rolloutAndScore(graph, chain, sampled, initial);
  for (size_t i = 0; i < 8; ++i) {
    const Values values = rolledOut(initial, sampled.middleCols(i * 3, 3));
    EXPECT_DOUBLES_EQUAL(graph.error(values), costs(i), 0.0);
  }
#ifdef GTSAM_USE_TBB
  const tbb::global_control oneThread(
      tbb::global_control::max_allowed_parallelism, 1);
  EXPECT(assert_equal(
      costs, controller.rolloutAndScore(graph, chain, sampled, initial), 0.0));
#endif
}

/// A PriorFactor that is never active, so NoiseModelFactor::error gives 0.
class InactivePrior : public PriorFactor<Pose2> {
 public:
  using PriorFactor<Pose2>::PriorFactor;

  /// Never active.
  bool active(const Values& /*values*/) const override { return false; }
};

// Checks that rolloutAndScore scores a subclass of PriorFactor, which may
// override active() or error(), through Values: an inactive prior on X(2)
// adds nothing to the graph error.
TEST(VanillaMPPIController, RolloutAndScorePriorSubclass) {
  NonlinearFactorGraph graph = goalGraph();
  graph.emplace_shared<InactivePrior>(X(2), Pose2(9.0, 9.0, 1.0), noise);
  const Values initial = initialValues();
  const Controller controller(testParams());
  const Matrix sampled = controller.sampleControls(controlsOf(initial));

  const Vector costs =
      controller.rolloutAndScore(graph, goalChain(graph), sampled, initial);
  for (size_t i = 0; i < 8; ++i) {
    const Values values = rolledOut(initial, sampled.middleCols(i * 3, 3));
    EXPECT_DOUBLES_EQUAL(graph.error(values), costs(i), 0.0);
  }
}

// Checks that rolloutAndScore scores priors with a robust noise model in
// scalar mode exactly as NoiseModelFactor::error does, with the robust loss of
// the error vector, which here differs from the loss of its squared
// Mahalanobis distance.
TEST(VanillaMPPIController, RolloutAndScoreRobustPriors) {
  const auto huber = noiseModel::mEstimator::Huber::Create(
      0.1, noiseModel::mEstimator::Base::Scalar);
  const auto robustState = noiseModel::Robust::Create(huber, noise);
  const auto robustControl =
      noiseModel::Robust::Create(huber, noiseModel::Unit::Create(2));
  NonlinearFactorGraph graph = futureGraph();
  for (size_t k = 0; k < 3; ++k) {
    graph.addPrior(X(k + 1), goal, robustState);
    graph.addPrior(U(k), Vector2{0.0, 0.0}, robustControl);
  }
  const Values initial = initialValues();
  const Controller controller(testParams());
  const Matrix sampled = controller.sampleControls(controlsOf(initial));

  const Vector costs =
      controller.rolloutAndScore(graph, goalChain(graph), sampled, initial);
  for (size_t i = 0; i < 8; ++i) {
    const Values values = rolledOut(initial, sampled.middleCols(i * 3, 3));
    EXPECT_DOUBLES_EQUAL(graph.error(values), costs(i), 0.0);
  }

  // The two losses differ for the prior on X(3) of the first sample.
  const Values first = rolledOut(initial, sampled.middleCols(0, 3));
  const PriorFactor<Pose2> terminal(X(3), goal, robustState);
  const Vector error = terminal.evaluateError(first.at<Pose2>(X(3)));
  EXPECT(std::abs(robustState->loss(error) -
                  robustState->loss(
                      robustState->squaredMahalanobisDistance(error))) > 1e-3);
}

// Checks that rolloutAndScore scores a prior whose noise model does not have
// the prior's dimension through Values, so that it throws as
// NoiseModelFactor::error does.
TEST(VanillaMPPIController, RolloutAndScoreChecksPriorDimension) {
  NonlinearFactorGraph graph = goalGraph();
  graph.emplace_shared<PriorFactor<Pose2>>(X(2), goal,
                                           Matrix(Matrix::Identity(2, 2)));
  const Values initial = initialValues();
  const Controller controller(testParams());
  const Matrix sampled = controller.sampleControls(controlsOf(initial));
  CHECK_EXCEPTION(
      controller.rolloutAndScore(graph, goalChain(graph), sampled, initial),
      std::invalid_argument);
}

/// initialValues() with the value at key replaced by one of another type.
template <class T>
Values retyped(Key key, const T& value) {
  Values values = initialValues();
  values.erase(key);
  values.insert(key, value);
  return values;
}

// Checks that checkRolloutTypes accepts values with exactly a Pose2 at every
// state key and a Vector2 at every control key of the chain, and throws for a
// value of another type or a missing key.
TEST(VanillaMPPIController, CheckRolloutTypes) {
  const Controller::Chain chain = goalChain(futureGraph());
  const Controller controller(testParams());
  controller.checkRolloutTypes(chain, initialValues());

  const Values dynamicControl = retyped(U(1), Vector(Vector2{2.0, 0.0}));
  CHECK_EXCEPTION(controller.checkRolloutTypes(chain, dynamicControl),
                  ValuesIncorrectType);
  const Values vectorState = retyped(X(2), Vector3{0.0, 0.0, 0.0});
  CHECK_EXCEPTION(controller.checkRolloutTypes(chain, vectorState),
                  ValuesIncorrectType);
  Values missing = initialValues();
  missing.erase(X(3));
  CHECK_EXCEPTION(controller.checkRolloutTypes(chain, missing),
                  ValuesKeyDoesNotExist);
}

// Checks that rolloutAndScore rejects initial values whose types rollout
// cannot overwrite in place: a control stored as a dynamic Vector.
TEST(VanillaMPPIController, RolloutAndScoreChecksTypes) {
  const NonlinearFactorGraph graph = goalGraph();
  const Controller::Chain chain = goalChain(graph);
  const Controller controller(testParams());
  const Values initial = retyped(U(1), Vector(Vector2{2.0, 0.0}));
  CHECK_EXCEPTION(
      controller.rolloutAndScore(graph, chain, Matrix::Zero(2, 6), initial),
      ValuesIncorrectType);
}

// Checks that getControl reads the control at each key, one column per key
// in the order of the keys, resizing ubar, and throws for a missing key.
TEST(VanillaMPPIController, GetControl) {
  const Controller controller(testParams());
  const Values initial = initialValues();
  Matrix ubar(5, 1);  // the wrong size, which getControl replaces
  controller.getControl(initial, {U(0), U(1), U(2)}, ubar);
  EXPECT(assert_equal(Matrix{{1.0, 2.0, 3.0}, {-1.0, 0.0, 1.0}}, ubar, 0.0));
  controller.getControl(initial, {U(2), U(0)}, ubar);
  EXPECT(assert_equal(Matrix{{3.0, 1.0}, {1.0, -1.0}}, ubar, 0.0));
  CHECK_EXCEPTION(controller.getControl(initial, {U(5)}, ubar),
                  ValuesKeyDoesNotExist);
}

// Checks that updateDynamicsFactors puts each control key's dynamics factor
// in the chain in the order of the keys, skipping the other factors and
// replacing what the chain held, and throws for a key without one.
TEST(VanillaMPPIController, UpdateDynamicsFactors) {
  const NonlinearFactorGraph graph = goalGraph();  // dynamics are factors 0-2
  const Controller controller(testParams());
  Controller::Chain chain(5);  // the wrong size, which is replaced
  controller.updateDynamicsFactors(graph, {U(2), U(0)}, chain);
  EXPECT_LONGS_EQUAL(2, chain.size());
  EXPECT(chain[0] == graph.at(2));
  EXPECT(chain[1] == graph.at(0));
  CHECK_EXCEPTION(
      controller.updateDynamicsFactors(graph, {U(0), U(3)}, chain),
      std::invalid_argument);
}

// Checks that optimize samples once in every iteration and returns initial
// with the final nominal controls and the states rolled out with them, which
// satisfy the dynamics. With only dynamics factors every sample costs 0, so
// the weights are equal and ubar becomes the mean of the samples.
TEST(VanillaMPPIController, OptimizeReturnsRollout) {
  Controller controller(testParams());
  const Values initial = initialValues();
  const Values result =
      controller.optimize(futureGraph(), initial, {U(0), U(1), U(2)});
  EXPECT(assert_equal(referenceOptimize(futureGraph(), initial, testParams()),
                      result, 1e-9));
  EXPECT_DOUBLES_EQUAL(0.0, futureGraph().error(result), 1e-12);
  EXPECT_LONGS_EQUAL(3, controller.stepCounter());
}

// Checks that optimize weights the samples by their cost: with a goal cost,
// each iteration's ubar is the weighted mean of the samples, as in the
// reference implementation.
TEST(VanillaMPPIController, OptimizeWeightsByCost) {
  Controller controller(testParams());
  const Values initial = initialValues();
  const Values result =
      controller.optimize(goalGraph(), initial, {U(0), U(1), U(2)});
  EXPECT(assert_equal(referenceOptimize(goalGraph(), initial, testParams()),
                      result, 1e-9));
  EXPECT_LONGS_EQUAL(3, controller.stepCounter());
}

// Checks that optimize lowers the cost: with a goal cost, the plan costs less
// than the initial controls rolled out.
TEST(VanillaMPPIController, OptimizeReducesCost) {
  const Values start = rolledOut(initialValues(), controlsOf(initialValues()));
  Controller controller(testParams());
  const Values result =
      controller.optimize(goalGraph(), start, {U(0), U(1), U(2)});
  EXPECT(goalGraph().error(result) < goalGraph().error(start));
}

// Checks that optimize gives the reference result when most samples are so
// far from the best one, (S_i - min S) * inverseTemperature > 709.79, that
// their weights are subnormal and are set to 0.
TEST(VanillaMPPIController, OptimizeLargeCostSpread) {
  VanillaMPPIParams params = testParams();
  params.inverseTemperature = 1000.0;
  const NonlinearFactorGraph graph = goalGraph();
  const Values initial = initialValues();

  // The first iteration's costs, from a controller with the same settings.
  const Controller probe(params);
  const Vector costs =
      probe.rolloutAndScore(graph, goalChain(graph),
                            probe.sampleControls(controlsOf(initial)), initial);
  const auto farFromBest =
      (((costs.array() - costs.minCoeff()) * params.inverseTemperature) >
       709.79)
          .count();
  EXPECT(farFromBest >= 4);

  Controller controller(params);
  const Values result = controller.optimize(graph, initial, {U(0), U(1), U(2)});
  EXPECT(assert_equal(referenceOptimize(graph, initial, params), result, 1e-9));
}

// Checks that optimize runs on a graph with a factor that is not a dynamics
// factor, a prior on X(0), which it leaves out of the chain.
TEST(VanillaMPPIController, OptimizeSkipsOtherFactors) {
  NonlinearFactorGraph graph = futureGraph();
  graph.addPrior(X(0), Pose2(1.0, 2.0, 0.3), noise);
  Controller controller(testParams());
  const Values result =
      controller.optimize(graph, initialValues(), {U(0), U(1), U(2)});
  EXPECT(result.exists(U(0)));
  EXPECT_LONGS_EQUAL(3, controller.stepCounter());
}

// Checks that optimize gives exactly the result of one thread on several
// threads, with a future graph of more than 512 factors, the size above which
// NonlinearFactorGraph::error runs in parallel: 200 steps, each with a
// dynamics factor and two priors.
TEST(VanillaMPPIController, OptimizeLargeGraph) {
  const size_t H = 200;
  NonlinearFactorGraph graph;
  Values initial;
  KeyVector controlKeys;
  initial.insert(X(0), Pose2(1.0, 2.0, 0.3));
  for (size_t k = 0; k < H; ++k) {
    graph.emplace_shared<UnicycleDynamicsFactor>(X(k), U(k), X(k + 1), model,
                                                 dt, noise);
    graph.addPrior(X(k), Pose2(5.0, 3.0, 0.5), noise);
    graph.addPrior(U(k), Vector2{0.0, 0.0}, noiseModel::Unit::Create(2));
    initial.insert(U(k), Vector2{1.0, 0.1});
    initial.insert(X(k + 1), Pose2());
    controlKeys.push_back(U(k));
  }
  EXPECT_LONGS_EQUAL(3 * H, graph.size());

  VanillaMPPIParams params = testParams();
  params.numSamples = 1024;
  Controller controller(params);
  const Values result = controller.optimize(graph, initial, controlKeys);
  EXPECT_LONGS_EQUAL(3, controller.stepCounter());
#ifdef GTSAM_USE_TBB
  const tbb::global_control oneThread(
      tbb::global_control::max_allowed_parallelism, 1);
  Controller serial(params);
  EXPECT(assert_equal(serial.optimize(graph, initial, controlKeys), result,
                      0.0));
#endif
}

// Checks that optimize needs the control covariance to sample. The graph has
// every dynamics factor, so only the sampler can throw.
TEST(VanillaMPPIController, OptimizeNeedsCovariance) {
  Controller controller;
  CHECK_EXCEPTION(controller.optimize(futureGraph(), initialValues(),
                                      {U(0), U(1), U(2)}),
                  std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, controller.stepCounter());
}

// Checks that optimize fails before sampling if a control key has no
// dynamics factor in the future graph.
TEST(VanillaMPPIController, OptimizeMissingDynamicsFactor) {
  Controller controller(testParams());
  Values initial = initialValues();
  initial.insert(U(3), Vector2{4.0, 2.0});
  CHECK_EXCEPTION(controller.optimize(futureGraph(), initial,
                                      {U(0), U(1), U(2), U(3)}),
                  std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, controller.stepCounter());
}

// Checks that optimize rejects an empty list of control keys before sampling.
TEST(VanillaMPPIController, OptimizeNoControlKeys) {
  Controller controller(testParams());
  CHECK_EXCEPTION(
      controller.optimize(futureGraph(), initialValues(), KeyVector()),
      std::invalid_argument);
  EXPECT_LONGS_EQUAL(0, controller.stepCounter());
}

// Checks that optimize fails before sampling if a control key is missing.
TEST(VanillaMPPIController, OptimizeMissingControl) {
  Controller controller(testParams());
  CHECK_EXCEPTION(controller.optimize(NonlinearFactorGraph(), threeControls(),
                                      {U(0), U(5)}),
                  ValuesKeyDoesNotExist);
  EXPECT_LONGS_EQUAL(0, controller.stepCounter());
}

// Checks that optimize fails before sampling if a control is stored with
// another type, here a dynamic Vector, which getControl still reads.
TEST(VanillaMPPIController, OptimizeWrongControlType) {
  Controller controller(testParams());
  const Values initial = retyped(U(1), Vector(Vector2{2.0, 0.0}));
  CHECK_EXCEPTION(
      controller.optimize(futureGraph(), initial, {U(0), U(1), U(2)}),
      ValuesIncorrectType);
  EXPECT_LONGS_EQUAL(0, controller.stepCounter());
}

}  // namespace vanilla_mppi_optimize
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.

/**
 * @file    testSamplingPlanner.cpp
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/planning/SamplingPlanner.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <limits>
#include <thread>

using namespace gtsam;
using symbol_shorthand::U;
using symbol_shorthand::X;

// A field in the moving tangent basis of S² is evaluated again at each state.
TEST(SamplingPlanner, SphereRollout) {
  const Unit3 start(1, 0, 0);
  const auto field = [](const Unit3& direction,
                        const Vector3& omega) -> Vector2 {
    return direction.basis().transpose() * direction.cross(omega);
  };
  Values initial;
  initial.insert(X(0), start);
  const SamplingPlanner<Unit3, Vector3> planner({X(0), X(1), X(2)},
                                                {U(0), U(1)}, 0.1);
  const Matrix controls{{0, 1}, {0, 0}, {1, 0}};
  const Values plan = planner.rollout(initial, controls, field);
  const Unit3 first =
      start.retract(Vector2(0.1 * field(start, controls.col(0))));
  const Unit3 second =
      first.retract(Vector2(0.1 * field(first, controls.col(1))));
  EXPECT(assert_equal(first, plan.at<Unit3>(X(1))));
  EXPECT(assert_equal(second, plan.at<Unit3>(X(2))));
  EXPECT(!initial.exists(X(1)));
}

// Reject ambiguous horizons before evaluating a user's vector field.
TEST(SamplingPlanner, InvalidLayout) {
  using Planner = SamplingPlanner<Vector1, Vector1>;
  CHECK_EXCEPTION(Planner({X(0)}, {}, 0.1), std::invalid_argument);
  CHECK_EXCEPTION(Planner({X(0), X(1)}, {X(1)}, 0.1), std::invalid_argument);
  CHECK_EXCEPTION(Planner({X(0), X(1)}, {U(0)}, 0.0), std::invalid_argument);
}

// State and control vector dimensions can differ and be known only at runtime.
TEST(SamplingPlanner, DynamicVectorField) {
  const SamplingPlanner<Vector, Vector> planner({X(0), X(1)}, {U(0)}, 0.1);
  Values initial;
  initial.insert(X(0), Vector::Zero(3).eval());
  initial.insert(U(0), Vector::Ones(2).eval());
  const auto field = [](const Vector&, const Vector& u) -> Vector {
    return Vector3{u[0], u[1], 0.};
  };
  NonlinearFactorGraph graph;
  graph.addPrior<Vector>(X(1), Vector::Zero(3), noiseModel::Unit::Create(3));
  const auto plan =
      planner.optimize(graph, initial, field,
                       GaussianProposal(Matrix2::Identity(), 8), MPPI(), 1);
  const Vector control = plan.at<Vector>(U(0));
  EXPECT(assert_equal(Vector3{0.1 * control[0], 0.1 * control[1], 0.},
                      plan.at<Vector>(X(1))));
  const auto wrongDimension = [](const Vector&, const Vector& u) { return u; };
  CHECK_EXCEPTION(planner.rollout(initial, Matrix::Ones(2, 1), wrongDimension),
                  std::invalid_argument);
}

// A discrete transition returns the next state without integration.
TEST(SamplingPlanner, DiscreteTransitionRollout) {
  const SamplingPlanner<Vector1, Vector1> planner({X(0), X(1), X(2)},
                                                  {U(0), U(1)}, 0.05);
  Values initial;
  initial.insert(X(0), Vector1{2.});
  initial.insert(100, 7.);
  const Matrix controls{{3., -1.}};
  const auto step = [](const Vector1& x, const Vector1& u) -> Vector1 {
    return x + u;
  };
  const Values plan =
      planner.rollout(initial, controls, discreteTransition(step));
  EXPECT(assert_equal(Vector1{5.}, plan.at<Vector1>(X(1))));
  EXPECT(assert_equal(Vector1{4.}, plan.at<Vector1>(X(2))));
  EXPECT_DOUBLES_EQUAL(7., plan.at<double>(100), 0.);
}

// Thread-affine transitions must run on the caller even with parallel scoring.
TEST(SamplingPlanner, TransitionOnCallingThread) {
  const auto caller = std::this_thread::get_id();
  const auto step = discreteTransition(
      [caller](const Vector1& x, const Vector1& u) -> Vector1 {
        if (std::this_thread::get_id() != caller)
          throw std::runtime_error("Transition evaluated on another thread.");
        return x + u;
      },
      false);
  SamplingPlanner<Vector1, Vector1> planner({X(0), X(1)}, {U(0)}, 0.1, true);
  NonlinearFactorGraph graph;
  graph.addPrior<Vector1>(X(1), Vector1{1.}, noiseModel::Unit::Create(1));
  Values initial;
  initial.insert(X(0), Vector1{0.});
  const Matrix controls = Matrix::Ones(1, 64);
  EXPECT(assert_equal(Vector::Zero(64).eval(),
                      planner.rolloutAndScore(graph, initial, controls, step)));
}

namespace prior_scoring {

/// A derived prior delegates its activation decision to the Values overload.
class InactivePrior : public PriorFactor<Vector2> {
 public:
  using PriorFactor<Vector2>::PriorFactor;
  bool active(const Values&) const override { return false; }
};

// Robust priors and subclass overrides must agree with full graph evaluation.
TEST(SamplingPlanner, PriorScoring) {
  SamplingPlanner<Vector2, Vector2> planner({X(0), X(1)}, {U(0)}, 0.5);
  const auto field = [](const Vector2&, const Vector2& u) { return u; };
  const auto noise = noiseModel::Unit::Create(2);
  const auto huber = noiseModel::mEstimator::Huber::Create(
      0.1, noiseModel::mEstimator::Base::Scalar);
  NonlinearFactorGraph graph;
  graph.addPrior<Vector2>(X(1), Vector2{3., 4.},
                          noiseModel::Robust::Create(huber, noise));
  graph.emplace_shared<InactivePrior>(U(0), Vector2{4., 5.}, noise);
  Values initial;
  initial.insert(X(0), Vector2{0., 0.});
  const Matrix controls{{1., 3.}, {-2., 0.}};
  const Vector costs = planner.rolloutAndScore(graph, initial, controls, field);
  for (Eigen::Index i = 0; i < controls.cols(); ++i)
    EXPECT_DOUBLES_EQUAL(
        graph.error(planner.rollout(initial, controls.col(i), field)), costs[i],
        0.);
  graph.addPrior<Vector2>(X(1), Vector2{1., 2.}, Matrix3::Identity());
  CHECK_EXCEPTION(planner.rolloutAndScore(graph, initial, controls, field),
                  std::invalid_argument);
}

}  // namespace prior_scoring

// A seeded draw uses Akash's per-candidate Gaussian streams, including repeated
// calls.
TEST(GaussianProposal, SeededDrawsMatchSampler) {
  const Matrix mean{{0.1, 0.3}, {1.0, -2.0}};
  const Matrix2 covariance{{1.0, 0.2}, {0.2, 0.3}};
  const GaussianNoiseSampler sampler(covariance, 7, 4321);
  const GaussianProposal proposal(covariance, 7, 4321);
  const auto search = proposal.initialize(mean);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT(assert_equal(sampler.sampleControls(mean),
                        proposal.sample(search, Values(), 0.1).controls, 0.0));
  }
  EXPECT(assert_equal(
      mean, proposal.sample(search, Values(), 0.1, true).controls.leftCols(2)));
}

// Check two iterations against an explicit rollout, graph error, and weighted
// average. The graph includes a factor on nonadjacent states and an auxiliary
// key.
TEST(SamplingPlanner, FactoredGaussianMPPI) {
  const Key anchor = 77;
  const SamplingPlanner<Vector1, Vector1> planner({X(0), X(1), X(2)},
                                                  {U(0), U(1)}, 0.25);
  const auto field = [](const Vector1&, const Vector1& u) { return u; };
  Values initial;
  initial.insert(X(0), Vector1{0.5});
  initial.insert(U(0), Vector1{0.2});
  initial.insert(U(1), Vector1{-0.1});
  initial.insert(anchor, Vector1{1.5});
  NonlinearFactorGraph graph;
  graph.addPrior<Vector1>(U(1), Vector1{0.4}, noiseModel::Unit::Create(1));
  graph.emplace_shared<BetweenFactor<Vector1>>(X(0), X(2), Vector1{1.0},
                                               noiseModel::Unit::Create(1));
  graph.emplace_shared<BetweenFactor<Vector1>>(X(1), anchor, Vector1{0.2},
                                               noiseModel::Unit::Create(1));

  const GaussianProposal gaussian(Matrix1::Identity(), 8, 917);
  const GaussianProposal reference(Matrix1::Identity(), 8, 917);
  const MPPI algorithm(2.0);
  auto expected = reference.initialize(planner.nominalControls(initial));
  for (size_t n = 0; n < 2; ++n) {
    const auto batch = reference.sample(expected, initial, 0.25);
    Vector errors(8);
    for (Eigen::Index j = 0; j < 8; ++j) {
      const auto controls = batch.controls.middleCols(2 * j, 2);
      const Values values = planner.rollout(initial, controls, field);
      errors[j] = graph.error(values);
    }
    EXPECT(assert_equal(errors, planner.rolloutAndScore(
                                    graph, initial, batch.controls, field)));
    const Vector weights = (-2. * errors.array()).exp();
    Matrix mean = Matrix::Zero(1, 2);
    for (Eigen::Index j = 0; j < 8; ++j)
      mean +=
          (weights[j] / weights.sum()) * batch.controls.middleCols(2 * j, 2);
    expected.mean = mean;
  }
  const Values result =
      planner.optimize(graph, initial, field, gaussian, algorithm, 2);
  EXPECT(assert_equal(planner.rollout(initial, expected.mean, field), result,
                      1e-12));

  const GaussianProposal repeated(Matrix1::Identity(), 8, 917);
  auto search = repeated.initialize(planner.nominalControls(initial));
  for (size_t n = 0; n < 2; ++n)
    planner.iterate(graph, initial, field, repeated, algorithm, search);
  EXPECT(assert_equal(expected.mean, search.mean, 1e-12));
  EXPECT(assert_equal(
      planner.rollout(initial, planner.nominalControls(initial), field),
      planner.optimize(graph, initial, field, repeated, algorithm, 0)));
}

// Weights remain stable under a large common offset and skip rejected nonfinite
// controls.
TEST(MPPI, StableWeightsAndRejectedControls) {
  const double inf = std::numeric_limits<double>::infinity();
  const Vector3 costs{10000., 10001., inf};
  EXPECT(assert_equal(Vector3{.5, .5, 0.}, MPPI(0.).weights(costs)));
  EXPECT(
      assert_equal(Vector3{.75, .25, 0.}, MPPI(std::log(3.)).weights(costs)));
  CHECK_EXCEPTION(MPPI().weights(Vector1{inf}), std::domain_error);
  CHECK_EXCEPTION(MPPI().weights(Vector{}), std::invalid_argument);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const GaussianProposal proposal(Matrix1::Identity(), 2);
  auto search = proposal.initialize(Matrix::Zero(1, 2));
  const ControlBatch batch{Matrix{{nan, nan, 3., 4.}}};
  MPPI().apply(proposal, search, batch, Vector2{inf, 0.});
  EXPECT(assert_equal(Matrix{{3., 4.}}, search.mean));
}

TEST(SamplingPlanner, EliteFitAndIncumbent) {
  const GaussianProposal proposal(Matrix1::Identity(), 3, 17);
  auto search = proposal.initialize(Matrix::Zero(1, 2));
  const ControlBatch batch{Matrix{{0., 1., 2., 1., 8., 9.}}};
  const Vector3 costs{2., 2., 8.};
  CEM(2, .25).apply(proposal, search, batch, costs);
  EXPECT(assert_equal(Matrix{{1., 1.}}, search.mean));
  EXPECT(assert_equal(Matrix{{1., .25}}, search.stddev));
  auto next =
      proposal.sample(search, {}, 1., PredictiveSampling::includeIncumbent);
  EXPECT(assert_equal(search.mean, Matrix(next.controls.leftCols(2))));
  EXPECT(
      assert_equal(Vector3{1., 0., 0.}, PredictiveSampling().weights(costs)));
  CHECK_EXCEPTION(GaussianProposal(Matrix1::Identity(), 0),
                  std::invalid_argument);
  CHECK_EXCEPTION(CEM(4).weights(costs), std::invalid_argument);
}

namespace candidate_scoring {

/// A cost with configurable thread affinity, like a user callback or cached
/// factor.
class CheckedPrior : public PriorFactor<Vector1> {
 public:
  CheckedPrior(bool sendable, bool requireCaller)
      : PriorFactor<Vector1>(symbol_shorthand::X(1), Vector1{1.},
                             noiseModel::Unit::Create(1)),
        sendable_(sendable),
        requireCaller_(requireCaller) {}
  bool sendable() const override { return sendable_; }
  double error(const Values& values) const override {
    if (requireCaller_ && std::this_thread::get_id() != caller_)
      throw std::runtime_error(
          "Cost factor evaluated outside the calling thread.");
    return PriorFactor<Vector1>::error(values);
  }

 private:
  std::thread::id caller_ = std::this_thread::get_id();
  bool sendable_, requireCaller_;
};

// Repeated factor instances and non-sendable callbacks retain thread affinity,
// including when a graph is large enough for nested graph-level TBB scheduling.
TEST(SamplingPlanner, CandidateScoringThreadAffinity) {
  Values initial;
  initial.insert(X(0), Vector1{0.});
  initial.insert(U(0), Vector1{0.});
  auto solve = [&](bool optIn, bool sendable) {
    SamplingPlanner<Vector1, Vector1> planner({X(0), X(1)}, {U(0)}, 1., optIn);
    NonlinearFactorGraph graph;
    auto prior = std::make_shared<CheckedPrior>(sendable, !optIn || !sendable);
    for (size_t i = 0; i < 513; ++i) graph.push_back(prior);
    GaussianProposal gaussian(Matrix1::Identity(), 64, 42);
    return planner.optimize(
        graph, initial, [](const Vector1&, const Vector1& u) { return u; },
        gaussian, PredictiveSampling(), 1);
  };
  const Values expected = solve(false, true);
  EXPECT(assert_equal(expected, solve(true, false)));
  EXPECT(assert_equal(expected, solve(true, true)));
}

}  // namespace candidate_scoring
/* ************************************************************************* */

/* ************************************************************************* */
namespace configurable_proposals {

// Shared 2-step scalar integrator helpers to avoid repeating test setup.
Vector1 integrator(const Vector1&, const Vector1& u) { return u; }

Values integratorValues(double x0, double u0 = 0., double u1 = 0.) {
  Values values;
  values.insert(X(0), Vector1{x0});
  values.insert(U(0), Vector1{u0});
  values.insert(U(1), Vector1{u1});
  return values;
}

NonlinearFactorGraph terminalPriorGraph(double target) {
  NonlinearFactorGraph graph;
  graph.addPrior(X(2), Vector1{target}, noiseModel::Unit::Create(1));
  return graph;
}

// Repeated iterate calls and optimize use the same draws, retained state and
// manifold rollout for each supported proposal and search pairing.
TEST(SamplingPlanner, IndependentProposalsAndAlgorithms) {
  const SamplingPlanner<Vector1, Vector1> planner({X(0), X(1), X(2)},
                                                  {U(0), U(1)}, 0.5);
  const Values initial = integratorValues(0.);
  const NonlinearFactorGraph graph = terminalPriorGraph(1.);
  const auto compare = [&](auto proposal, const auto& algorithm) {
    auto sameSeed = proposal;
    const auto plan =
        planner.optimize(graph, initial, integrator, sameSeed, algorithm, 3);
    auto search = proposal.initialize(planner.nominalControls(initial));
    for (int i = 0; i < 3; ++i)
      planner.iterate(graph, initial, integrator, proposal, algorithm, search);
    EXPECT_LONGS_EQUAL(3, search.iteration);
    EXPECT(
        assert_equal(plan, planner.rollout(initial, search.mean, integrator)));
    EXPECT(graph.error(plan) < graph.error(planner.rollout(
                                   initial, Matrix::Zero(1, 2), integrator)));
  };
  const auto compareFittable = [&](const auto& proposal) {
    compare(proposal, MPPI());
    compare(proposal, PredictiveSampling());
    compare(proposal, CEM(8));
  };
  compareFittable(GaussianProposal(Matrix1::Identity(), 64));
  compareFittable(ColoredNoiseProposal(Matrix1::Identity(), 64, 2.));
  compareFittable(NormalLognormalProposal(Matrix1::Identity(), 64, 0.6));
  compareFittable(SmoothControlProposal(Matrix1::Identity(), 64));
  const PiecewiseLinearProposal linear(Matrix1::Identity(), 64, 1);
  compare(linear, MPPI());
  compare(linear, PredictiveSampling());
}

// Compare a corrected MPPI iteration to a hand-computed double integrator
// using the Gaussian sampler's MPPI-Generic likelihood cost convention.
TEST(SamplingPlanner, GaussianLikelihoodUpdate) {
  struct FixedDraws : GaussianProposal {
    using GaussianProposal::GaussianProposal;
    Matrix drawNoise(Eigen::Index) const override {
      return Matrix{{0., 0., 1., -2., -2., -1.}};
    }
  };
  GaussianProposalOptions options;
  options.likelihoodCoefficients = Vector1{0.5};  // Horizon normalization.
  FixedDraws proposal(Matrix1::Identity(), 3, 42, options);
  const SamplingPlanner<Vector2, Vector1> planner({X(0), X(1), X(2)},
                                                  {U(0), U(1)}, 1.);
  const auto dynamics = [](const Vector2& x, const Vector1& u) {
    return Vector2{x.y(), u.x()};
  };
  Values initial;
  initial.insert(X(0), Vector2(0., 0.));
  initial.insert(U(0), Vector1{1.});
  initial.insert(U(1), Vector1{2.});
  NonlinearFactorGraph graph;
  graph.addPrior(X(2), Vector2(1., 2.),
                 noiseModel::Isotropic::Sigma(2, std::sqrt(2.)));
  auto state = proposal.initialize(planner.nominalControls(initial));
  const auto batch = proposal.sample(state, initial, 1., false);
  EXPECT(assert_equal((Matrix{{1., 2., 2., 0., -1., 1.}}), batch.controls));
  EXPECT(assert_equal(Vector3(-1.25, 0.25, 0.75), batch.logWeightOffsets));
  const Vector3 costs{0.25, 0.25, 2.};
  for (Eigen::Index j = 0; j < 3; ++j)
    EXPECT_DOUBLES_EQUAL(
        costs[j],
        graph.error(planner.rollout(
            initial, batch.controls.middleCols(2 * j, 2), dynamics)),
        1e-12);
  Vector3 weights{std::exp(-1.5), 1., std::exp(-1.25)};
  weights /= weights.sum();
  EXPECT(assert_equal(weights, MPPI().weights(costs, batch.logWeightOffsets)));
  planner.iterate(graph, initial, dynamics, proposal, MPPI(), state);
  const Matrix expected{
      {weights[0] + 2 * weights[1] - weights[2], 2 * weights[0] + weights[2]}};
  EXPECT(assert_equal(expected, state.mean));
}

// Hold the normal draws fixed to isolate time-varying spread, decay, and
// centering.
TEST(ControlProposals, GaussianOptions) {
  GaussianProposalOptions options;
  options.stddev = Matrix{{2., 3.}};
  options.decay = 0.5;
  options.zeroMeanFraction = 0.5;
  options.includeNominal = true;
  GaussianProposal proposal(Matrix1::Identity(), 4, 7, options);
  const Matrix nominal{{3., 4.}};
  auto state = proposal.initialize(nominal);
  state.iteration = 1;
  auto controls = proposal.sample(state, {}, 1., false).controls;
  GaussianNoiseSampler white(Matrix1::Identity(), 4, 7);
  Matrix expected =
      white.sampleNoise(2).cwiseProduct(options.stddev.replicate(1, 4)) * 0.5;
  expected.leftCols(2) = nominal;
  expected.middleCols(2, 2) += nominal;
  EXPECT(assert_equal(expected, controls));
}

// Power-law coloring preserves marginal scale and attenuates fast changes.
TEST(ControlProposals, SpectralColor) {
  const Matrix nominal = Matrix::Zero(1, 31);
  ColoredNoiseProposal colored(Matrix1::Identity(), 128, 2.);
  GaussianProposal white(Matrix1::Identity(), 128);
  auto state = white.initialize(nominal);
  const Matrix smooth = colored.sample(state, {}, 1., false).controls;
  const Matrix raw = white.sample(state, {}, 1., false).controls;
  const auto energy = [](const Matrix& samples) {
    double total = 0.;
    for (Eigen::Index j = 0; j < 128; ++j)
      total +=
          (samples.middleCols(31 * j + 1, 30) - samples.middleCols(31 * j, 30))
              .squaredNorm();
    return total;
  };
  EXPECT(energy(smooth) < 0.3 * energy(raw));
  EXPECT(std::abs(smooth.squaredNorm() / smooth.size() - 1.) < 0.15);
  EXPECT(colored.sample(colored.initialize(Matrix::Zero(1, 1)), {}, 1., false)
             .controls.allFinite());
}

// The normalized normal-lognormal product uses independent seeded streams.
TEST(ControlProposals, NormalLognormal) {
  const Matrix nominal = Matrix::Zero(1, 5);
  NormalLognormalProposal proposal(Matrix1::Identity(), 8, 0.5, 123);
  GaussianNoiseSampler white(Matrix1::Identity(), 8, 123);
  GaussianNoiseSampler logScale(Matrix1::Identity(), 8, ~std::uint64_t(123));
  const Matrix expected = white.sampleNoise(5).array() *
                          (0.5 * logScale.sampleNoise(5).array() - 0.25).exp();
  EXPECT(assert_equal(
      expected,
      proposal.sample(proposal.initialize(nominal), {}, 1., false).controls));
}

// Linear interpolation gives at most one interior change of slope for two
// segments.
TEST(ControlProposals, PiecewiseLinear) {
  const Matrix nominal = Matrix::Constant(1, 12, 2.);
  PiecewiseLinearProposal proposal(Matrix1::Identity(), 8, 2);
  const auto batch =
      proposal.sample(proposal.initialize(nominal), {}, 1., true);
  EXPECT(assert_equal(nominal, Matrix(batch.controls.leftCols(12))));
  for (Eigen::Index j = 1; j < 8; ++j) {
    const auto u = batch.controls.middleCols(12 * j, 12);
    EXPECT_LONGS_EQUAL(
        1, ((u.leftCols(10) - 2 * u.middleCols(1, 10) + u.rightCols(10))
                .array()
                .abs() > 1e-10)
               .count());
  }
}

// The next draw uses the fitted derivative mean; CEM estimates spread in
// derivative units.
TEST(ControlProposals, SmoothCoordinates) {
  const Matrix nominal{{3., 4.}};
  SmoothControlProposal proposal(Matrix1::Identity(), 3, 42);
  auto state = proposal.initialize(nominal);
  const auto first = proposal.sample(state, {}, 0.2, false);
  EXPECT(assert_equal(Matrix(nominal.replicate(1, 3) + 0.2 * first.derivatives),
                      first.controls));
  CEM(2, 0.01).apply(proposal, state, first, Vector3{0., 9., 1.});
  const Matrix fitted =
      (first.derivatives.leftCols(2) + first.derivatives.rightCols(2)) / 2.;
  EXPECT(assert_equal(fitted, state.derivativeMean));
  const Matrix spread =
      (first.derivatives.leftCols(2) - fitted).array().abs().max(0.01);
  EXPECT(assert_equal(spread, state.stddev));
  EXPECT(assert_equal(Matrix(nominal + 0.2 * fitted), state.mean));
  GaussianNoiseSampler white(Matrix1::Identity(), 3, 42);
  white.sampleNoise(2);
  Matrix expected = fitted.replicate(1, 3) +
                    spread.replicate(1, 3).cwiseProduct(white.sampleNoise(2));
  expected.leftCols(2).setZero();
  const auto second = proposal.sample(state, {}, 0.2, true);
  EXPECT(assert_equal(expected, second.derivatives));
  EXPECT(assert_equal(state.mean, Matrix(second.controls.leftCols(2))));
}

struct FeedbackProposal : GaussianProposal {
  using GaussianProposal::GaussianProposal;
  struct Batch : ControlBatch {
    Vector1 operator()(Eigen::Index j, size_t k, const Vector1& x) const {
      if (j == 0) return controls.col(static_cast<Eigen::Index>(k));
      return Vector1{-0.5 * x.x()};
    }
  };
  Batch sample(const State& state, const Values&, double, bool) const {
    return {{state.mean.replicate(1, 2), {}}};
  }
};

// A feedback candidate sees its own predicted state and the winning realized
// controls.
TEST(SamplingPlanner, FeedbackCandidate) {
  FeedbackProposal proposal(Matrix1::Identity(), 2);
  const SamplingPlanner<Vector1, Vector1> planner({X(0), X(1), X(2)},
                                                  {U(0), U(1)}, 1.);
  const Values initial = integratorValues(4.);
  const NonlinearFactorGraph graph = terminalPriorGraph(0.);
  const auto plan = planner.optimize(graph, initial, integrator, proposal,
                                     PredictiveSampling(), 1);
  EXPECT(assert_equal(Vector1{-2.}, plan.at<Vector1>(U(0))));
  EXPECT(assert_equal(Vector1{-1.}, plan.at<Vector1>(U(1))));
  EXPECT(assert_equal(Vector1{1.}, plan.at<Vector1>(X(2))));
}

}  // namespace configurable_proposals
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

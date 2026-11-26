// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
/**
 * @file testMPPITracking.cpp
 * @author Frank Dellaert
 * @author Akash Ratheesh
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/planning/MPPITracking.h>
#include <gtsam/planning/TrackingFeedback.h>
#include <gtsam/slam/PriorFactor.h>

using namespace gtsam;

/* ************************************************************************* */
namespace weights {

// Costs at or beyond the Tsallis cutoff have exactly zero mass.
TEST(TsallisMPPI, Cutoff) {
  const TsallisMPPI update(2., 2.);
  EXPECT(assert_equal(Vector3(2. / 3, 1. / 3, 0.),
                      update.weights(Vector3(4., 5., 6.))));
  CHECK_EXCEPTION(TsallisMPPI(0, 2), std::invalid_argument);
}

// Upstream interpolates the quantile and adds it to the upper-tail average.
TEST(TrajectoryRisk, EmpiricalQuantiles) {
  const Vector costs = (Vector(4) << 8., 0., 2., 4.).finished();
  EXPECT_DOUBLES_EQUAL(3., valueAtRisk(costs, 0.5), 1e-12);
  EXPECT_DOUBLES_EQUAL(5., conditionalValueAtRisk(costs, 0.5), 1e-12);
  EXPECT_DOUBLES_EQUAL(4., conditionalValueAtRisk(costs, 1. / 3.), 1e-12);
  EXPECT_DOUBLES_EQUAL(8., conditionalValueAtRisk(costs, 1.), 1e-12);
  CHECK_EXCEPTION(valueAtRisk(costs, -0.1), std::invalid_argument);
}

// Mixture slots, perturbation bounds and fixed input prefix survive sampling.
TEST(PiecewiseLinearProposal, MotionPrimitives) {
  MotionPrimitiveOptions mixture;
  mixture.amplitude = Vector1{2.};
  mixture.coloredFraction = .3;
  mixture.shiftedFraction = .3;
  mixture.coloredScale = 0.;
  mixture.shiftedScale = .5;
  mixture.fixedPrefix = 1;
  PiecewiseLinearProposal proposal(Matrix1::Identity(), 10, 3, mixture, 9);
  Matrix nominal = Matrix::Constant(1, 16, 8.);
  auto state = proposal.initialize(nominal);
  auto batch = proposal.sample(state, {}, 0.1, true);
  // 0 = incumbent, 1 = zero after prefix, 2 = colored with zero scale.
  EXPECT(assert_equal(nominal, Matrix(batch.controls.leftCols(16))));
  EXPECT(batch.controls.middleCols(17, 15).isZero());
  EXPECT(assert_equal(nominal, Matrix(batch.controls.middleCols(32, 16))));
  for (Eigen::Index k = 0; k < 10; ++k)
    EXPECT_DOUBLES_EQUAL(8., batch.controls(0, 16 * k), 0.);
  EXPECT((batch.controls.middleCols(49, 15).array() - 8.).abs().maxCoeff() <=
         1.);
  // An absolute curve starts at the incumbent and heads toward bounded knots.
  EXPECT_DOUBLES_EQUAL(8., batch.controls(0, 16 * 6 + 1), 0.);
  mixture.fixedPrefix = 0;
  PiecewiseLinearProposal fixed(Matrix1::Identity(), 10, 3, mixture, 9);
  batch = fixed.sample(state, {}, .1, true);
  EXPECT(batch.controls.rightCols(4 * 16).cwiseAbs().maxCoeff() <= 2.);
}
}  // namespace weights
/* ************************************************************************* */

/* ************************************************************************* */
namespace tracking {

// A centered five-point quadratic filter exactly preserves quadratics away from
// the padded right edge.
TEST(TrackingFeedback, SavitzkyGolay) {
  const Matrix controls{{0., 1., 4., 9., 16.}}, history{{4., 1.}};
  const Matrix result = smoothControlTrajectory(controls, history);
  EXPECT(
      assert_equal(Matrix(controls.leftCols(3)), Matrix(result.leftCols(3))));
  EXPECT_DOUBLES_EQUAL((-12. + 108. + 272. + 192. - 48.) / 35., result(0, 4),
                       1e-12);
}

// The minimum-norm correction satisfies the requested contraction equality.
TEST(TrackingFeedback, Contraction) {
  EXPECT(assert_equal(
      Vector1{-2.}, contractionFeedback(Vector1{1.}, Matrix1::Identity(),
                                        Matrix1::Identity(), Vector1{1.}, 1.)));
  EXPECT(assert_equal(
      Vector1{0.}, contractionFeedback(Vector1{1.}, Matrix1::Identity(),
                                       Matrix1::Identity(), Vector1{-2.}, 1.)));
}

// A one-step linear-quadratic problem has a closed-form optimal input and gain.
TEST(TrackingFeedback, DDP) {
  DDPTracking solver(Matrix1::Zero(), Matrix1::Identity(),
                     Matrix1::Identity() * 3.);
  const Matrix x{{0., 2.}}, u{{0.}};
  Matrix controls = solver.optimize(
      Vector1{0.}, x, u,
      [](const Vector& x, const Vector& u) -> Vector { return x + u; }, 2);
  EXPECT_DOUBLES_EQUAL(1.5, controls(0, 0), 1e-6);
  EXPECT_DOUBLES_EQUAL(1.5, solver.states()(0, 1), 1e-6);
  EXPECT_DOUBLES_EQUAL(-0.75, solver.gain(0)(0, 0), 1e-6);
  controls = solver.optimize(
      Vector1{0.}, x, u,
      [](const Vector& x, const Vector& u) -> Vector { return x + u; }, 2,
      Vector1{-0.5}, Vector1{0.5});
  EXPECT_DOUBLES_EQUAL(0.5, controls(0, 0), 1e-10);
  EXPECT_DOUBLES_EQUAL(0.5, solver.states()(0, 1), 1e-10);
}

// The final backward sweep resets damping to the minimum so gain(0) matches
// the exact local sensitivity du0*/dx0 even when earlier iterations damped.
TEST(TrackingFeedback, DDPFinalGainUndamped) {
  const double dt = 0.05;
  const auto pendulum = [dt](const Vector& x, const Vector& u) -> Vector {
    return Vector2(x[0] + dt * x[1],
                   x[1] + dt * (-9.81 * std::sin(x[0]) + u[0]));
  };
  const Eigen::Index h = 25;
  Matrix targetStates = Matrix::Zero(2, h + 1);
  targetStates.row(0).setConstant(M_PI);
  const Matrix targetControls = Matrix::Zero(1, h);
  const Matrix q = Vector2(2., 0.2).asDiagonal();
  const Matrix r = Matrix1::Identity() * 0.1;
  const Matrix terminal = Vector2(20., 2.).asDiagonal();
  DDPTracking solver(q, r, terminal);
  const Vector2 x0(0.2, 0.0);
  const Matrix uStar =
      solver.optimize(x0, targetStates, targetControls, pendulum, 50);
  const Matrix k0 = solver.gain(0);
  // Compare gain(0) against finite differences of the re-solved optimum.
  const double eps = 1e-4;
  for (int i = 0; i < 2; ++i) {
    Vector2 dx = Vector2::Zero();
    dx[i] = eps;
    DDPTracking plus(q, r, terminal), minus(q, r, terminal);
    const double uPlus = plus.optimize(x0 + dx, targetStates, targetControls,
                                       pendulum, 50)(0, 0);
    const double uMinus = minus.optimize(x0 - dx, targetStates, targetControls,
                                         pendulum, 50)(0, 0);
    const double fd = (uPlus - uMinus) / (2 * eps);
    EXPECT_DOUBLES_EQUAL(fd, k0(0, i), 1e-3);
  }
}
}  // namespace tracking
/* ************************************************************************* */

/* ************************************************************************* */
namespace controllers {
using Planner = SamplingPlanner<Vector1, Vector1>;

KeyVector states() { return {0, 1, 2}; }
KeyVector inputs() { return {3, 4}; }
Vector1 dynamics(const Vector1&, const Vector1& u) { return u; }
Vector1 feedback(size_t, const Vector1& x, const Vector1& ref, const Vector1&) {
  return Vector1(ref - x);
}

// Shared builders for the 2-step scalar tracking fixtures below.
Planner makePlanner(double dt = 1.) { return Planner(states(), inputs(), dt); }
MPPITracking makeTracking(size_t samples = 16) {
  return MPPITracking(Matrix1::Identity(), samples, 1.);
}

Values initial(double start) {
  Values values;
  for (Key key : states()) values.insert(key, Vector1{start});
  for (Key key : inputs()) values.insert(key, Vector1{0.});
  return values;
}

NonlinearFactorGraph cost() {
  NonlinearFactorGraph graph;
  graph.addPrior(states().front(), Vector1{0.}, noiseModel::Unit::Create(1));
  return graph;
}

// Tube uses the measured system when its baseline improves and otherwise tracks
// the old tube.
TEST(MPPITracking, Tube) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking();

  // Measured start (1) beats nominal (4) -> switches to candidate 0.
  auto result = tracking.tube(planner, cost(), initial(4.), initial(1.),
                              dynamics, feedback, 0., 1);
  EXPECT_LONGS_EQUAL(0, result.candidate);
  EXPECT(assert_equal(Vector1{1.}, result.nominal.at<Vector1>(0)));

  // Nominal start (1) beats measured (4) -> keeps candidate 1 and tracks it.
  auto old = tracking.tube(planner, cost(), initial(1.), initial(4.), dynamics,
                           feedback, 0., 1);
  EXPECT_LONGS_EQUAL(1, old.candidate);
  EXPECT(assert_equal(old.nominal.at<Vector1>(1), old.actual.at<Vector1>(1)));
}

// Passing a non-Gaussian proposal through its base preserves both coupled
// branches and advances the caller's proposal by one batch per iteration.
TEST(MPPITracking, TubeReplaysDynamicGaussianType) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking();
  NormalLognormalProposal heavyTails(Matrix1::Identity(), 7, 0.8, 42);
  const GaussianProposal& distribution = heavyTails;
  NormalLognormalProposal independent = heavyTails;

  auto result = tracking.tube(planner, cost(), initial(1.), initial(1.),
                              dynamics, feedback, distribution, -1., 2);
  auto expected =
      planner.optimize(cost(), initial(1.), dynamics, independent, MPPI(), 2);
  EXPECT_LONGS_EQUAL(1, result.candidate);
  EXPECT(assert_equal(expected, result.nominal, 1e-10));

  // Verify the caller's RNG stream advanced identically to the open-loop run.
  auto search = heavyTails.initialize(Matrix::Zero(1, 2));
  EXPECT(assert_equal(heavyTails.sample(search, {}, 1., false).controls,
                      independent.sample(search, {}, 1., false).controls));
}

struct ConditionedProposal : GaussianProposal {
  using GaussianProposal::GaussianProposal;
  ControlBatch sample(const State& state, const Values& initial, double,
                      bool) const {
    const auto count = initial.at<Vector1>(states()[0])[0] < 1. ? 2 : 1;
    return {Matrix::Constant(1, count * state.mean.cols(), 3.), {}};
  }
  void update(State& state, const ControlBatch& batch,
              const Vector& weights) const {
    GaussianProposal::update(state, batch, weights);
    state.mean.array() += 1.;
  }
};

// Tube replay preserves a concrete proposal's sampling and update operations.
// A context-dependent count mismatch would leave some innovations unpaired.
TEST(MPPITracking, TubeCustomProposal) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking(4);
  ConditionedProposal proposal(Matrix1::Identity(), 4);

  auto result = tracking.tube(planner, cost(), initial(0.), initial(0.),
                              dynamics, feedback, proposal, -1., 1);
  for (Key key : inputs())
    EXPECT(assert_equal(Vector1{4.}, result.nominal.at<Vector1>(key)));
  CHECK_EXCEPTION(tracking.tube(planner, cost(), initial(0.), initial(2.),
                                dynamics, feedback, proposal, 0., 1),
                  std::invalid_argument);
}

struct StateFeedbackProposal : GaussianProposal {
  using GaussianProposal::GaussianProposal;
  struct Batch : ControlBatch {
    Vector1 operator()(Eigen::Index, size_t, const Vector1& state) const {
      return -0.5 * state;
    }
  };
  Batch sample(const State& state, const Values&, double, bool) const {
    return {{Matrix::Zero(1, state.mean.cols()), {}}};
  }
};

// Tube realizes a policy from each branch's state. Robust reuses controls
// realized on the nominal path for the actual trajectory's feedforward term.
TEST(MPPITracking, StateFeedbackProposal) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking(4);
  StateFeedbackProposal proposal(Matrix1::Identity(), 1);
  const auto zeroFeedback = [](size_t, const Vector1&, const Vector1&,
                               const Vector1&) { return Vector1::Zero(); };

  auto tube = tracking.tube(planner, cost(), initial(2.), initial(4.), dynamics,
                            zeroFeedback, proposal, 20., 1);
  EXPECT_LONGS_EQUAL(0, tube.candidate);
  EXPECT(assert_equal(Vector1{-2.}, tube.nominal.at<Vector1>(inputs()[0])));
  EXPECT(assert_equal(Vector1{-1.}, tube.nominal.at<Vector1>(inputs()[1])));

  auto robust =
      tracking.robust(planner, cost(), initial(2.), initial(4.), dynamics,
                      zeroFeedback, proposal, 1., 3, 0, 1, Matrix1::Zero());
  EXPECT_LONGS_EQUAL(0, robust.candidate);
  EXPECT(assert_equal(Vector1{-1.}, robust.actual.at<Vector1>(inputs()[0])));
  EXPECT(assert_equal(Vector1{-0.5}, robust.actual.at<Vector1>(inputs()[1])));
  EXPECT(
      assert_equal(Vector1{2.5}, robust.actual.at<Vector1>(states().back())));
}

// Derivative-coordinate batches retain their native state across Tube
// iterations.
TEST(MPPITracking, TubeSmoothProposal) {
  const auto planner = makePlanner(0.2);
  const auto tracking = makeTracking();
  SmoothControlProposal smooth(Matrix1::Identity(), 8, 37);
  SmoothControlProposal independent = smooth;

  auto result = tracking.tube(planner, cost(), initial(0.), initial(0.),
                              dynamics, feedback, smooth, -1., 3);
  auto expected =
      planner.optimize(cost(), initial(0.), dynamics, independent, MPPI(), 3);
  EXPECT(assert_equal(expected, result.nominal, 1e-10));
}

TEST(MPPITracking, Robust) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking();
  Values previous = initial(0.);
  previous.update(1, Vector1{1.});

  // Candidate starts are 0, 0.5, 1, 2, 3; the last free energy < 1 is at 1.
  auto result =
      tracking.robust(planner, cost(), previous, initial(3.), dynamics,
                      feedback, 1., 5, 1, 2, Matrix1::Identity());
  EXPECT_LONGS_EQUAL(2, result.candidate);
  EXPECT(assert_equal(Vector1{1.}, result.nominal.at<Vector1>(0)));
  EXPECT(assert_equal(Vector1{3.}, result.actual.at<Vector1>(0)));
  CHECK_EXCEPTION(
      tracking.robust(planner, cost(), previous, initial(3.), dynamics,
                      feedback, 1., 4, 1, 2, Matrix1::Identity()),
      std::invalid_argument);
}

// An unsafe best primitive activates the caller-specified stopping trajectory.
TEST(MPPITracking, Primitives) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking();

  // High cost exceeds stopThreshold -> returns the stopping controls.
  auto result =
      tracking.primitives(planner, cost(), initial(4.), initial(4.), dynamics,
                          1, 2., 0., 2, 1, 1., Matrix::Zero(1, 2));
  EXPECT_LONGS_EQUAL(1, result.candidate);
  EXPECT(assert_equal(Vector1{0.}, result.nominal.at<Vector1>(3)));
  EXPECT(assert_equal(Vector1{0.}, result.nominal.at<Vector1>(4)));

  // Equal baselines with positive hysteresis favor colored MPPI (candidate 0).
  result = tracking.primitives(planner, cost(), initial(0.), initial(0.),
                               dynamics, 1, 2., 0.1, 1, 1, 0., {});
  EXPECT_LONGS_EQUAL(0, result.candidate);
  EXPECT(assert_equal(result.nominal, result.actual));
}

// A stopping command takes priority over the filter used for normal planning.
TEST(MPPITracking, StopWithSmoothing) {
  const auto planner = makePlanner();
  auto tracking = makeTracking(4);
  tracking.setControlHistory(Matrix{{10., 10.}});

  const auto result =
      tracking.primitives(planner, cost(), initial(4.), initial(4.), dynamics,
                          1, 0., 0., 1, 1, 1., Matrix::Zero(1, 2));
  EXPECT_LONGS_EQUAL(1, result.candidate);
  for (Key key : inputs()) {
    EXPECT(assert_equal(Vector1{0.}, result.nominal.at<Vector1>(key)));
    EXPECT(assert_equal(Vector1{0.}, result.actual.at<Vector1>(key)));
  }
}

// If every sample is infeasible, return the supplied stop in both branches.
TEST(MPPITracking, StopWithInfiniteCosts) {
  const auto planner = makePlanner();
  const auto tracking = makeTracking(4);
  auto infeasible = cost();
  infeasible.addPrior(states().front(), Vector1{1e200},
                      noiseModel::Unit::Create(1));
  const auto start = initial(0.);
  EXPECT(std::isinf(infeasible.error(start)));

  try {
    const auto result =
        tracking.primitives(planner, infeasible, start, start, dynamics, 1, 0.,
                            0., 1, 1, 1., Matrix{{-1., 0.}});
    EXPECT(assert_equal(Vector1{-1.}, result.nominal.at<Vector1>(inputs()[0])));
    EXPECT(assert_equal(Vector1{0.}, result.nominal.at<Vector1>(inputs()[1])));
    EXPECT(assert_equal(result.nominal, result.actual));
  } catch (const std::domain_error&) {
    FAIL(
        "The competing search prevented the stopping plan from being "
        "returned.");
  }
}

}  // namespace controllers
/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

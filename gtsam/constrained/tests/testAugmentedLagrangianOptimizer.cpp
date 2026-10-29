/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  testAugmentedLagrangianOptimizer.cpp
 * @brief Test the augmented Lagrangian optimizer for equality and inequality
 * constraints, including Aggressive and BCL update policies and PHR terms.
 * @author Yetong Zhang
 * @author Frank Dellaert (codex assisted)
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/expressions.h>

#include <cmath>

#include "constrainedExample.h"

using namespace gtsam;

/* ************************************************************************* */
namespace scalar_problem {

const Symbol kX('x', 0);
const Double_ kExpression(kX);

Values ValuesAt(double x) {
  Values values;
  values.insert(kX, x);
  return values;
}

NonlinearFactorGraph CostTo(double target, double sigma = 1.0) {
  NonlinearFactorGraph costs;
  costs.addPrior(kX, target, noiseModel::Isotropic::Sigma(1, sigma));
  return costs;
}

NonlinearEqualityConstraints EqualityTo(double target, double sigma = 1.0) {
  NonlinearEqualityConstraints constraints;
  constraints.emplace_shared<ExpressionEqualityConstraint<double>>(
      kExpression, target, Vector1(sigma));
  return constraints;
}

NonlinearInequalityConstraints LessEqual(double upperBound,
                                         double sigma = 1.0) {
  NonlinearInequalityConstraints constraints;
  constraints.emplace_shared<ScalarExpressionInequalityConstraint>(
      kExpression - Double_(upperBound), sigma);
  return constraints;
}

ConstrainedOptProblem Problem(
    const NonlinearFactorGraph& costs,
    const NonlinearEqualityConstraints& equalities,
    const NonlinearInequalityConstraints& inequalities) {
  return ConstrainedOptProblem(costs, equalities, inequalities);
}

AugmentedLagrangianState StateAt(const ConstrainedOptProblem& problem, double x,
                                 double penalty) {
  AugmentedLagrangianState state(0, ValuesAt(x), problem);
  state.initializeLagrangeMultipliers(problem);
  state.muEq = penalty;
  state.muIneq = penalty;
  return state;
}

AugmentedLagrangianParams::shared_ptr BclParams() {
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->updatePolicy = AugmentedLagrangianUpdatePolicy::BCL;
  params->maxIterations = 30;
  params->absoluteViolationTolerance = 1e-7;
  params->absoluteStationarityTolerance = 1e-7;
  params->lmParams.maxIterations = 100;
  params->storeOptProgress = true;
  return params;
}

double Gradient(const NonlinearFactorGraph& graph, const Values& values) {
  return graph.linearize(values)->gradientAtZero().at(kX)(0);
}

}  // namespace scalar_problem
/* ************************************************************************* */

/* ************************************************************************* */
namespace phr_factor_tests {

// Verifies exact PHR values and gradients on the active shifted-ramp branch.
TEST(AugmentedLagrangianPHR, ActiveValueAndGradient) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  AugmentedLagrangianState state = StateAt(problem, 1.0, 4.0);
  state.lambdaIneq.at(0) = 2.0;

  const AugmentedLagrangianOptimizer optimizer(problem, state.values);
  const NonlinearFactorGraph graph =
      optimizer.augmentedLagrangianFunction(state);

  // The graph stores max(0, lambda+rho*g)^2/(2*rho); subtracting the fixed
  // lambda^2/(2*rho) gives the mathematical PHR term.
  EXPECT_DOUBLES_EQUAL(4.0, graph.error(state.values) - 0.5, 1e-12);
  EXPECT_DOUBLES_EQUAL(6.0, Gradient(graph, state.values), 1e-12);
}

// Verifies the inactive PHR branch is constant with zero derivative.
TEST(AugmentedLagrangianPHR, InactiveValueAndGradient) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  AugmentedLagrangianState state = StateAt(problem, -1.0, 4.0);
  state.lambdaIneq.at(0) = 2.0;

  const AugmentedLagrangianOptimizer optimizer(problem, state.values);
  const NonlinearFactorGraph graph =
      optimizer.augmentedLagrangianFunction(state);

  EXPECT_DOUBLES_EQUAL(0.0, graph.error(state.values), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, Gradient(graph, state.values), 1e-12);
}

// Verifies the chosen derivative at the PHR transition is zero.
TEST(AugmentedLagrangianPHR, TransitionValueAndGradient) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  AugmentedLagrangianState state = StateAt(problem, -0.5, 4.0);
  state.lambdaIneq.at(0) = 2.0;

  const AugmentedLagrangianOptimizer optimizer(problem, state.values);
  const NonlinearFactorGraph graph =
      optimizer.augmentedLagrangianFunction(state);

  EXPECT_DOUBLES_EQUAL(0.0, graph.error(state.values), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, Gradient(graph, state.values), 1e-12);
}

// Verifies ALM rejects smooth penalties that break the PHR multiplier identity.
TEST(AugmentedLagrangianPHR, RejectsCustomSmoothedPenalty) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->ineqConstraintPenaltyFunction =
      std::make_shared<SmoothRampPoly2>(1.0);

  CHECK_EXCEPTION(
      AugmentedLagrangianOptimizer(problem, ValuesAt(0.0), params).optimize(),
      std::invalid_argument);
}

}  // namespace phr_factor_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace diagnostic_tests {

// Verifies q drives the projected inequality update on both active branches.
TEST(AugmentedLagrangianDiagnostics, ProjectedMultiplierAndResidual) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  auto params = BclParams();
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(0.25), params);

  AugmentedLagrangianState active = StateAt(problem, 0.25, 10.0);
  active.bclAlpha = 0.1;
  active.bclOmega = 100.0;
  active.bclEta = 100.0;
  AugmentedLagrangianState activeResult;
  double nextEq, nextIneq;
  std::tie(activeResult, nextEq, nextIneq) =
      optimizer.iterate(active, 10.0, 10.0);
  EXPECT_DOUBLES_EQUAL(0.25, activeResult.generalizedConstraintViolation,
                       1e-12);
  EXPECT_DOUBLES_EQUAL(2.5, activeResult.lambdaIneq.at(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(10.0, nextEq, 1e-12);

  AugmentedLagrangianState inactive = StateAt(problem, -1.0, 10.0);
  inactive.lambdaIneq.at(0) = 2.0;
  inactive.bclAlpha = 0.1;
  inactive.bclOmega = 100.0;
  inactive.bclEta = 100.0;
  AugmentedLagrangianState inactiveResult;
  std::tie(inactiveResult, nextEq, nextIneq) =
      optimizer.iterate(inactive, 10.0, 10.0);
  EXPECT_DOUBLES_EQUAL(0.2, inactiveResult.generalizedConstraintViolation,
                       1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, inactiveResult.primalInequalityViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, inactiveResult.lambdaIneq.at(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.0, inactiveResult.complementarity, 1e-12);
}

// Verifies mixed diagnostics combine equality feasibility, q, and
// complementarity.
TEST(AugmentedLagrangianDiagnostics, MixedConstraints) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem =
      Problem({}, EqualityTo(1.0), LessEqual(0.25));
  auto params = BclParams();
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(0.5), params);

  AugmentedLagrangianState state = StateAt(problem, 0.5, 2.0);
  state.lambdaIneq.at(0) = 1.0;
  state.bclAlpha = 0.1;
  state.bclOmega = 100.0;
  state.bclEta = 100.0;
  const auto result = std::get<0>(optimizer.iterate(state, 2.0, 2.0));

  EXPECT_DOUBLES_EQUAL(0.5, result.generalizedConstraintViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.25, result.primalInequalityViolation, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.375, result.complementarity, 1e-12);
  EXPECT(result.augmentedLagrangianStationarity > 0.0);
}

}  // namespace diagnostic_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace bcl_policy_tests {

// Verifies the translated paper defaults initialize alpha, omega, and eta.
TEST(AugmentedLagrangianBCL, DefaultInitialization) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem(CostTo(0.0), {}, {});
  auto params = BclParams();
  params->maxIterations = 1;
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(0.0), params);
  optimizer.optimize();

  const auto& initial = optimizer.progress().front();
  EXPECT_DOUBLES_EQUAL(10.0, initial.muEq, 1e-12);
  EXPECT_DOUBLES_EQUAL(10.0, initial.muIneq, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.1, initial.bclAlpha, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.1, initial.bclOmega, 1e-12);
  EXPECT_DOUBLES_EQUAL(std::pow(0.1, 0.1), initial.bclEta, 1e-12);
}

// Verifies an accepted BCL iteration updates multipliers and tightens
// tolerances.
TEST(AugmentedLagrangianBCL, AcceptedUpdateSchedule) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  auto params = BclParams();
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(0.01), params);
  AugmentedLagrangianState state = StateAt(problem, 0.01, 10.0);
  state.bclAlpha = 0.1;
  state.bclOmega = 1.0;
  state.bclEta = 1.0;

  double nextEq, nextIneq;
  AugmentedLagrangianState result;
  std::tie(result, nextEq, nextIneq) = optimizer.iterate(state, 10.0, 10.0);

  EXPECT(result.innerConverged);
  EXPECT(result.updateType == AugmentedLagrangianUpdateType::Multiplier);
  EXPECT_DOUBLES_EQUAL(0.1, result.lambdaIneq.at(0), 1e-12);
  EXPECT_DOUBLES_EQUAL(0.1, result.bclOmega, 1e-12);
  EXPECT_DOUBLES_EQUAL(std::pow(0.1, 0.9), result.bclEta, 1e-12);
  EXPECT_DOUBLES_EQUAL(10.0, nextEq, 1e-12);
  EXPECT_DOUBLES_EQUAL(10.0, nextIneq, 1e-12);
}

// Verifies a rejected BCL iteration grows rho and resets both tolerances.
TEST(AugmentedLagrangianBCL, PenaltyUpdateSchedule) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem({}, {}, LessEqual(0.0));
  auto params = BclParams();
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(1.0), params);
  AugmentedLagrangianState state = StateAt(problem, 1.0, 10.0);
  state.bclAlpha = 0.1;
  state.bclOmega = 100.0;
  state.bclEta = 0.1;

  double nextEq, nextIneq;
  AugmentedLagrangianState result;
  std::tie(result, nextEq, nextIneq) = optimizer.iterate(state, 10.0, 10.0);

  EXPECT(result.innerConverged);
  EXPECT(result.updateType == AugmentedLagrangianUpdateType::Penalty);
  EXPECT_DOUBLES_EQUAL(1000.0, nextEq, 1e-12);
  EXPECT_DOUBLES_EQUAL(1000.0, nextIneq, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.001, result.bclAlpha, 1e-12);
  EXPECT_DOUBLES_EQUAL(0.001, result.bclOmega, 1e-12);
  EXPECT_DOUBLES_EQUAL(std::pow(0.001, 0.1), result.bclEta, 1e-12);
}

// Verifies an unmet inner stationarity target performs no outer update.
TEST(AugmentedLagrangianBCL, InnerFailurePerformsNoUpdate) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem = Problem(CostTo(0.0), {}, {});
  auto params = BclParams();
  params->lmParams.maxIterations = 0;
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(1.0), params);
  AugmentedLagrangianState state = StateAt(problem, 1.0, 10.0);
  state.bclAlpha = 0.1;
  state.bclOmega = 1e-12;
  state.bclEta = 100.0;

  double nextEq, nextIneq;
  AugmentedLagrangianState result;
  std::tie(result, nextEq, nextIneq) = optimizer.iterate(state, 10.0, 10.0);

  EXPECT(!result.innerConverged);
  EXPECT(result.updateType == AugmentedLagrangianUpdateType::None);
  EXPECT_DOUBLES_EQUAL(10.0, nextEq, 1e-12);
  EXPECT_DOUBLES_EQUAL(10.0, nextIneq, 1e-12);
  EXPECT_LONGS_EQUAL(0, static_cast<long>(result.unconstrainedIterations));
}

}  // namespace bcl_policy_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace sequencing_tests {

// Verifies Aggressive updates its multiplier from x_{k+1}, not x_k.
TEST(AugmentedLagrangianAggressive, UsesPostLmPoint) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem =
      Problem(CostTo(2.0), EqualityTo(0.0), {});
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->updatePolicy = AugmentedLagrangianUpdatePolicy::Aggressive;
  params->lmParams.maxIterations = 100;
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(-4.0), params);
  AugmentedLagrangianState state = StateAt(problem, -4.0, 1.0);

  const AugmentedLagrangianState result =
      std::get<0>(optimizer.iterate(state, 1.0, 1.0));
  const double solvedX = result.values.at<double>(kX);

  EXPECT_DOUBLES_EQUAL(solvedX, result.lambdaEq.at(0)(0), 1e-9);
  EXPECT(std::abs(result.lambdaEq.at(0)(0) + 4.0) > 1.0);
}

// Verifies vector equality biases use each constraint sigma independently.
TEST(AugmentedLagrangian, VectorBiasUsesElementwiseSigmas) {
  const Symbol xKey('v', 0);
  const Vector3_ x(xKey);
  const Vector sigmas{{1.0, 2.0, 4.0}};
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<ExpressionEqualityConstraint<Vector3>>(
      x, Vector3::Zero(), sigmas);
  const ConstrainedOptProblem problem =
      ConstrainedOptProblem::EqConstrainedOptProblem({}, equalities);
  Values values;
  values.insert<Vector3>(xKey, Vector3::Zero());

  AugmentedLagrangianState state(0, values, problem);
  state.initializeLagrangeMultipliers(problem);
  state.muEq = 0.2;
  state.muIneq = 1.0;
  state.lambdaEq.at(0) = Vector{{0.3, -0.5, 1.2}};
  const AugmentedLagrangianOptimizer optimizer(problem, values);
  const NonlinearFactorGraph graph =
      optimizer.augmentedLagrangianFunction(state);
  const auto factor = std::dynamic_pointer_cast<NoiseModelFactor>(graph.at(0));

  EXPECT(factor);
  const Vector expected =
      (state.lambdaEq.at(0) / state.muEq).cwiseProduct(sigmas);
  EXPECT(assert_equal(expected, factor->unwhitenedError(values), 1e-12));
}

}  // namespace sequencing_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace optimization_tests {

// Verifies the default BCL policy solves the equality-only reference problem.
TEST(AugmentedLagrangianBCL, DefaultAndEqualityOptimization) {
  using namespace constrained_example1;
  auto params = std::make_shared<AugmentedLagrangianParams>();
  EXPECT(params->updatePolicy == AugmentedLagrangianUpdatePolicy::BCL);
  const Values result =
      AugmentedLagrangianOptimizer(problem, init_values, params).optimize();
  EXPECT(assert_equal(optimal_values, result, 2e-4));
}

// Verifies the opt-in Aggressive policy still solves the reference problem.
TEST(AugmentedLagrangianAggressive, EqualityOnlyOptimization) {
  using namespace constrained_example1;
  auto params = std::make_shared<AugmentedLagrangianParams>();
  params->updatePolicy = AugmentedLagrangianUpdatePolicy::Aggressive;
  const Values result =
      AugmentedLagrangianOptimizer(problem, init_values, params).optimize();
  EXPECT(assert_equal(optimal_values, result, 1e-4));
}

// Verifies BCL activates an inequality whose unconstrained minimizer is
// infeasible.
TEST(AugmentedLagrangianBCL, ActiveInequalityOptimization) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem =
      Problem(CostTo(2.0), {}, LessEqual(0.0));
  auto params = BclParams();
  const Values result =
      AugmentedLagrangianOptimizer(problem, ValuesAt(2.0), params).optimize();
  EXPECT_DOUBLES_EQUAL(0.0, result.at<double>(kX), 2e-4);
}

// Verifies BCL leaves an inactive inequality multiplier at zero.
TEST(AugmentedLagrangianBCL, InactiveInequalityOptimization) {
  using namespace scalar_problem;
  const ConstrainedOptProblem problem =
      Problem(CostTo(-1.0), {}, LessEqual(0.0));
  auto params = BclParams();
  const AugmentedLagrangianOptimizer optimizer(problem, ValuesAt(1.0), params);
  const Values result = optimizer.optimize();
  EXPECT_DOUBLES_EQUAL(-1.0, result.at<double>(kX), 2e-4);
  EXPECT_DOUBLES_EQUAL(0.0, optimizer.progress().back().lambdaIneq.at(0), 1e-9);
}

// Verifies BCL handles a mixed nonlinear problem whose active set changes.
TEST(AugmentedLagrangianBCL, MixedChangingActiveSetOptimization) {
  using namespace constrained_example2;
  auto params = scalar_problem::BclParams();
  params->maxIterations = 40;
  const Values result =
      AugmentedLagrangianOptimizer(problem, init_values, params).optimize();
  EXPECT(assert_equal(optimal_values, result, 5e-4));
}

}  // namespace optimization_tests
/* ************************************************************************* */

/* ************************************************************************* */
namespace persistent_solver_tests {

// Direct gradient accumulation must apply both noise-whitening factors when
// a cost supplies an unwhitened Jacobian rather than the usual unit model.
TEST(AugmentedLagrangianBCL, StationarityWithUnwhitenedJacobian) {
  using namespace scalar_problem;
  NonlinearFactorGraph costs;
  costs.emplace_shared<LinearContainerFactor>(
      JacobianFactor(kX, Matrix11::Constant(2.0), Vector1(4.0),
                     noiseModel::Isotropic::Sigma(1, 2.0)),
      ValuesAt(0.0));
  const auto problem = Problem(costs, {}, {});
  auto params = BclParams();
  params->lmParams.maxIterations = 0;
  const auto state = StateAt(problem, 0.0, 10.0);
  const AugmentedLagrangianOptimizer optimizer(problem, state.values, params);
  const auto result = std::get<0>(optimizer.iterate(state, 10.0, 10.0));
  EXPECT_DOUBLES_EQUAL(2.0, result.augmentedLagrangianStationarity, 1e-12);
  EXPECT(assert_equal(state.values, result.values));
}

class CountingCost : public NoiseModelFactor1<double> {
 public:
  mutable size_t linearizations = 0;

  CountingCost()
      : NoiseModelFactor1<double>(noiseModel::Unit::Create(1),
                                  scalar_problem::kX) {}

  Vector evaluateError(const double& x, OptionalMatrixType H) const override {
    if (H) {
      ++linearizations;
      *H = Matrix11::Constant(2.0 * x);
    }
    return Vector1(x * x - 1.0);
  }
};

// Nonlinear steps, damping retries, and BCL stationarity share one Jacobian
// evaluation per visited point, including the terminal point.
TEST(AugmentedLagrangianBCL, SharesStationarityLinearization) {
  using namespace scalar_problem;
  auto cost = std::make_shared<CountingCost>();
  NonlinearFactorGraph costs;
  costs.push_back(cost);
  const auto problem = Problem(costs, {}, {});
  auto params = BclParams();
  auto state = StateAt(problem, 0.1, 10.0);
  state.bclOmega = 1e-8;
  state.bclEta = 1.0;
  const AugmentedLagrangianOptimizer optimizer(problem, state.values, params);
  const auto result = std::get<0>(optimizer.iterate(state, 10.0, 10.0));
  EXPECT(result.innerConverged);
  EXPECT_DOUBLES_EQUAL(1.0, result.values.at<double>(kX), 1e-7);
  EXPECT_LONGS_EQUAL(result.unconstrainedIterations + 1, cost->linearizations);
  const double x = result.values.at<double>(kX);
  EXPECT_DOUBLES_EQUAL(std::abs(2.0 * x * (x * x - 1.0)),
                       result.augmentedLagrangianStationarity, 1e-12);
}

// Terminal residuals recover vector equality updates with unequal sigmas.
// The terminal AL gradient is the ordinary Lagrangian gradient at lambda+.
TEST(AugmentedLagrangianBCL, VectorMultiplierFromTerminalModel) {
  const Symbol key('v', 0);
  const Vector3 sigmas(1.0, 2.0, 4.0);
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<ExpressionEqualityConstraint<Vector3>>(
      Vector3_(key), Vector3::Zero(), sigmas);
  NonlinearFactorGraph costs;
  const Vector3 target(1.0, -2.0, 3.0);
  costs.addPrior(key, target, noiseModel::Unit::Create(3));
  const ConstrainedOptProblem problem(costs, equalities, {});
  Values initial;
  initial.insert(key, Vector3(2.0, 1.0, -1.0));
  auto params = scalar_problem::BclParams();
  AugmentedLagrangianState state(0, initial, problem);
  state.initializeLagrangeMultipliers(problem);
  state.lambdaEq[0] = Vector3(0.3, -0.5, 1.2);
  state.bclOmega = 1e-6;
  state.bclEta = 100.0;
  const AugmentedLagrangianOptimizer optimizer(problem, initial, params);
  const auto result = std::get<0>(optimizer.iterate(state, 10.0, 10.0));
  EXPECT(result.innerConverged);
  const Vector3 x = result.values.at<Vector3>(key);
  EXPECT(
      assert_equal(Vector(state.lambdaEq[0] + 10.0 * x.cwiseQuotient(sigmas)),
                   result.lambdaEq[0], 1e-10));
  const Vector3 lagrangianGradient =
      x - target + result.lambdaEq[0].cwiseQuotient(sigmas);
  EXPECT_DOUBLES_EQUAL(lagrangianGradient.lpNorm<Eigen::Infinity>(),
                       result.augmentedLagrangianStationarity, 1e-10);
}

// Reusing numerical storage across changed penalties and multipliers must
// match independently constructed solves, including changing PHR activity.
TEST(AugmentedLagrangianBCL, PersistentMatchesFreshSubproblems) {
  using namespace constrained_example2;
  for (bool diagonal : {false, true}) {
    auto params = scalar_problem::BclParams();
    params->maxIterations = 40;
    params->lmParams.dampingParams.diagonalDamping = diagonal;
    params->lmParams.dampingParams.exactHessianDiagonal = diagonal;
    params->lmParams.useFixedLambdaFactor = false;
    params->lmParams.ordering = Ordering{x2_key, x1_key};
    const AugmentedLagrangianOptimizer optimizer(problem, init_values, params);
    const Values result = optimizer.optimize();
    const auto progress = optimizer.progress();
    EXPECT(assert_equal(optimal_values, result, 5e-4));
    EXPECT(progress.size() > 2);
    for (size_t i = 1; i < progress.size(); ++i) {
      const auto& expected = progress[i];
      const auto fresh = std::get<0>(
          optimizer.iterate(progress[i - 1], expected.muEq, expected.muIneq));
      EXPECT(assert_equal(expected.values, fresh.values, 1e-9));
      EXPECT_DOUBLES_EQUAL(expected.augmentedLagrangianStationarity,
                           fresh.augmentedLagrangianStationarity, 1e-9);
      EXPECT(expected.updateType == fresh.updateType);
    }
    // A second optimize() starts from the original primal/dual state.
    EXPECT(assert_equal(result, optimizer.optimize(), 1e-9));
  }
}

// A failed damping search leaves the current point/model intact and cannot
// authorize a BCL multiplier update merely because the cost stopped changing.
TEST(AugmentedLagrangianBCL, RejectedStepKeepsTerminalModel) {
  using namespace scalar_problem;
  const auto problem = Problem(CostTo(2.0), {}, LessEqual(0.0));
  auto params = BclParams();
  params->lmParams.minModelFidelity = 2.0;
  params->lmParams.lambdaUpperBound = 1.0;
  auto state = StateAt(problem, 1.0, 10.0);
  state.bclOmega = 1e-12;
  state.bclEta = 100.0;
  const AugmentedLagrangianOptimizer optimizer(problem, state.values, params);
  const auto result = std::get<0>(optimizer.iterate(state, 10.0, 10.0));
  EXPECT(!result.innerConverged);
  EXPECT(result.updateType == AugmentedLagrangianUpdateType::None);
  EXPECT_LONGS_EQUAL(0, result.unconstrainedIterations);
  EXPECT(assert_equal(state.values, result.values));
  EXPECT_DOUBLES_EQUAL(9.0, result.augmentedLagrangianStationarity, 1e-12);
}

// Once the inner gradient is accurate enough for the requested final result,
// BCL must continue feasibility updates rather than demand sub-roundoff steps.
TEST(AugmentedLagrangianBCL, PoorScalingRespectsFinalStationarityTolerance) {
  using namespace constrained_example;
  NonlinearFactorGraph costs;
  costs.addPrior(x1_key, 1.5, noiseModel::Unit::Create(1));
  costs.addPrior(x2_key, 1.2, noiseModel::Unit::Create(1));
  NonlinearEqualityConstraints equalities;
  equalities.emplace_shared<ExpressionEqualityConstraint<double>>(
      x1 + x2, 1.0, Vector1(0.01));
  NonlinearInequalityConstraints inequalities;
  inequalities.emplace_shared<ScalarExpressionInequalityConstraint>(
      x1 - Double_(0.4), 10.0);
  const ConstrainedOptProblem problem(costs, equalities, inequalities);
  Values initial;
  initial.insert(x1_key, 2.0);
  initial.insert(x2_key, -1.0);
  auto params = scalar_problem::BclParams();
  const AugmentedLagrangianOptimizer optimizer(problem, initial, params);
  const Values result = optimizer.optimize();
  EXPECT(optimizer.progress().back().converged);
  EXPECT_DOUBLES_EQUAL(0.4, result.at<double>(x1_key), 1e-5);
  EXPECT_DOUBLES_EQUAL(0.6, result.at<double>(x2_key), 1e-5);
  for (const auto& state : optimizer.progress()) {
    EXPECT(state.bclOmega >= params->absoluteStationarityTolerance);
  }
}

}  // namespace persistent_solver_tests
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

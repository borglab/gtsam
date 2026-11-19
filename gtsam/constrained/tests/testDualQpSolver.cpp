/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415. All Rights Reserved.
 * See LICENSE for the license information.
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/internal/DualQpSolver.h>
#include <limits>
#include <random>

using namespace gtsam;
using Params = ActiveSetSolverParams;
using Formulation = Params::QpFormulation;
using Backend = Params::QpSubproblemSolver;

/* ************************************************************************* */
namespace dual_qp {

Params::shared_ptr parameters(Backend backend, Formulation formulation = Formulation::Dual) {
  auto result = std::make_shared<Params>();
  result->qpSubproblemSolver = backend;
  result->qpFormulation = formulation;
  result->regularization = 0;
  return result;
}

QpProblem objective(const Vector& target) {
  QpProblem result;
  result.addCost(JacobianFactor(0, Matrix::Identity(target.size(), target.size()), target));
  return result;
}

void less(QpProblem* problem, const Matrix& normals, const Vector& bounds) {
  problem->addConstraint(LinearConstraint::LessEqual(JacobianFactor(0, normals, bounds)));
}

void equal(QpProblem* problem, const Matrix& normals, const Vector& bounds) {
  problem->addConstraint(LinearConstraint::Equal(JacobianFactor(0, normals, bounds)));
}

Values initial(const Vector& point) {
  Values values;
  values.insert(0, point);
  return values;
}

// Primal-infeasible starts bypass primal initialization and return existing signs.
TEST(DualQp, InfeasibleStartAndMultipliers) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector3{3, -2, 5});
    less(&problem, Matrix{{1, 0, 0}}, Vector1{1});
    problem.addConstraint(LinearConstraint::GreaterEqual(
        JacobianFactor(0, Matrix{{0, 1, 0}}, Vector1{0})));
    equal(&problem, Matrix{{0, 0, 1}}, Vector1{2});
    auto params = parameters(backend);
    params->qpInitialization = Params::QpInitialization::FeasibleStepThenProjectedGradient;
    const auto result = ActiveSetSolver(problem, params).optimizeWithState(initial(Vector3{9, -9, -9}));
    EXPECT(assert_equal(Vector3(1, 0, 2), result.first.at<Vector>(0), 1e-8));
    CHECK(result.second.qpFormulation == Formulation::Dual);
    CHECK(result.second.converged);
    EXPECT(assert_equal(Vector1(-2), result.second.inequalityMultipliers[0], 1e-8));
    EXPECT(assert_equal(Vector1(-2), result.second.inequalityMultipliers[1], 1e-8));
    EXPECT(assert_equal(Vector1(-3), result.second.equalityMultipliers[0], 1e-8));
  }
}

// Cold Automatic chooses Dual even at or above the old dimension threshold.
TEST(DualQp, AutomaticColdStarts) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    for (int count : {0, 1, 2, 3, 5}) {
      QpProblem problem = objective(Vector4{2, 2, 2, 2});
      equal(&problem, Matrix{{0, 0, 1, 0}, {0, 0, 2, 0}, {0, 0, 0, 1}}, Vector3{0, 0, 0});
      if (count) less(&problem, Matrix::Ones(count, 4), Vector::Constant(count, 1));
      const auto result = ActiveSetSolver(problem, parameters(backend, Formulation::Automatic)).optimizeWithState();
      CHECK(result.second.qpFormulation == (count ? Formulation::Dual : Formulation::Primal));
      CHECK(result.second.converged);
    }
  }
}

// Automatic retains converged formulations, but infeasible/stale starts use Dual.
TEST(DualQp, AutomaticWarmStartPolicy) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector2{3, 2});
    less(&problem, Matrix::Identity(2, 2), Vector2{1, 1});
    const auto automatic = parameters(backend, Formulation::Automatic);
    for (auto mode : {Formulation::Primal, Formulation::Dual}) {
      const auto previous = ActiveSetSolver(problem, parameters(backend, mode)).optimizeWithState();
      const auto resumed = ActiveSetSolver(problem, automatic).optimizeWithState(previous.first, previous.second);
      CHECK(resumed.second.qpFormulation == mode);
      EXPECT(assert_equal(previous.first, resumed.first, 1e-8));
      auto stale = previous.second;
      stale.converged = false;
      CHECK(ActiveSetSolver(problem, automatic).optimizeWithState(previous.first, stale)
                .second.qpFormulation == Formulation::Dual);
      const auto infeasible = ActiveSetSolver(problem, automatic).optimizeWithState(
          initial(Vector2{5, 5}), previous.second);
      CHECK(infeasible.second.qpFormulation == Formulation::Dual);
      EXPECT(assert_equal(previous.first, infeasible.first, 1e-8));
      auto malformed = previous.second;
      malformed.inequalityMultipliers = {Vector1{-1}};
      CHECK_EXCEPTION(ActiveSetSolver(problem, automatic).optimizeWithState(previous.first, malformed),
                      std::invalid_argument);
      malformed = previous.second;
      malformed.inequalityMultipliers[0](0) = std::numeric_limits<double>::quiet_NaN();
      CHECK_EXCEPTION(ActiveSetSolver(problem, automatic).optimizeWithState(previous.first, malformed),
                      std::invalid_argument);
      malformed = previous.second;
      malformed.qpFormulation = Formulation::Automatic;
      CHECK_EXCEPTION(ActiveSetSolver(problem, automatic).optimizeWithState(previous.first, malformed),
                      std::invalid_argument);
    }
    // Values alone provide a cold proximal center, even when already feasible.
    CHECK(ActiveSetSolver(problem, automatic).optimizeWithState(initial(Vector2{0, 0}))
              .second.qpFormulation == Formulation::Dual);
  }
}

// Singular curvature falls back only in Automatic; proximal Dual solves the QP.
TEST(DualQp, SingularCurvaturePolicy) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix{{1, 0}}, Vector1{2}));
    less(&problem, Matrix{{0, -1}}, Vector1{-1});
    auto params = parameters(backend);
    CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimize(), std::runtime_error);
    params->qpFormulation = Formulation::Automatic;
    params->regularization = 1e-3;
    const auto result = ActiveSetSolver(problem, params).optimizeWithState();
    CHECK(result.second.qpFormulation == Formulation::Dual);
    EXPECT(assert_equal(Vector2(2, 1), result.first.at<Vector>(0), 1e-6));
    // A singular QP whose active row fixes the free coordinate supports primal.
    params->regularization = 0;
    const auto fallback = ActiveSetSolver(problem, params).optimizeWithState(initial(Vector2{0, 1}));
    CHECK(fallback.second.qpFormulation == Formulation::Primal);
    EXPECT(assert_equal(Vector2(2, 1), fallback.first.at<Vector>(0), 1e-6));
    const auto warmFallback = ActiveSetSolver(problem, params).optimizeWithState(result.first, result.second);
    CHECK(warmFallback.second.qpFormulation == Formulation::Primal);
  }
}

// Fully fixed Automatic problems check inequalities without entering Dual.
TEST(DualQp, AutomaticFixedVariables) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector2{3, 2});
    equal(&problem, Matrix{{1, 0}, {2, 0}, {0, 1}}, Vector3{1, 2, 1});
    less(&problem, Matrix{{1, 1}}, Vector1{2});
    const auto params = parameters(backend, Formulation::Automatic);
    const auto solved = ActiveSetSolver(problem, params).optimizeWithState();
    CHECK(solved.second.qpFormulation == Formulation::Primal);
    EXPECT(assert_equal(Vector2(1, 1), solved.first.at<Vector>(0), 1e-8));
    // A supplied center cannot change the unique equality-feasible point.
    EXPECT(assert_equal(Vector2(1, 1), ActiveSetSolver(problem, params)
        .optimize(initial(Vector2{9, 9})).at<Vector>(0), 1e-8));
    less(&problem, Matrix{{1, 1}}, Vector1{1});
    CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimize(), std::runtime_error);
  }
}

// Curvature need only be invertible after exact equalities are removed.
TEST(DualQp, EqualityRemovesSingularDirection) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem;
    problem.addCost(JacobianFactor(0, Matrix{{1, 0}}, Vector1{2}));
    equal(&problem, Matrix{{0, 1}}, Vector1{3});
    less(&problem, Matrix{{1, 0}}, Vector1{1});
    EXPECT(assert_equal(Vector2(1, 3), ActiveSetSolver(problem, parameters(backend)).optimize().at<Vector>(0), 1e-8));
  }
}

// Dense formulations agree with and without permanent coordinate reduction,
// including redundant equalities and curvature singular only outside the face.
TEST(DualQp, OptionalDenseEqualityProjection) {
  for (bool project : {false, true}) {
    for (auto mode : {Formulation::Primal, Formulation::Dual, Formulation::Automatic}) {
      auto params = parameters(Backend::Dense, mode);
      params->projectDenseQpEqualities = project;
      QpProblem problem;
      problem.addCost(JacobianFactor(0, Matrix{{1, 0, 0}, {0, 1, 0}}, Vector2{3, -2}));
      equal(&problem, Matrix{{0, 0, 1}, {0, 0, 2}}, Vector2{4, 8});
      less(&problem, Matrix{{1, 0, 0}, {2, 0, 0}, {0, -1, 0}}, Vector3{1, 2, 0});
      const auto solved = ActiveSetSolver(problem, params).optimizeWithState();
      EXPECT(assert_equal(Vector3(1, 0, 4), solved.first.at<Vector>(0), 1e-8));
      CHECK(solved.second.converged);
      const auto warm = ActiveSetSolver(problem, params).optimizeWithState(solved.first, solved.second);
      EXPECT(assert_equal(solved.first, warm.first, 1e-8));
      equal(&problem, Matrix{{0, 0, 1}}, Vector1{5});
      CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimize(), std::runtime_error);

      QpProblem coupled = objective(Vector3{3, 2, 4});
      equal(&coupled, Matrix{{1, 1, 0}, {2, 2, 0}}, Vector2{1, 2});
      less(&coupled, Matrix{{0, 1, 0}, {0, 0, 1}}, Vector2{0, 2});
      const auto result = ActiveSetSolver(coupled, params).optimizeWithState();
      EXPECT(assert_equal(Vector3(1, 0, 2), result.first.at<Vector>(0), 1e-8));
      const Vector eq = result.second.equalityMultipliers[0];
      EXPECT(assert_equal(Vector3(-2, -2, 0),
          Vector(Matrix{{1, 2}, {1, 2}, {0, 0}} * eq), 1e-8));

      QpProblem fixed = objective(Vector2{3, 2});
      equal(&fixed, Matrix{{1, 0}, {2, 0}, {0, 1}}, Vector3{1, 2, 1});
      less(&fixed, Matrix{{1, 1}}, Vector1{2});
      EXPECT(assert_equal(Vector2(1, 1), ActiveSetSolver(fixed, params).optimize().at<Vector>(0), 1e-8));
      // The equality-only route uses the same selected backend option.
      QpProblem equalityOnly = objective(Vector3{3, 3, 4});
      equal(&equalityOnly, Matrix{{1, 1, 0}}, Vector1{2});
      EXPECT(assert_equal(Vector3(1, 1, 4), ActiveSetSolver(equalityOnly, params)
          .optimize().at<Vector>(0), 1e-8));
    }
  }
}

// Disabling coordinate reduction does not create curvature or add a floor;
// Automatic still falls back at setup, and requested proximal terms suffice.
TEST(DualQp, UnprojectedSingularCurvaturePolicy) {
  QpProblem problem;
  problem.addCost(JacobianFactor(0, Matrix{{1, 0, 0}}, Vector1{2}));
  equal(&problem, Matrix{{0, 0, 1}}, Vector1{3});
  less(&problem, Matrix{{0, -1, 0}}, Vector1{-1});
  auto params = parameters(Backend::Dense);
  params->projectDenseQpEqualities = false;
  CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimize(), std::runtime_error);
  params->qpFormulation = Formulation::Automatic;
  const auto fallback = ActiveSetSolver(problem, params).optimizeWithState(initial(Vector3{0, 1, 3}));
  CHECK(fallback.second.qpFormulation == Formulation::Primal);
  EXPECT(assert_equal(Vector3(2, 1, 3), fallback.first.at<Vector>(0), 1e-8));
  params->regularization = 0.1;
  const auto proximal = ActiveSetSolver(problem, params).optimizeWithState();
  CHECK(proximal.second.qpFormulation == Formulation::Dual);
  EXPECT(assert_equal(Vector3(2, 1, 3), proximal.first.at<Vector>(0), 1e-7));
}

// Opposing/dependent working rows require singular steps and row removal.
TEST(DualQp, DependentWarmStartAndInfeasibility) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector2{3, 2});
    less(&problem, Matrix{{1, 0}, {2, 0}, {-1, 0}, {0, 1}}, Vector4{1, 2, 0, 1});
    ActiveSetSolverState warm;
    warm.activeInequalityRows = {true, true, true, true};
    warm.inequalityMultipliers = {Vector4{-1, -1, -1, -1}};
    const auto result = ActiveSetSolver(problem, parameters(backend)).optimizeWithState(initial(Vector2{0, 0}), warm);
    EXPECT(assert_equal(Vector2(1, 1), result.first.at<Vector>(0), 1e-8));
    QpProblem impossible = objective(Vector2{0, 0});
    less(&impossible, Matrix{{1, 0}, {-1, 0}}, Vector2{0, -1});
    CHECK_EXCEPTION(ActiveSetSolver(impossible, parameters(backend)).optimize(), std::runtime_error);
  }
}

// Equality-only and fully fixed problems retain original feasibility checks.
TEST(DualQp, EmptyAndFixedTangentSpaces) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector2{3, 2});
    EXPECT(assert_equal(Vector2(3, 2), ActiveSetSolver(problem, parameters(backend)).optimize().at<Vector>(0), 1e-8));
    equal(&problem, Matrix::Identity(2, 2), Vector2{1, 1});
    less(&problem, Matrix{{1, 1}}, Vector1{2});
    EXPECT(assert_equal(Vector2(1, 1), ActiveSetSolver(problem, parameters(backend)).optimize().at<Vector>(0), 1e-8));
    less(&problem, Matrix{{1, 1}}, Vector1{1});
    CHECK_EXCEPTION(ActiveSetSolver(problem, parameters(backend)).optimize(), std::runtime_error);
    QpProblem inconsistent = objective(Vector2{0, 0});
    equal(&inconsistent, Matrix{{1, 0}, {1, 0}}, Vector2{0, 1});
    CHECK_EXCEPTION(ActiveSetSolver(inconsistent, parameters(backend)).optimize(), std::runtime_error);
  }
}

// Cross-formulation warm starts retain row identities and reject malformed data.
TEST(DualQp, WarmStartValidation) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector2{3, 2});
    less(&problem, Matrix::Identity(2, 2), Vector2{1, 1});
    auto primal = ActiveSetSolver(problem, parameters(backend, Formulation::Primal)).optimizeWithState();
    auto params = parameters(backend);
    auto dual = ActiveSetSolver(problem, params).optimizeWithState(primal.first, primal.second);
    CHECK(dual.second.iterations == 1);
    EXPECT(assert_equal(primal.first, dual.first, 1e-8));
    auto warm = dual.second;
    warm.inequalityMultipliers = {Vector1{-1}};
    CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimizeWithState(primal.first, warm), std::invalid_argument);
    warm.inequalityMultipliers.clear();
    EXPECT(assert_equal(primal.first, ActiveSetSolver(problem, params).optimizeWithState(primal.first, warm).first, 1e-8));
    params->maxIterations = 0;
    CHECK_EXCEPTION(ActiveSetSolver(problem, params).optimize(), std::runtime_error);
  }
}

// Formulation selection does not change the feasible-descent API or value shapes.
TEST(DualQp, FeasibleStepAndMatrixValues) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    QpProblem problem = objective(Vector4{1, 2, 3, 4});
    less(&problem, Matrix{{1, 0, 0, 0}}, Vector1{0.5});
    Values start;
    start.insert(0, Matrix::Zero(2, 2).eval());
    auto params = parameters(backend);
    params->ordering = Ordering{0};
    auto result = ActiveSetSolver(problem, params).optimize(start);
    EXPECT(assert_equal(Matrix({{0.5, 3}, {2, 4}}), result.at<Matrix>(0), 1e-8));
    auto step = ActiveSetSolver(problem, params).computeFeasibleStep(start);
    CHECK(step.status == ActiveSetFeasibleStepResult::Status::FeasibleProgress);
    CHECK(step.constraintViolation <= params->feasibilityTolerance);
    CHECK(step.objectiveDecrease > 0);
  }
}

// Planted KKT solutions check coupled curvature, active releases and all modes.
TEST(DualQp, SeededKktSolutions) {
  std::mt19937 generator(42);
  std::normal_distribution<double> random;
  for (int sample = 0; sample < 12; ++sample) {
    const int n = 5, m = 2 + sample;
    Matrix jacobian(n, n), normals(m, n);
    for (int i = 0; i < jacobian.size(); ++i) jacobian.data()[i] = random(generator);
    jacobian += 3 * Matrix::Identity(n, n);
    for (int i = 0; i < normals.size(); ++i) normals.data()[i] = random(generator);
    const Vector expected = Vector::Ones(n);
    Vector lambda = Vector::Zero(m), bounds = normals * expected;
    for (int i = 0; i < m; ++i) {
      if (i < 2) lambda(i) = 1 + i;
      else bounds(i) += 0.5 + std::abs(random(generator));
    }
    const Matrix hessian = jacobian.transpose() * jacobian;
    QpProblem problem;
    problem.addCost(HessianFactor(0, hessian, Vector(hessian * expected + normals.transpose() * lambda), 0));
    less(&problem, normals, bounds);
    for (auto backend : {Backend::Dense, Backend::Sparse})
      for (auto mode : {Formulation::Primal, Formulation::Dual, Formulation::Automatic}) {
        const auto result = ActiveSetSolver(problem, parameters(backend, mode)).optimizeWithState(initial(expected));
        CHECK(result.second.converged);
        EXPECT(assert_equal(expected, result.first.at<Vector>(0), 1e-6));
        const Vector actualLambda = -result.second.inequalityMultipliers[0];
        CHECK(actualLambda.minCoeff() >= -1e-7);
        EXPECT(assert_equal(Vector::Zero(n), Vector(hessian * (result.first.at<Vector>(0) - expected) +
            normals.transpose() * (actualLambda - lambda)), 1e-6));
      }
  }
}

// RHS changes and working-row changes retain the fixed sparse factorization.
TEST(DualQp, SparseFactorReuse) {
  internal::SparseQpData data;
  data.keyDimensions = {{0, 2}};
  data.regularization = 0.1;
  data.objective.emplace_shared<JacobianFactor>(0, Matrix::Identity(2, 2), Vector2{3, 2});
  data.inequalities.emplace_shared<JacobianFactor>(0, Matrix{{1, 0}}, Vector1{1}, noiseModel::Constrained::All(1));
  internal::DualQpSolver solver(data);
  CHECK(solver.prepare());
  VectorValues center;
  center.insert(0, Vector2{0, 0});
  Vector multipliers = Vector::Zero(1);
  std::vector<bool> active(1, false);
  size_t iterations = 0;
  center = solver.solve(center, &multipliers, &active, &iterations, 100, 1e-7, 1e-7);
  const size_t responses = solver.statistics().responseSolves;
  center = solver.solve(center, &multipliers, &active, &iterations, 100, 1e-7, 1e-7);
  CHECK(solver.statistics().objectiveFactorizations == 1);
  CHECK(solver.statistics().responseSolves == responses);
  CHECK(solver.statistics().rowAdditions == 1);
}

// Coupled affine rows and weak curvature require accurate homogeneous responses
// and refinement against original boundaries, including multiple variable keys.
TEST(DualQp, WeakCurvatureAffineBoundary) {
  for (auto backend : {Backend::Dense, Backend::Sparse}) {
    const Matrix hessian{{1e-8, 0}, {0, 1}};
    const Vector2 expected{0.25, 0.75};
    QpProblem problem;
    problem.addCost(HessianFactor(0, hessian, Vector(hessian * expected + Vector2{2, 5}), 0));
    problem.addCost(JacobianFactor(1, Matrix{{1}}, Vector1{4}));
    problem.addConstraint(LinearConstraint::Equal(JacobianFactor(
        0, Matrix{{0, 1}}, 1, Matrix{{1}}, Vector1{1.75})));
    less(&problem, Matrix{{1, 1}}, Vector1{1});
    auto params = parameters(backend);
    params->feasibilityTolerance = params->multiplierTolerance = 1e-9;
    params->ordering = Ordering{1, 0};
    const auto result = ActiveSetSolver(problem, params).optimizeWithState();
    EXPECT(assert_equal(expected, result.first.at<Vector>(0), 1e-8));
    EXPECT(assert_equal(Vector1(1), result.first.at<Vector>(1), 1e-8));
  }
}

// Native SPD Hessians preserve original KKT signs in either formulation. Weak
// or indefinite curvature must retain the previous rank and convexity policy.
TEST(DualQp, NativeHessianCurvaturePolicy) {
  for (auto mode : {Formulation::Primal, Formulation::Dual, Formulation::Automatic}) {
    auto params = parameters(Backend::Dense, mode);
    for (const Matrix& hessian : {Matrix{{4, 1}, {1, 3}}, Matrix{{1e-20, 0}, {0, 1}}}) {
      QpProblem problem;
      const Vector2 expected{1, 2};
      problem.addCost(HessianFactor(0, hessian,
          Vector(hessian * expected + Vector2{2 * hessian(0, 0), 0}), 0));
      less(&problem, Matrix{{1, 0}}, Vector1{1});
      const auto result = ActiveSetSolver(problem, params).optimizeWithState();
      EXPECT(assert_equal(expected, result.first.at<Vector>(0), 1e-8));
      CHECK(result.second.converged);
      EXPECT(assert_equal(Vector1(-2 * hessian(0, 0)),
                          result.second.inequalityMultipliers[0], 1e-8));
    }
    QpProblem nonconvex;
    nonconvex.addCost(HessianFactor(0, Matrix{{-0.01, 0}, {0, 1}}, Vector2{0, 0}, 0));
    params->regularization = 1;
    CHECK_EXCEPTION(ActiveSetSolver(nonconvex, params), std::runtime_error);
  }
}

// Defaults select Sparse/Automatic QPs; LPs resolve Automatic and reject Dual.
TEST(DualQp, ParameterValidation) {
  CHECK(Params().qpFormulation == Formulation::Automatic);
  CHECK(Params().qpSubproblemSolver == Backend::Sparse);
  QpProblem qp = objective(Vector2{3, 2});
  less(&qp, Matrix::Identity(2, 2), Vector2{1, 1});
  const auto cold = ActiveSetSolver(qp).optimizeWithState(initial(Vector2{10, 10}));
  CHECK(cold.second.qpFormulation == Formulation::Dual);
  EXPECT(assert_equal(Vector2(1, 1), cold.first.at<Vector>(0), 1e-7));
  LpProblem lp;
  lp.addCost(LpCost(JacobianFactor(0, Matrix{{1}}, Vector1{0})));
  lp.addConstraint(LinearConstraint::GreaterEqual(JacobianFactor(0, Matrix{{1}}, Vector1{0})));
  CHECK(ActiveSetSolver(lp).optimizeWithState().second.qpFormulation == Formulation::Primal);
  auto automatic = std::make_shared<Params>();
  CHECK(ActiveSetSolver(lp, automatic).optimizeWithState().second.qpFormulation == Formulation::Primal);
  CHECK(automatic->qpFormulation == Formulation::Automatic);
  CHECK_EXCEPTION(ActiveSetSolver(lp, parameters(Backend::Sparse)), std::invalid_argument);
  auto params = parameters(Backend::Sparse);
  params->qpFormulation = static_cast<Formulation>(99);
  CHECK_EXCEPTION(ActiveSetSolver(objective(Vector1{0}), params), std::invalid_argument);
}

}  // namespace dual_qp
/* ************************************************************************* */

int main() { TestResult tr; return TestRegistry::runAllTests(tr); }

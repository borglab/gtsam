/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/constrained/ActiveSetSolver.h>
#include <gtsam/constrained/AugmentedLagrangianOptimizer.h>
#include <gtsam/constrained/QpProblem.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/nonlinear/NonlinearMultifrontalSolver.h>

#include <random>

using namespace gtsam;

/* ************************************************************************* */
namespace analytic_qp {

AugmentedLagrangianParams::shared_ptr Params() {
  auto p = std::make_shared<AugmentedLagrangianParams>();
  p->absoluteViolationTolerance = p->absoluteStationarityTolerance = 1e-9;
  p->maxIterations = 40;
  p->storeOptProgress = true;
  return p;
}

GaussianFactorGraph Objective(bool jacobian) {
  GaussianFactorGraph graph;
  if (jacobian)
    graph.emplace_shared<JacobianFactor>(0, Matrix2::Identity(), Vector2(2, -1),
                                         noiseModel::Unit::Create(2));
  else
    graph.emplace_shared<HessianFactor>(0, Matrix2::Identity(), Vector2(2, -1),
                                        0.);
  return graph;
}

GaussianFactorGraph Equalities() {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(0, Matrix12::Ones(), Vector1(1),
                                       noiseModel::Isotropic::Sigma(1, 2));
  return graph;
}

GaussianFactorGraph Inequalities() {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(
      0, Matrix12((Matrix12() << 1., 0.).finished()), Vector1(.5),
      noiseModel::Isotropic::Sigma(1, 3));
  return graph;
}

// Jacobian and Hessian inputs solve the same QP and preserve multiplier
// scaling.
TEST(AugmentedLagrangianSolver, NativeRepresentationsAndKkt) {
  for (bool jacobian : {false, true}) {
    const auto costs = Objective(jacobian), equalities = Equalities(),
               inequalities = Inequalities();
    auto p = Params();
    AugmentedLagrangianSolver solver(costs, equalities, inequalities, p);
    const auto result = solver.optimizeWithState();
    EXPECT(result.converged);
    EXPECT(result.status == AugmentedLagrangianStatus::Converged);
    const Vector2 x = result.values.at(0);
    EXPECT(assert_equal(Vector2(.5, .5), x, 1e-7));
    const Vector2 stationarity = x - Vector2(2, -1) +
                                 Vector2::Ones() * result.lambdaEq[0](0) / 2 +
                                 Vector2(1, 0) * result.lambdaIneq[0] / 3;
    EXPECT(stationarity.lpNorm<Eigen::Infinity>() < 1e-8);
    EXPECT_DOUBLES_EQUAL(-3., result.lambdaEq[0](0), 1e-7);
    EXPECT_DOUBLES_EQUAL(9., result.lambdaIneq[0], 1e-7);
    EXPECT_LONGS_EQUAL(0, result.statistics.modelLinearizations);
    EXPECT_LONGS_EQUAL(1, result.statistics.objectiveFactorCopies);
    EXPECT_LONGS_EQUAL(1, result.statistics.symbolicFactorizations);
    EXPECT(result.statistics.modelUpdates > 1);
    EXPECT(result.statistics.constraintCoefficientUpdates <
           2 * result.statistics.modelUpdates);
    EXPECT(result.statistics.numericalEliminations >=
           result.statistics.modelUpdates);
    EXPECT_LONGS_EQUAL(result.iteration + 1, solver.progress().size());
    EXPECT(assert_equal(result.values, solver.optimize(), 1e-9));
    VectorValues infeasible;
    infeasible.insert(0, Vector2(10, -20));
    EXPECT(assert_equal(result.values, solver.optimize(infeasible), 1e-7));
  }
}

// Both frontends use the same BCL/Aggressive engine on equivalent scaled rows.
TEST(AugmentedLagrangianSolver, MatchesNonlinearAdapter) {
  for (auto policy : {AugmentedLagrangianUpdatePolicy::BCL,
                      AugmentedLagrangianUpdatePolicy::Aggressive}) {
    const auto costs = Objective(false);
    QpProblem problem;
    problem.addCost(*costs[0]);
    JacobianFactor equality(0, Matrix12::Ones(), Vector1(1));
    JacobianFactor inequality(0, Matrix12((Matrix12() << 1., 0.).finished()),
                              Vector1(.5));
    problem.addConstraint(LinearConstraint::Equal(equality, Vector1(2)));
    problem.addConstraint(LinearConstraint::LessEqual(inequality, Vector1(3)));
    auto p = Params();
    p->updatePolicy = policy;
    p->maxIterations = 60;
    Values initial;
    initial.insert(0, Vector(Vector2::Zero()));
    AugmentedLagrangianOptimizer optimizer(problem, initial, p);
    AugmentedLagrangianSolver solver(costs, Equalities(), Inequalities(), p);
    const auto nonlinear = optimizer.optimizeWithState();
    const auto direct = solver.optimizeWithState();
    EXPECT(nonlinear.converged == direct.converged);
    EXPECT(assert_equal(nonlinear.values.at<Vector>(0), direct.values.at(0),
                        1e-6));
    EXPECT_DOUBLES_EQUAL(nonlinear.augmentedLagrangianStationarity,
                         direct.augmentedLagrangianStationarity, 1e-7);
    EXPECT(direct.statistics.modelLinearizations == 0);
    EXPECT(nonlinear.statistics.modelLinearizations > 0);
  }
}

// Arbitrary objective constants must not prevent useful sub-roundoff steps.
TEST(AugmentedLagrangianSolver, ObjectiveConstantInvariance) {
  for (double constant : {0., .5, 1e6, 1e9}) {
    GaussianFactorGraph costs;
    costs.emplace_shared<HessianFactor>(0, Matrix11::Identity(), Vector1(1),
                                        2 * constant);
    auto p = Params();
    p->absoluteStationarityTolerance = 1e-12;
    const auto state =
        AugmentedLagrangianSolver(costs, {}, {}, p).optimizeWithState();
    EXPECT(state.converged);
    EXPECT_DOUBLES_EQUAL(1., state.values.at(0)(0), 1e-12);
    QpProblem qp;
    qp.addCost(*costs[0]);
    Values initial;
    initial.insert(0, Vector(Vector1::Zero()));
    const auto nonlinear =
        AugmentedLagrangianOptimizer(qp, initial, p).optimizeWithState();
    EXPECT(nonlinear.converged);
    EXPECT_DOUBLES_EQUAL(1., nonlinear.values.at<Vector>(0)(0), 1e-12);
  }
}

// Gaussian inequality blocks flatten to scalar rows without changing ordering.
TEST(AugmentedLagrangianSolver, BlockBoundsAndDependentRows) {
  GaussianFactorGraph inequalities;
  Matrix32 A;
  A << 1, 0, 1, 0, 0, -1;
  inequalities.emplace_shared<JacobianFactor>(0, A, Vector3(.5, .5, 0));
  const auto state =
      AugmentedLagrangianSolver(Objective(false), {}, inequalities, Params())
          .optimizeWithState();
  EXPECT(state.converged);
  EXPECT(assert_equal(Vector2(.5, 0), state.values.at(0), 1e-7));
  EXPECT_LONGS_EQUAL(3, state.lambdaIneq.size());
  EXPECT_DOUBLES_EQUAL(1.5, state.lambdaIneq[0] + state.lambdaIneq[1], 1e-7);
  EXPECT_DOUBLES_EQUAL(1., state.lambdaIneq[2], 1e-7);
}

// An equality supplies missing curvature without a permanent objective floor.
TEST(AugmentedLagrangianSolver, SingularCurvatureAndConstraintOnlyKey) {
  GaussianFactorGraph costs, equalities;
  costs.emplace_shared<HessianFactor>(0, Matrix11::Identity(), Vector1(2), 0);
  equalities.emplace_shared<JacobianFactor>(1, Matrix11::Identity(),
                                            Vector1(3));
  const auto state = AugmentedLagrangianSolver(costs, equalities, {}, Params())
                         .optimizeWithState();
  EXPECT(state.converged);
  EXPECT_DOUBLES_EQUAL(2., state.values.at(0)(0), 1e-7);
  EXPECT_DOUBLES_EQUAL(3., state.values.at(1)(0), 1e-7);
}

// Initial Matrix shapes survive the QpProblem convenience driver unchanged.
TEST(AugmentedLagrangianSolver, MatrixConvenienceDriver) {
  QpProblem problem;
  problem.addCost(
      HessianFactor(0, Matrix4::Identity(), Vector4(1, 3, -2, 4), 0));
  Values initial;
  initial.insert(0, Matrix(Matrix2::Zero()));
  const Values result =
      problem.optimize(initial, QpSolverType::AugmentedLagrangian);
  Matrix2 expected;
  expected << 1, -2, 3, 4;
  EXPECT(assert_equal(Matrix(expected), result.at<Matrix>(0), 1e-6));
  EXPECT(assert_equal(
      Vector(Vector4(1, 3, -2, 4)),
      problem.optimize(QpSolverType::AugmentedLagrangian).at<Vector>(0), 1e-6));
}

// Iteration limits and inconsistent equalities return explicit non-success
// states.
TEST(AugmentedLagrangianSolver, LimitsAndInconsistentRows) {
  auto p = Params();
  p->lmParams.maxIterations = 0;
  auto state = AugmentedLagrangianSolver(Objective(false), {}, {}, p)
                   .optimizeWithState();
  EXPECT(!state.converged);
  EXPECT(state.status == AugmentedLagrangianStatus::InnerIterationLimit);
  GaussianFactorGraph equalities;
  equalities.emplace_shared<JacobianFactor>(0, Matrix2::Identity(),
                                            Vector2::Zero());
  equalities.emplace_shared<JacobianFactor>(0, Matrix2::Identity(),
                                            Vector2::Ones());
  p = Params();
  p->maxIterations = 2;
  state = AugmentedLagrangianSolver(Objective(false), equalities, {}, p)
              .optimizeWithState();
  EXPECT(!state.converged);
  EXPECT(state.status == AugmentedLagrangianStatus::OuterIterationLimit);
}

// QP options reach ALM and select the requested leaf factorization.
TEST(AugmentedLagrangianSolver, QpQrOptions) {
  Matrix54 design;
  design << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1;
  const Vector4 target(2, -1, .5, 3);
  GaussianFactorGraph costs;
  costs.emplace_shared<JacobianFactor>(0, design.leftCols(1), 1,
                                       design.rightCols(3), design * target);
  costs.emplace_shared<JacobianFactor>(1, Matrix3::Identity(), target.tail(3));
  costs.emplace_shared<JacobianFactor>(
      2, design.leftCols(1), 1, design.rightCols(3),
      (design.leftCols(1) * Vector1(.75) + design.rightCols(3) * target.tail(3))
          .eval());
  QpProblem problem;
  for (const auto& factor : costs) problem.addCost(*factor);
  using QRMode = MultifrontalParameters::QRMode;
  for (QRMode mode : {QRMode::Off, QRMode::Allow, QRMode::Force}) {
    auto params = Params();
    params->lmParams.ordering = Ordering{0, 2, 1};
    auto& multifrontal = params->lmParams.multifrontalParams;
    multifrontal.leafMergeDimCap = multifrontal.mergeDimCap = 0;
    multifrontal.qrMode = mode;
    NonlinearMultifrontalSolver workspace(costs, Ordering{0, 2, 1},
                                          multifrontal);
    workspace.load(costs);
    bool usesQr = false;
    const auto visit = [&](const auto& self, const auto& clique) -> void {
      usesQr = usesQr || clique->useQR();
      for (const auto& child : clique->children) self(self, child);
    };
    for (const auto& root : workspace.roots()) visit(visit, root);
    EXPECT(usesQr == (mode != QRMode::Off));
    const auto direct =
        AugmentedLagrangianSolver(costs, {}, {}, params).optimizeWithState();
    EXPECT(direct.converged);
    EXPECT(assert_equal(Vector(Vector1(target(0))), direct.values.at(0), 1e-7));
    EXPECT(assert_equal(Vector(target.tail(3)), direct.values.at(1), 1e-7));
    const Values solved = problem.optimize(*params);
    EXPECT(assert_equal(direct.values.at(0), solved.at<Vector>(0), 1e-7));
    EXPECT(assert_equal(direct.values.at(1), solved.at<Vector>(1), 1e-7));
    // A zero budget makes ignoring the supplied options observable.
    params->lmParams.maxIterations = 0;
    Values initial;
    initial.insert(0, Vector(Vector1(.25)));
    initial.insert(1, Vector(Vector3(-.5, .5, -.5)));
    initial.insert(2, Vector(Vector1(-.25)));
    EXPECT(assert_equal(initial, problem.optimize(initial, *params)));
    const auto zero = problem.optimize(*params);
    EXPECT(assert_equal(Vector(Vector1::Zero()), zero.at<Vector>(0)));
    EXPECT(assert_equal(Vector(Vector3::Zero()), zero.at<Vector>(1)));
  }
}

// Nonfinite merit cannot be certified by an otherwise zero gradient.
TEST(AugmentedLagrangianSolver, NonfiniteMeritReportsFailure) {
  GaussianFactorGraph costs;
  costs.emplace_shared<HessianFactor>(0, Matrix11::Identity(), Vector1::Zero(),
                                      std::numeric_limits<double>::infinity());
  auto p = Params();
  const auto direct =
      AugmentedLagrangianSolver(costs, {}, {}, p).optimizeWithState();
  EXPECT(!direct.converged);
  EXPECT(direct.status == AugmentedLagrangianStatus::NumericalFailure);
  QpProblem problem;
  problem.addCost(*costs[0]);
  Values values;
  values.insert(0, Vector(Vector1::Zero()));
  const auto nonlinear =
      AugmentedLagrangianOptimizer(problem, values, p).optimizeWithState();
  EXPECT(!nonlinear.converged);
  EXPECT(nonlinear.status == AugmentedLagrangianStatus::NumericalFailure);
}

// Malformed inputs fail before entering the numerical solver.
TEST(AugmentedLagrangianSolver, RejectsMalformedValuesAndScales) {
  AugmentedLagrangianSolver solver(Objective(false), {}, {}, Params());
  VectorValues wrong;
  wrong.insert(0, Vector1(0));
  bool rejected = false;
  try {
    solver.optimize(wrong);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
  VectorValues nonfinite;
  nonfinite.insert(0, Vector2(std::numeric_limits<double>::quiet_NaN(), 0));
  rejected = false;
  try {
    solver.optimize(nonfinite);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
  GaussianFactorGraph invalidScales;
  invalidScales.emplace_shared<JacobianFactor>(0, Matrix12::Ones(), Vector1(0),
                                               noiseModel::Constrained::All(1));
  rejected = false;
  try {
    AugmentedLagrangianSolver(Objective(false), invalidScales, {}).optimize();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
  auto p = Params();
  p->lmParams.ordering = Ordering{1};
  rejected = false;
  try {
    AugmentedLagrangianSolver(Objective(false), {}, {}, p).optimize();
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  EXPECT(rejected);
}

}  // namespace analytic_qp
/* ************************************************************************* */

/* ************************************************************************* */
namespace seeded_qp {

// Seeded SPD QPs compare primal values and original KKT residuals with Dual QP.
TEST(AugmentedLagrangianSolver, SeededOriginalKkt) {
  std::mt19937 generator(4271);
  std::normal_distribution<double> random;
  for (size_t trial = 0; trial < 8; ++trial) {
    Matrix L(5, 5);
    Vector target(5);
    for (DenseIndex i = 0; i < L.size(); ++i)
      L.data()[i] = .2 * random(generator);
    for (double& value : target) value = random(generator);
    Matrix H = Matrix::Identity(5, 5) + L.transpose() * L;
    GaussianFactorGraph costs, inequalities;
    costs.emplace_shared<HessianFactor>(0, H, H * target, 0);
    Matrix A(10, 5);
    A.topRows(5) = Matrix::Identity(5, 5);
    A.bottomRows(5) = -Matrix::Identity(5, 5);
    inequalities.emplace_shared<JacobianFactor>(0, A, Vector::Constant(10, .5));
    auto p = analytic_qp::Params();
    p->maxIterations = 60;
    const auto result = AugmentedLagrangianSolver(costs, {}, inequalities, p)
                            .optimizeWithState();
    EXPECT(result.converged);
    QpProblem qp;
    qp.addCost(*costs[0]);
    for (size_t row = 0; row < 10; ++row)
      qp.addConstraint(LinearConstraint::LessEqual(
          JacobianFactor(0, A.row(row), Vector1(.5))));
    const Vector reference = qp.optimize().at<Vector>(0);
    EXPECT(assert_equal(reference, result.values.at(0), 1e-6));
    Vector lambda(10);
    for (size_t row = 0; row < 10; ++row) lambda(row) = result.lambdaIneq[row];
    const Vector gradient =
        H * (result.values.at(0) - target) + A.transpose() * lambda;
    EXPECT(gradient.lpNorm<Eigen::Infinity>() < 1e-7);
    EXPECT((A * result.values.at(0) - Vector::Constant(10, .5)).maxCoeff() <
           1e-7);
  }
}

}  // namespace seeded_qp
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

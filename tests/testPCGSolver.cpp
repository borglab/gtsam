/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPCGSolver.cpp
 * @brief   Unit tests for PCGSolver class
 * @author  Yong-Dian Jian
 * @date    Aug 06, 2014
 * @author Fan Jiang
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/FlatGaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/PCGSolver.h>
#include <gtsam/linear/SubgraphPreconditioner.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>
#include <tests/smallExample.h>

#include <Eigen/Cholesky>

using namespace std;
using namespace gtsam;

using symbol_shorthand::L;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace cholesky_fixture {

// Tests LLT decomposition and triangular back substitution.
TEST(PCGSolver, llt) {
  Matrix R{{1., -1., -1.}, {0., 2., -1.}, {0., 0., 1.}};
  Matrix AtA = R.transpose() * R;

  Vector Rvector{{1., -1., -1., 0., 2., -1., 0., 0., 1.}};

  Vector b = Vector3(1., 2., 3.);

  Vector x = Vector3(6.5, 2.5, 3.);

  /* test cholesky */
  Matrix R_hat = AtA.llt().matrixL().transpose();
  EXPECT(assert_equal(R, R_hat, 1e-5));

  /* test backward substitution */
  Vector xhat = R_hat.triangularView<Eigen::Upper>().solve(b);
  EXPECT(assert_equal(x, xhat, 1e-5));

  /* test in-place back substitution */
  xhat = b;
  R_hat.triangularView<Eigen::Upper>().solveInPlace(xhat);
  EXPECT(assert_equal(x, xhat, 1e-5));

  /* test triangular matrix map */
  Eigen::Map<Eigen::MatrixXd> R_adapter(Rvector.data(), 3, 3);
  xhat = R_adapter.transpose().triangularView<Eigen::Upper>().solve(b);
  EXPECT(assert_equal(x, xhat, 1e-5));
}

}  // namespace cholesky_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace vector_layout_fixture {

// The legacy converter accepts a partial ordering and ignores trailing values.
TEST(PCGSolver, LegacyBuildVectorValuesCompatibility) {
  const map<Key, size_t> dimensions{{0, 1}, {1, 2}, {2, 1}};
  const Ordering ordering{2, 0};
  const Vector flat{{4.0, 5.0, 99.0}};

  const VectorValues actual = buildVectorValues(flat, ordering, dimensions);
  const VectorValues expected{{2, Vector1(4.0)}, {0, Vector1(5.0)}};
  EXPECT(assert_equal(expected, actual));
}

}  // namespace vector_layout_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace graph_system_fixture {

// Tests GaussianFactorGraphSystem::multiply and getb.
TEST(GaussianFactorGraphSystem, multiply_getb) {
  // Create a Gaussian Factor Graph
  GaussianFactorGraph simpleGFG;
  SharedDiagonal unit2 = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.3));
  simpleGFG.emplace_shared<JacobianFactor>(2, Matrix{{10, 0}, {0, 10}},
                                           Vector{{-1, -1}}, unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, Matrix{{-10, 0}, {0, -10}}, 0,
                                           Matrix{{10, 0}, {0, 10}},
                                           Vector{{2, -1}}, unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, Matrix{{-5, 0}, {0, -5}}, 1,
                                           Matrix{{5, 0}, {0, 5}},
                                           Vector{{0, 1}}, unit2);
  simpleGFG.emplace_shared<JacobianFactor>(0, Matrix{{-5, 0}, {0, -5}}, 1,
                                           Matrix{{5, 0}, {0, 5}},
                                           Vector{{-1, 1.5}}, unit2);
  simpleGFG.emplace_shared<JacobianFactor>(0, Matrix{{1, 0}, {0, 1}},
                                           Vector{{0, 0}}, unit2);
  simpleGFG.emplace_shared<JacobianFactor>(1, Matrix{{1, 0}, {0, 1}},
                                           Vector{{0, 0}}, unit2);
  simpleGFG.emplace_shared<JacobianFactor>(2, Matrix{{1, 0}, {0, 1}},
                                           Vector{{0, 0}}, unit2);

  // Create a dummy-preconditioner and a GaussianFactorGraphSystem
  DummyPreconditioner dummyPreconditioner;
  KeyInfo keyInfo(simpleGFG);
  std::map<Key, Vector> lambda;
  dummyPreconditioner.build(simpleGFG, keyInfo, lambda);
  GaussianFactorGraphSystem system(simpleGFG, dummyPreconditioner, keyInfo,
                                   lambda);

  // Prepare container for each variable
  Vector initial, residual, preconditionedResidual, p, actualAp;
  initial = Vector{{0., 0., 0., 0., 0., 0.}};

  // Calculate values using GaussianFactorGraphSystem same as inside of
  // PCGSolver
  system.residual(initial, residual); /* r = b-Ax */
  system.leftPrecondition(residual,
                          preconditionedResidual);     /* pr = L^{-1} (b-Ax) */
  system.rightPrecondition(preconditionedResidual, p); /* p = L^{-T} pr */
  system.multiply(p, actualAp);                        /* A p */

  // Expected value of Ap for the first iteration of this example problem
  Vector expectedAp{
      {100400, -249074.074, -2080, 148148.148, -146480, 37962.963}};
  EXPECT(assert_equal(expectedAp, actualAp, 1e-3));

  // Expected value of getb
  Vector expectedb{{100.0, -194.444, -20.0, 138.889, -120.0, -55.556}};
  Vector actualb;
  system.getb(actualb);
  EXPECT(assert_equal(expectedb, actualb, 1e-3));
}

}  // namespace graph_system_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace nonlinear_solver_fixture {

constexpr double kTolerance = 1e-3;

// Tests optimization with the dummy preconditioner.
TEST(PCGSolver, dummy) {
  LevenbergMarquardtParams params;
  params.linearSolverType = LevenbergMarquardtParams::Iterative;
  auto pcg = std::make_shared<PCGSolverParameters>(
      std::make_shared<DummyPreconditionerParameters>());
  params.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10, 10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, params).optimize();

  DOUBLES_EQUAL(0, fg.error(actualPCG), kTolerance);
}

// Tests optimization with the block-Jacobi preconditioner.
TEST(PCGSolver, blockjacobi) {
  LevenbergMarquardtParams params;
  params.linearSolverType = LevenbergMarquardtParams::Iterative;
  auto pcg = std::make_shared<PCGSolverParameters>(
      std::make_shared<BlockJacobiPreconditionerParameters>());
  params.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10, 10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, params).optimize();

  DOUBLES_EQUAL(0, fg.error(actualPCG), kTolerance);
}

// Tests optimization with the incremental subgraph preconditioner.
TEST(PCGSolver, subgraph) {
  LevenbergMarquardtParams params;
  params.linearSolverType = LevenbergMarquardtParams::Iterative;
  auto pcg = std::make_shared<PCGSolverParameters>(
      std::make_shared<SubgraphPreconditionerParameters>());
  params.iterativeParams = pcg;

  NonlinearFactorGraph fg = example::createReallyNonlinearFactorGraph();

  Point2 x0(10, 10);
  Values c0;
  c0.insert(X(1), x0);

  Values actualPCG = LevenbergMarquardtOptimizer(fg, c0, params).optimize();

  DOUBLES_EQUAL(0, fg.error(actualPCG), kTolerance);
}

}  // namespace nonlinear_solver_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace detailed_result_fixture {

GaussianFactorGraph createTwoDimensionalGraph() {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(
      0, (Matrix(2, 2) << 2.0, 0.5, 0.0, 3.0).finished(),
      (Vector(2) << 1.0, 2.0).finished(), noiseModel::Unit::Create(2));
  return graph;
}

PCGSolverParameters createParameters() {
  PCGSolverParameters parameters(
      std::make_shared<DummyPreconditionerParameters>());
  parameters.minIterations = 0;
  parameters.maxIterations = 10;
  parameters.reset = 11;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 1e-12;
  return parameters;
}

struct NegativeCurvatureSystem {
  void residual(const Vector& x, Vector& residual) const {
    residual = Vector1(1.0 + x(0));
  }
  void multiply(const Vector& x, Vector& product) const { product = -x; }
  void leftPrecondition(const Vector& x, Vector& y) const { y = x; }
  void rightPrecondition(const Vector& x, Vector& y) const { y = x; }
  void scal(double alpha, Vector& x) const { x *= alpha; }
  double dot(const Vector& x, const Vector& y) const { return x.dot(y); }
  void axpy(double alpha, const Vector& x, Vector& y) const { y += alpha * x; }
};

// Verifies the detailed API reports convergence and an aligned residual trace.
TEST(PCGSolver, DetailedResult) {
  const GaussianFactorGraph graph = createTwoDimensionalGraph();
  const PCGSolverResult result =
      PCGSolver(createParameters()).optimizeDetailed(graph);
  const VectorValues expected = graph.optimize();

  EXPECT(assert_equal(expected, result.solution, 1e-10));
  CHECK(result.stats.converged());
  LONGS_EQUAL(
      static_cast<long>(result.stats.iterations + 1),
      static_cast<long>(result.stats.preconditionedResidualNormHistory.size()));
  DOUBLES_EQUAL(result.stats.finalPreconditionedResidualNorm,
                result.stats.preconditionedResidualNormHistory.back(), 1e-15);
  CHECK(result.operatorSetupSeconds >= 0.0);
  CHECK(result.preconditionerSetupSeconds >= 0.0);
  CHECK(result.solveSeconds >= 0.0);
}

// Verifies null linear factors are ignored when PCG discovers key dimensions.
TEST(PCGSolver, DetailedResultWithNullFactor) {
  const GaussianFactorGraph graph = createTwoDimensionalGraph();
  GaussianFactorGraph graphWithNull = graph;
  graphWithNull.push_back(GaussianFactor::shared_ptr());

  const PCGSolverResult expected =
      PCGSolver(createParameters()).optimizeDetailed(graph, false);
  const PCGSolverResult actual =
      PCGSolver(createParameters()).optimizeDetailed(graphWithNull, false);

  EXPECT(assert_equal(expected.solution, actual.solution, 1e-12));
  CHECK(actual.stats.converged());
}

// Verifies a fixed one-iteration solve reports the iteration limit.
TEST(PCGSolver, DetailedMaxIterations) {
  const GaussianFactorGraph graph = createTwoDimensionalGraph();
  PCGSolverParameters parameters = createParameters();
  parameters.minIterations = 1;
  parameters.maxIterations = 1;
  parameters.epsilon_rel = 0.0;
  const PCGSolverResult result = PCGSolver(parameters).optimizeDetailed(graph);

  LONGS_EQUAL(1, static_cast<long>(result.stats.iterations));
  CHECK(result.stats.terminationReason ==
        ConjugateGradientTerminationReason::kMaxIterations);
}

// Verifies non-positive curvature is reported before an invalid division.
TEST(PCGSolver, DetailedNumericalBreakdown) {
  ConjugateGradientParameters parameters;
  parameters.minIterations = 0;
  parameters.maxIterations = 5;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 0.0;
  const Vector initial = Vector1::Zero();
  const auto result = preconditionedConjugateGradientDetailed(
      NegativeCurvatureSystem(), initial, parameters);

  LONGS_EQUAL(0, static_cast<long>(result.stats.iterations));
  CHECK(result.stats.terminationReason ==
        ConjugateGradientTerminationReason::kNumericalBreakdown);
  EXPECT(assert_equal(Vector1::Zero(), result.solution));
}

}  // namespace detailed_result_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace mixed_factor_fixture {

// Verifies flat Jacobian, Hessian, and compact-Batch plans compose correctly.
TEST(GaussianFactorGraphSystem, MixedFactorTypes) {
  const Key firstKey = 10;
  const Key secondKey = 2;
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(
      firstKey, (Matrix(2, 2) << 1.0, 0.2, -0.3, 2.0).finished(),
      (Vector(2) << 0.5, -1.0).finished(),
      noiseModel::Diagonal::Sigmas(Vector2(2.0, 3.0)));
  const JacobianFactor hessianSource(
      secondKey, (Matrix(2, 2) << 0.7, -0.1, 0.4, 1.3).finished(),
      (Vector(2) << -0.2, 0.8).finished());
  graph.emplace_shared<HessianFactor>(hessianSource);

  using Batch = BatchJacobianFactor<1, 2, 2>;
  auto batch = std::make_shared<Batch>(
      KeyVector{firstKey, secondKey}, std::vector<size_t>{2, 2},
      noiseModel::Diagonal::Sigmas(Vector2(1.5, 2.5)));
  CHECK(dynamic_cast<const FlatGaussianFactor*>(batch.get()) != nullptr);
  batch->addRow({0, 1},
                {(Matrix(1, 2) << 0.5, -0.25).finished(),
                 (Matrix(1, 2) << 1.0, 0.75).finished()},
                Vector1(0.3));
  batch->addRow({0, 1},
                {(Matrix(1, 2) << -0.1, 0.8).finished(),
                 (Matrix(1, 2) << 0.4, -0.6).finished()},
                Vector1(-0.7));
  graph.push_back(batch);

  const Ordering ordering{firstKey, secondKey};
  const KeyInfo keyInfo(graph, ordering);
  DummyPreconditioner preconditioner;
  preconditioner.build(graph, keyInfo, {});
  const GaussianFactorGraphSystem system(graph, preconditioner, keyInfo, {});
  const Vector x = (Vector(4) << 0.2, -0.4, 1.1, 0.3).finished();

  const VectorValues vectorValuesX = buildVectorValues(x, keyInfo);
  VectorValues expectedValues = keyInfo.x0();
  graph.multiplyHessianAdd(1.0, vectorValuesX, expectedValues);
  const Vector expected = expectedValues.vector(ordering);
  Vector actual;
  system.multiply(x, actual);
  EXPECT(assert_equal(expected, actual, 1e-12));

  Vector inPlace = x;
  system.multiply(inPlace, inPlace);
  EXPECT(assert_equal(expected, inPlace, 1e-12));

  Vector expectedRhs = -graph.gradientAtZero().vector(ordering);
  Vector actualRhs;
  system.getb(actualRhs);
  EXPECT(assert_equal(expectedRhs, actualRhs, 1e-12));
}

}  // namespace mixed_factor_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace flat_gaussian_factor_fixture {

using ImplicitFactor = RegularImplicitSchurFactor<CalibratedCamera>;

ImplicitFactor::shared_ptr createImplicitFactor(Key firstKey, Key secondKey) {
  const Matrix26 firstBlock = (Matrix26() << 1.0, 0.2, -0.1, 0.3, 0.5, -0.4,
                               -0.2, 0.8, 0.4, -0.3, 0.1, 0.6)
                                  .finished();
  const Matrix26 secondBlock = (Matrix26() << -0.5, 0.3, 0.7, -0.2, 0.4, 0.1,
                                0.6, -0.4, 0.2, 0.9, -0.1, 0.5)
                                   .finished();
  const std::vector<Matrix26, Eigen::aligned_allocator<Matrix26>> blocks{
      firstBlock, secondBlock};
  const Matrix E = (Matrix(4, 3) << 0.2, -0.1, 0.3, 0.4, 0.2, -0.2, -0.3, 0.5,
                    0.1, 0.1, -0.4, 0.6)
                       .finished();
  const Matrix3 pointCovariance = 0.05 * Matrix3::Identity();
  const Vector b = (Vector(4) << 0.4, -0.2, 0.1, 0.3).finished();
  return std::make_shared<ImplicitFactor>(KeyVector{firstKey, secondKey}, blocks,
                                          E, pointCovariance, b);
}

GaussianFactorGraph createMixedFlatGaussianGraph() {
  const Key firstKey = 10;
  const Key secondKey = 2;
  GaussianFactorGraph graph;

  const auto unit6 = noiseModel::Unit::Create(6);
  // Add ordinary Jacobian factors handled by the compiled sparse path.
  graph.emplace_shared<JacobianFactor>(
      firstKey, Matrix6::Identity(),
      (Vector6() << 0.2, -0.1, 0.4, 0.3, -0.2, 0.1).finished(), unit6);
  graph.emplace_shared<JacobianFactor>(
      secondKey, 1.5 * Matrix6::Identity(),
      (Vector6() << -0.3, 0.5, -0.2, 0.1, 0.4, -0.1).finished(), unit6);

  // Add an implicit Schur factor handled by the flat Gaussian-factor path.
  graph.push_back(createImplicitFactor(firstKey, secondKey));
  return graph;
}

Vector legacyProduct(const GaussianFactorGraph& graph, const KeyInfo& keyInfo,
                     const Vector& x) {
  const VectorValues valuesX = buildVectorValues(x, keyInfo);
  VectorValues valuesY = keyInfo.x0();
  graph.multiplyHessianAdd(1.0, valuesX, valuesY);
  return valuesY.vector(keyInfo.ordering());
}

Matrix legacyHessian(const GaussianFactorGraph& graph, const KeyInfo& keyInfo) {
  const DenseIndex dimension = static_cast<DenseIndex>(keyInfo.numCols());
  Matrix hessian(dimension, dimension);
  for (DenseIndex column = 0; column < dimension; ++column) {
    Vector basis = Vector::Zero(dimension);
    basis(column) = 1.0;
    hessian.col(column) = legacyProduct(graph, keyInfo, basis);
  }
  return hessian;
}

// Verifies sparse Jacobian and flat-factor contributions compose in products,
// the right-hand side, and a complete block-Jacobi PCG solve.
TEST(GaussianFactorGraphSystem, MixedSupportedAndFlatGaussianFactors) {
  const GaussianFactorGraph graph = createMixedFlatGaussianGraph();
  const Ordering ordering{10, 2};
  const KeyInfo keyInfo(graph, ordering);
  DummyPreconditioner dummy;
  dummy.build(graph, keyInfo, {});
  const GaussianFactorGraphSystem system(graph, dummy, keyInfo, {}, false, 1);

  const Vector x =
      Vector::LinSpaced(static_cast<DenseIndex>(keyInfo.numCols()), -0.6, 0.7);
  Vector actualProduct;
  system.multiply(x, actualProduct);
  EXPECT(assert_equal(legacyProduct(graph, keyInfo, x), actualProduct, 1e-12));

  const Vector expectedRhs = -graph.gradientAtZero().vector(ordering);
  Vector actualRhs;
  system.getb(actualRhs);
  EXPECT(assert_equal(expectedRhs, actualRhs, 1e-12));

  PCGSolverParameters parameters(
      std::make_shared<BlockJacobiPreconditionerParameters>());
  parameters.minIterations = 0;
  parameters.maxIterations = 100;
  parameters.reset = 101;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 1e-12;
  parameters.parallel = false;
  const PCGSolverResult result =
      PCGSolver(parameters)
          .optimizeDetailed(graph, keyInfo, {}, keyInfo.x0(), false);
  const Vector expectedSolution =
      legacyHessian(graph, keyInfo).ldlt().solve(expectedRhs);

  CHECK(result.stats.converged());
  EXPECT(
      assert_equal(expectedSolution, result.solution.vector(ordering), 1e-10));
}

// Verifies the parallel flat-factor reduction matches serial keyed execution.
TEST(GaussianFactorGraphSystem, ParallelFlatGaussianFactors) {
  const Key firstKey = 10;
  const Key secondKey = 2;
  GaussianFactorGraph graph;
  for (size_t index = 0; index < 256; ++index) {
    graph.push_back(createImplicitFactor(firstKey, secondKey));
  }

  const Ordering ordering{firstKey, secondKey};
  const KeyInfo keyInfo(graph, ordering);
  DummyPreconditioner preconditioner;
  preconditioner.build(graph, keyInfo, {});
  const GaussianFactorGraphSystem parallel(graph, preconditioner, keyInfo, {},
                                            true, 2);
  LONGS_EQUAL(2, static_cast<long>(parallel.numThreads()));

  const Vector input =
      Vector::LinSpaced(static_cast<DenseIndex>(keyInfo.numCols()), -0.4, 0.6);
  Vector actual;
  parallel.multiply(input, actual);
  EXPECT(assert_equal(legacyProduct(graph, keyInfo, input), actual, 1e-10));
}

}  // namespace flat_gaussian_factor_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace parallel_pcg_fixture {

GaussianFactorGraph createDenseChain(size_t variableCount) {
  const Matrix3 firstBlock =
      (Matrix3() << 1.0, 0.2, -0.1, -0.3, 0.9, 0.4, 0.15, -0.25, 1.1)
          .finished();
  const Matrix3 secondBlock =
      (Matrix3() << -0.8, 0.1, 0.3, 0.2, -1.2, 0.15, -0.1, 0.35, -0.7)
          .finished();
  const auto model = noiseModel::Unit::Create(3);

  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(0, Matrix3::Identity(), Vector3::Zero(),
                                       model);
  for (size_t key = 1; key < variableCount; ++key) {
    const Vector3 rhs(0.01 * static_cast<double>(key % 7), -0.02, 0.03);
    graph.emplace_shared<JacobianFactor>(key - 1, firstBlock, key, secondBlock,
                                         rhs, model);
  }
  return graph;
}

// Verifies scheduler execution is enabled by default but remains configurable.
TEST(PCGSolver, ParallelParametersDefaultOn) {
  const PCGSolverParameters parameters(
      std::make_shared<DummyPreconditionerParameters>());
  CHECK(parameters.parallel);
  LONGS_EQUAL(0, static_cast<long>(parameters.numThreads));
}

// Verifies parallel products, preconditioning, and PCG match the serial path.
TEST(PCGSolver, ParallelMatchesSerial) {
  const GaussianFactorGraph graph = createDenseChain(4096);
  const KeyInfo keyInfo(graph);
  BlockJacobiPreconditioner preconditioner;
  preconditioner.build(graph, keyInfo, {});

  const GaussianFactorGraphSystem serial(graph, preconditioner, keyInfo, {},
                                         false, 1);
  const GaussianFactorGraphSystem parallel(graph, preconditioner, keyInfo, {},
                                           true, 2);
  LONGS_EQUAL(1, static_cast<long>(serial.numThreads()));
  LONGS_EQUAL(2, static_cast<long>(parallel.numThreads()));

  const Vector input =
      Vector::LinSpaced(static_cast<DenseIndex>(keyInfo.numCols()), -0.5, 0.5);
  Vector serialProduct, parallelProduct;
  serial.multiply(input, serialProduct);
  parallel.multiply(input, parallelProduct);
  EXPECT(assert_equal(serialProduct, parallelProduct, 1e-12));

  Vector serialLeft, parallelLeft, serialRight, parallelRight;
  serial.leftPrecondition(input, serialLeft);
  parallel.leftPrecondition(input, parallelLeft);
  serial.rightPrecondition(input, serialRight);
  parallel.rightPrecondition(input, parallelRight);
  EXPECT(assert_equal(serialLeft, parallelLeft, 1e-12));
  EXPECT(assert_equal(serialRight, parallelRight, 1e-12));

  ConjugateGradientParameters parameters;
  parameters.minIterations = 5;
  parameters.maxIterations = 5;
  parameters.reset = 6;
  parameters.epsilon_abs = 0.0;
  parameters.epsilon_rel = 0.0;
  const Vector zero = Vector::Zero(input.size());
  const Vector serialSolution =
      preconditionedConjugateGradient(serial, zero, parameters);
  const Vector parallelSolution =
      preconditionedConjugateGradient(parallel, zero, parameters);
  EXPECT(assert_equal(serialSolution, parallelSolution, 1e-12));
}

}  // namespace parallel_pcg_fixture
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

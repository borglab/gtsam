/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testMultifrontalSolver.cpp
 * @brief Unit tests for MultifrontalSolver.
 * @author Frank Dellaert
 * @date   December 2025
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/MatrixConstants.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/linear/internal/CompactLeafSchurKernel.h>
#include <gtsam/nonlinear/Marginals.h>
#include <tests/smallExample.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

using namespace std;
using namespace gtsam;
using symbol_shorthand::L;
using symbol_shorthand::X;

/* ************************************************************************* */
namespace compact_leaf_schur_kernel_fixture {

// Verifies generic in-place factorization and non-contiguous mapped Schur
// subtraction against dense reference calculations.
TEST(CompactLeafSchurKernel, FactorAndScatter) {
  const std::vector<size_t> sourceDimensions{2, 1, 2};
  VerticalBlockMatrix rows(sourceDimensions, 2, true);
  const Matrix Hff{{4.0, 1.0}, {1.0, 3.0}};
  const Matrix retained{{0.5, -1.0, 2.0, 0.25}, {1.5, 0.75, -0.5, -0.125}};
  rows.matrix() << Hff, retained;

  Eigen::LLT<Matrix, Eigen::Upper> referenceFactorization(Hff);
  Matrix expectedRows(2, 6);
  expectedRows.leftCols(2) = referenceFactorization.matrixU();
  expectedRows.rightCols(4) = retained;
  referenceFactorization.matrixU().transpose().solveInPlace(
      expectedRows.rightCols(4));

  EXPECT(internal::CompactLeafSchurKernel::factorFrontalRows(&rows, 2));
  EXPECT(assert_equal(expectedRows, rows.matrix(), 1e-12));

  SymmetricBlockMatrix target(std::vector<size_t>{1, 2, 3, 2}, true);
  target.setAllZero();
  const std::vector<DenseIndex> blockOffsets{
      target.blockScalarOffset(2), target.blockScalarOffset(0),
      target.blockScalarOffset(target.nBlocks() - 1)};
  const std::vector<DenseIndex> scalarOffsets =
      internal::CompactLeafSchurKernel::expandScalarOffsets(
          std::vector<size_t>{1, 2, 1}, blockOffsets);
  internal::CompactLeafSchurKernel::subtractMappedOuterProduct(
      rows, 2, scalarOffsets, &target);

  Matrix expected = Matrix::Zero(target.rows(), target.cols());
  const Matrix schur =
      expectedRows.rightCols(4).transpose() * expectedRows.rightCols(4);
  for (DenseIndex column = 0; column < 4; ++column) {
    for (DenseIndex row = 0; row <= column; ++row) {
      const DenseIndex targetRow = scalarOffsets[row];
      const DenseIndex targetColumn = scalarOffsets[column];
      expected(std::min(targetRow, targetColumn),
               std::max(targetRow, targetColumn)) -= schur(row, column);
    }
  }
  const Matrix expectedSymmetric = expected.selfadjointView<Eigen::Upper>();
  const Matrix actualSymmetric = target.selfadjointView();
  EXPECT(assert_equal(expectedSymmetric, actualSymmetric, 1e-12));
}

// Verifies indefinite and scale-normalized near-zero pivots are rejected.
TEST(CompactLeafSchurKernel, RejectsInvalidPivots) {
  const std::vector<size_t> dimensions{2};
  VerticalBlockMatrix indefinite(dimensions, 2, true);
  indefinite.matrix() << 1.0, 2.0, 0.0, 2.0, 1.0, 0.0;
  EXPECT(!internal::CompactLeafSchurKernel::factorFrontalRows(&indefinite, 2));

  VerticalBlockMatrix nearSingular(dimensions, 2, true);
  nearSingular.matrix() << 1.0, 1.0, 0.0, 1.0, 1.0 + 1e-10, 0.0;
  EXPECT(
      !internal::CompactLeafSchurKernel::factorFrontalRows(&nearSingular, 2));
}

}  // namespace compact_leaf_schur_kernel_fixture
/* ************************************************************************* */

namespace {
const Key x1 = 1, x2 = 2, x3 = 3, x4 = 4;
const SharedDiagonal chainNoise1 = noiseModel::Isotropic::Sigma(1, 0.5);
const SharedDiagonal chainNoise2 = noiseModel::Isotropic::Sigma(1, 1.0);
const SharedDiagonal chainNoise3 = noiseModel::Isotropic::Sigma(1, 2.0);
const SharedDiagonal chainNoise4 = noiseModel::Isotropic::Sigma(1, 0.25);
const GaussianFactorGraph chain = {
    std::make_shared<JacobianFactor>(x2, I_1x1, x1, I_1x1, I_1x1, chainNoise1),
    std::make_shared<JacobianFactor>(x2, I_1x1, x3, I_1x1, I_1x1, chainNoise2),
    std::make_shared<JacobianFactor>(x3, I_1x1, x4, I_1x1, I_1x1, chainNoise3),
    std::make_shared<JacobianFactor>(x4, I_1x1, I_1x1, chainNoise4)};
const Ordering chainOrdering{x2, x1, x3, x4};

MultifrontalSolver::Parameters noMergeParams() {
  MultifrontalSolver::Parameters params;
  params.leafMergeDimCap = 0;
  params.mergeDimCap = 0;
  params.leafAggregationProblemSize = 0;
  return params;
}

}  // namespace

/* ************************************************************************* */
namespace partial_elimination_fixture {

const Key point1 = L(1), point2 = L(2);
const Key camera1 = X(1), camera2 = X(2);
const Ordering pointOrdering{point1, point2};
const Ordering cameraOrdering{camera1, camera2};
const Ordering fullOrdering{point1, point2, camera1, camera2};

GaussianFactorGraph createGraph(double rhsScale = 1.0) {
  const auto model = noiseModel::Unit::Create(1);
  return GaussianFactorGraph{
      std::make_shared<JacobianFactor>(point1, I_1x1, camera1, -I_1x1,
                                       Vector1(rhsScale), model),
      std::make_shared<JacobianFactor>(point1, I_1x1, camera2, -I_1x1,
                                       Vector1(2.0 * rhsScale), model),
      std::make_shared<JacobianFactor>(point2, I_1x1, camera1, -I_1x1,
                                       Vector1(3.0 * rhsScale), model),
      std::make_shared<JacobianFactor>(point2, I_1x1, camera2, -I_1x1,
                                       Vector1(4.0 * rhsScale), model),
      std::make_shared<JacobianFactor>(camera1, I_1x1, Vector1(0.5 * rhsScale),
                                       model)};
}

bool checkPartialElimination(MultifrontalSolver* solver,
                             const GaussianFactorGraph& graph) {
  solver->eliminatePartialInPlace(graph);
  const GaussianFactorGraph actualRemaining = solver->remainingFactorGraph();
  const auto [pointBayesTree, expectedRemaining] =
      graph.eliminatePartialMultifrontal(pointOrdering);
  (void)pointBayesTree;

  const bool remainingFactorsAgree =
      assert_equal(expectedRemaining->augmentedHessian(cameraOrdering),
                   actualRemaining.augmentedHessian(cameraOrdering), 1e-9);
  const bool cliquesAreAggregated =
      actualRemaining.size() < expectedRemaining->size();

  const VectorValues retainedSolution = actualRemaining.optimize();
  const VectorValues& actualSolution = solver->updateSolution(retainedSolution);
  const VectorValues expectedSolution = graph.optimize();
  return remainingFactorsAgree && cliquesAreAggregated &&
         assert_equal(expectedSolution, actualSolution, 1e-9);
}

// Partially eliminates point frontals, exports clique-level camera factors,
// and reuses the symbolic structure after numerical values change.
TEST(MultifrontalSolver, PartialElimination) {
  const GaussianFactorGraph graph = createGraph();
  MultifrontalSolver solver(graph, fullOrdering, pointOrdering.size(),
                            noMergeParams());

  EXPECT(checkPartialElimination(&solver, graph));
  EXPECT(checkPartialElimination(&solver, createGraph(2.0)));
}

// Leaf aggregation changes only task scheduling, not clique structure or the
// reduced system produced by partial elimination.
TEST(MultifrontalSolver, PartialEliminationLeafAggregation) {
  const GaussianFactorGraph graph = createGraph();

  MultifrontalSolver::Parameters separateTasks = noMergeParams();
  separateTasks.qrMode = MultifrontalParameters::QRMode::Off;
  MultifrontalSolver solverSeparate(graph, fullOrdering, pointOrdering.size(),
                                    separateTasks);
  solverSeparate.eliminatePartialInPlace(graph);

  const Matrix expected =
      solverSeparate.remainingFactorGraph().augmentedHessian(cameraOrdering);
  for (const auto mode : {MultifrontalParameters::LeafMode::Bounded,
                          MultifrontalParameters::LeafMode::SameSeparator}) {
    MultifrontalSolver::Parameters aggregatedTasks = noMergeParams();
    aggregatedTasks.qrMode = MultifrontalParameters::QRMode::Off;
    aggregatedTasks.leafAggregationProblemSize = 2048;
    aggregatedTasks.leafMode = mode;
    MultifrontalSolver solverAggregated(graph, fullOrdering,
                                        pointOrdering.size(), aggregatedTasks);
    solverAggregated.eliminatePartialInPlace(graph);

    EXPECT_LONGS_EQUAL(solverSeparate.cliqueCount(),
                       solverAggregated.cliqueCount());
    EXPECT(
        assert_equal(expected,
                     solverAggregated.remainingFactorGraph().augmentedHessian(
                         cameraOrdering),
                     1e-9));
  }
}

}  // namespace partial_elimination_fixture
/* ************************************************************************* */

namespace algebraic_leaf_merge_fixture {

struct Problem {
  GaussianFactorGraph graph;
  Ordering pointOrdering;
  Ordering cameraOrdering;
  Ordering fullOrdering;
};

Problem createProblem(size_t pointCount, bool differentLastSeparator = false,
                      double rhsScale = 1.0) {
  Problem problem;
  const KeyVector cameras{X(10), X(11), X(12)};
  problem.cameraOrdering = Ordering(cameras.begin(), cameras.end());
  const auto unit = noiseModel::Unit::Create(1);
  for (size_t index = 0; index < pointCount; ++index) {
    const Key point = L(10 + index);
    const Key secondCamera = differentLastSeparator && index + 1 == pointCount
                                 ? cameras[2]
                                 : cameras[1];
    problem.pointOrdering.push_back(point);
    problem.graph.emplace_shared<JacobianFactor>(
        point, I_1x1, cameras[0], I_1x1,
        Vector1(rhsScale * (1.0 + static_cast<double>(index))), unit);
    problem.graph.emplace_shared<JacobianFactor>(
        point, I_1x1, secondCamera, -I_1x1,
        Vector1(rhsScale * (2.0 + static_cast<double>(index))), unit);
  }
  for (const Key camera : cameras) {
    problem.graph.emplace_shared<JacobianFactor>(camera, I_1x1, Vector1::Zero(),
                                                 unit);
  }
  for (size_t first = 0; first < cameras.size(); ++first) {
    for (size_t second = first + 1; second < cameras.size(); ++second) {
      problem.graph.emplace_shared<JacobianFactor>(cameras[first], I_1x1,
                                                   cameras[second], -I_1x1,
                                                   Vector1::Zero(), unit);
    }
  }
  problem.fullOrdering = problem.pointOrdering;
  problem.fullOrdering.insert(problem.fullOrdering.end(), cameras.begin(),
                              cameras.end());
  return problem;
}

std::vector<KeyVector> leafFrontals(MultifrontalSolver* solver) {
  std::vector<KeyVector> result;
  solver->runTopDown([&result](MultifrontalClique& clique) {
    if (!clique.children.empty()) return;
    const auto frontalEnd = clique.orderedKeys().begin() + clique.numFrontals();
    result.emplace_back(clique.orderedKeys().begin(), frontalEnd);
  });
  return result;
}

bool containsFrontals(const std::vector<KeyVector>& leaves,
                      const KeyVector& expected) {
  return std::find(leaves.begin(), leaves.end(), expected) != leaves.end();
}

// Compatible multi-factor leaves merge algebraically, while disabling the
// dedicated cap leaves task aggregation free to operate on the original tree.
TEST(MultifrontalSolver, AlgebraicLeafMergeAndTaskAggregationIndependent) {
  const Problem problem = createProblem(2);
  auto separate = noMergeParams();
  MultifrontalSolver separateSolver(problem.graph, problem.fullOrdering,
                                    separate);

  auto tasksOnly = separate;
  tasksOnly.leafAggregationProblemSize = 2048;
  MultifrontalSolver taskSolver(problem.graph, problem.fullOrdering, tasksOnly);
  EXPECT_LONGS_EQUAL(separateSolver.cliqueCount(), taskSolver.cliqueCount());

  auto algebraic = separate;
  algebraic.leafMergeDimCap = 256;
  MultifrontalSolver mergedSolver(problem.graph, problem.fullOrdering,
                                  algebraic);
  EXPECT_LONGS_EQUAL(separateSolver.cliqueCount() - 1,
                     mergedSolver.cliqueCount());
  EXPECT(
      containsFrontals(leafFrontals(&mergedSolver), KeyVector{L(10), L(11)}));
}

// The historical dimension cap splits same-separator leaves in stable order,
// and leaves with a different separator stay outside the merged batch.
TEST(MultifrontalSolver, AlgebraicLeafMergeCapAndSeparator) {
  const Problem sameSeparator = createProblem(3);
  auto capped = noMergeParams();
  capped.leafMergeDimCap = 6;
  MultifrontalSolver cappedSolver(sameSeparator.graph,
                                  sameSeparator.fullOrdering, capped);
  const std::vector<KeyVector> cappedLeaves = leafFrontals(&cappedSolver);
  EXPECT(containsFrontals(cappedLeaves, KeyVector{L(10), L(11)}));
  EXPECT(containsFrontals(cappedLeaves, KeyVector{L(12)}));

  const Problem differentSeparator = createProblem(3, true);
  capped.leafMergeDimCap = 256;
  MultifrontalSolver separatedSolver(differentSeparator.graph,
                                     differentSeparator.fullOrdering, capped);
  const std::vector<KeyVector> separatedLeaves = leafFrontals(&separatedSolver);
  EXPECT(containsFrontals(separatedLeaves, KeyVector{L(10), L(11)}));
  EXPECT(containsFrontals(separatedLeaves, KeyVector{L(12)}));
}

// Merged ordinary leaves agree with generic full and partial elimination and
// remain correct when compatible numerical values are loaded again.
TEST(MultifrontalSolver, AlgebraicLeafMergeNumericsAndReload) {
  const Problem problem = createProblem(3);
  auto params = noMergeParams();
  params.leafMergeDimCap = 256;
  params.qrMode = MultifrontalParameters::QRMode::Off;

  MultifrontalSolver fullSolver(problem.graph, problem.fullOrdering, params);
  const VectorValues expected = problem.graph.optimize(problem.fullOrdering);
  fullSolver.eliminateInPlace(problem.graph);
  EXPECT(assert_equal(expected, fullSolver.updateSolution(), 1e-9));

  const Problem changed = createProblem(3, false, 2.0);
  const VectorValues changedExpected =
      changed.graph.optimize(changed.fullOrdering);
  fullSolver.eliminateInPlace(changed.graph);
  EXPECT(assert_equal(changedExpected, fullSolver.updateSolution(), 1e-9));

  MultifrontalSolver partialSolver(problem.graph, problem.fullOrdering,
                                   problem.pointOrdering.size(), params);
  partialSolver.eliminatePartialInPlace(problem.graph);
  const Matrix actual = partialSolver.remainingFactorGraph().augmentedHessian(
      problem.cameraOrdering);
  const Matrix partialExpected =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering)
          .second->augmentedHessian(problem.cameraOrdering);
  EXPECT(assert_equal(partialExpected, actual, 1e-9));
}

}  // namespace algebraic_leaf_merge_fixture
/* ************************************************************************* */

namespace compact_leaf_fixture {

constexpr size_t kCameraCount = 3;

struct Problem {
  GaussianFactorGraph graph;
  Ordering pointOrdering;
  Ordering cameraOrdering;
  Ordering fullOrdering;
};

Problem createProblem(bool addFallbackFactor = false,
                      bool addSeparatorFallback = false,
                      bool addSecondBatchFactor = false) {
  Problem problem;
  const Key point = L(0);
  problem.pointOrdering.push_back(point);
  problem.fullOrdering.push_back(point);

  KeyVector batchKeys{point};
  for (size_t i = 0; i < kCameraCount; ++i) {
    const Key camera = X(i);
    batchKeys.push_back(camera);
    problem.cameraOrdering.push_back(camera);
    problem.fullOrdering.push_back(camera);
  }

  // The point sees two cameras. A third camera forms a larger parent clique,
  // keeping the point conditional as a separate Bayes-tree leaf.
  batchKeys.resize(3);
  using PointCameraBatch = BatchJacobianFactor<1, 1, 1, 1>;
  auto observations = std::make_shared<PointCameraBatch>(
      batchKeys, std::vector<size_t>{1, 1, 1});
  observations->addRow({0, 1, 2}, {I_1x1, I_1x1, -I_1x1}, Vector1(0.25));
  problem.graph.push_back(observations);

  if (addSecondBatchFactor) {
    auto secondObservations = std::make_shared<PointCameraBatch>(
        batchKeys, std::vector<size_t>{1, 1, 1});
    secondObservations->addRow({0, 1, 2}, {2.0 * I_1x1, -I_1x1, I_1x1},
                               Vector1(-0.5));
    problem.graph.push_back(secondObservations);
  }

  if (addFallbackFactor) {
    problem.graph.emplace_shared<JacobianFactor>(point, I_1x1, Vector1(0.125),
                                                 noiseModel::Unit::Create(1));
  }

  if (addSeparatorFallback) {
    problem.graph.emplace_shared<JacobianFactor>(
        point, I_1x1, problem.cameraOrdering.front(), -I_1x1, Vector1(0.375),
        noiseModel::Unit::Create(1));
  }

  const auto unit = noiseModel::Unit::Create(1);
  for (size_t i = 0; i < kCameraCount; ++i) {
    problem.graph.emplace_shared<JacobianFactor>(problem.cameraOrdering[i],
                                                 I_1x1, Vector1(0.0), unit);
  }
  for (size_t i = 0; i < kCameraCount; ++i) {
    for (size_t j = i + 1; j < kCameraCount; ++j) {
      problem.graph.emplace_shared<JacobianFactor>(
          problem.cameraOrdering[i], I_1x1, problem.cameraOrdering[j], -I_1x1,
          Vector1(0.0), unit);
    }
  }
  return problem;
}

Problem createSameSeparatorProblem() {
  Problem problem;
  const KeyVector points{L(0), L(1)};
  const KeyVector cameras{X(0), X(1), X(2)};
  problem.pointOrdering = Ordering(points.begin(), points.end());
  problem.cameraOrdering = Ordering(cameras.begin(), cameras.end());
  problem.fullOrdering = problem.pointOrdering;
  problem.fullOrdering.insert(problem.fullOrdering.end(), cameras.begin(),
                              cameras.end());

  using PointCameraBatch = BatchJacobianFactor<1, 1, 1, 1>;
  for (size_t index = 0; index < points.size(); ++index) {
    const KeyVector keys{points[index], cameras[0], cameras[1]};
    auto observations =
        std::make_shared<PointCameraBatch>(keys, std::vector<size_t>{1, 1, 1});
    observations->addRow({0, 1, 2}, {I_1x1, I_1x1, -I_1x1},
                         Vector1(0.25 + static_cast<double>(index)));
    problem.graph.push_back(observations);
  }

  const auto unit = noiseModel::Unit::Create(1);
  for (const Key camera : cameras) {
    problem.graph.emplace_shared<JacobianFactor>(camera, I_1x1, Vector1::Zero(),
                                                 unit);
  }
  for (size_t first = 0; first < cameras.size(); ++first) {
    for (size_t second = first + 1; second < cameras.size(); ++second) {
      problem.graph.emplace_shared<JacobianFactor>(cameras[first], I_1x1,
                                                   cameras[second], -I_1x1,
                                                   Vector1::Zero(), unit);
    }
  }
  return problem;
}

Problem createSingleJacobianProblem() {
  Problem problem = createProblem();
  problem.graph[0] = std::make_shared<JacobianFactor>(
      L(0), I_1x1, X(0), I_1x1, X(1), -I_1x1, Vector1(0.25),
      noiseModel::Unit::Create(1));
  return problem;
}

Problem createIneligibleBatchProblem() {
  Problem problem = createProblem();
  problem.graph.emplace_shared<JacobianFactor>(
      X(0), I_1x1, Vector1::Zero(), noiseModel::Constrained::All(1));
  return problem;
}

MultifrontalSolver::Parameters compactParams() {
  auto params = noMergeParams();
  params.qrMode = MultifrontalParameters::QRMode::Off;
  params.compactCholeskySeparatorDimThreshold = 2;
  return params;
}

MultifrontalSolver::Parameters fusedStarParams() {
  auto params = noMergeParams();
  params.qrMode = MultifrontalParameters::QRMode::Off;
  return params;
}

size_t compactCliqueCount(MultifrontalSolver* solver) {
  size_t count = 0;
  solver->runTopDown([&](MultifrontalClique& clique) {
    if (clique.useCompactCholesky()) ++count;
  });
  return count;
}

MultifrontalClique* compactClique(MultifrontalSolver* solver) {
  MultifrontalClique* result = nullptr;
  solver->runTopDown([&](MultifrontalClique& clique) {
    if (clique.useCompactCholesky()) result = &clique;
  });
  return result;
}

MultifrontalClique* pointLeaf(MultifrontalSolver* solver) {
  MultifrontalClique* result = nullptr;
  solver->runTopDown([&](MultifrontalClique& clique) {
    if (clique.children.empty() && clique.separatorDim > 0) result = &clique;
  });
  return result;
}

}  // namespace compact_leaf_fixture

// A large-separator batch leaf forms the same reduced factor without
// materializing C, including through the fused load/eliminate overload.
TEST(MultifrontalSolver, CompactLeafPartialElimination) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem();
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            problem.pointOrdering.size(), compactParams());
  EXPECT_LONGS_EQUAL(1, compactCliqueCount(&solver));

  solver.eliminatePartialInPlace(problem.graph);
  EXPECT_LONGS_EQUAL(0, compactClique(&solver)->Ab().rows());
  const GaussianFactorGraph actual = solver.remainingFactorGraph();
  const auto [unusedBayesTree, expected] =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering);
  (void)unusedBayesTree;
  EXPECT(assert_equal(expected->augmentedHessian(problem.cameraOrdering),
                      actual.augmentedHessian(problem.cameraOrdering), 1e-9));
}

// The same compact leaf path feeds an ordinary parent clique in a complete
// multifrontal solve.
TEST(MultifrontalSolver, CompactLeafFullElimination) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem();
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            compactParams());
  EXPECT_LONGS_EQUAL(1, compactCliqueCount(&solver));

  solver.eliminateInPlace(problem.graph);
  const VectorValues& actual = solver.updateSolution();
  const VectorValues expected =
      problem.graph.eliminateMultifrontal(problem.fullOrdering)->optimize();
  EXPECT(assert_equal(expected, actual, 1e-8));
}

// A compact leaf packs only the Jacobian fallback row and reuses that storage
// across loads while preserving the retained Hessian.
TEST(MultifrontalSolver, CompactLeafMixedFallbackStorageReuse) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem(true);
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            problem.pointOrdering.size(), compactParams());
  MultifrontalClique* clique = compactClique(&solver);
  EXPECT(clique != nullptr);

  solver.load(problem.graph);
  EXPECT_LONGS_EQUAL(1, clique->Ab().rows());
  const double* storage = clique->Ab().matrix().data();
  solver.eliminatePartialInPlace();
  const Matrix expected =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering)
          .second->augmentedHessian(problem.cameraOrdering);
  EXPECT(assert_equal(
      expected,
      solver.remainingFactorGraph().augmentedHessian(problem.cameraOrdering),
      1e-9));

  solver.eliminatePartialInPlace(problem.graph);
  EXPECT(storage == clique->Ab().matrix().data());
  EXPECT_LONGS_EQUAL(1, clique->Ab().rows());
  EXPECT(assert_equal(
      expected,
      solver.remainingFactorGraph().augmentedHessian(problem.cameraOrdering),
      1e-9));
}

// An eligible one-frontal-block star bypasses the default compact separator
// threshold while retaining only its unary fallback rows.
TEST(MultifrontalSolver, FusedStarBypassesCompactThreshold) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem(true);
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            problem.pointOrdering.size(), fusedStarParams());
  solver.load(problem.graph);
  MultifrontalClique* clique = compactClique(&solver);
  EXPECT(clique != nullptr);
  EXPECT_LONGS_EQUAL(0, clique->info().nBlocks());
  EXPECT_LONGS_EQUAL(1, clique->Ab().rows());

  solver.eliminatePartialInPlace();
  const Matrix expected =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering)
          .second->augmentedHessian(problem.cameraOrdering);
  EXPECT(assert_equal(
      expected,
      solver.remainingFactorGraph().augmentedHessian(problem.cameraOrdering),
      1e-9));
}

// Automatic QR defers a sole direct batch observation to the fused star path
// and preserves that choice and its row-free storage across repeated loads.
TEST(MultifrontalSolver, AutoQrSingleBatchUsesFusedStar) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem();
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            noMergeParams());
  MultifrontalClique* leaf = pointLeaf(&solver);
  EXPECT(leaf != nullptr);
  EXPECT(leaf->useQR());

  solver.load(problem.graph);
  EXPECT(!leaf->useQR());
  EXPECT(leaf->useCompactCholesky());
  EXPECT_LONGS_EQUAL(0, leaf->Ab().rows());
  EXPECT_LONGS_EQUAL(0, leaf->info().nBlocks());
  solver.eliminateInPlace();
  const VectorValues expected =
      problem.graph.eliminateMultifrontal(problem.fullOrdering)->optimize();
  EXPECT(assert_equal(expected, solver.updateSolution(), 1e-9));

  solver.eliminateInPlace(problem.graph);
  EXPECT(!leaf->useQR());
  EXPECT_LONGS_EQUAL(0, leaf->Ab().rows());
  EXPECT(assert_equal(expected, solver.updateSolution(), 1e-9));
}

// A sole ordinary Jacobian keeps the existing automatic QR selection.
TEST(MultifrontalSolver, AutoQrSingleJacobianRemainsQr) {
  using namespace compact_leaf_fixture;
  const Problem problem = createSingleJacobianProblem();
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            noMergeParams());
  MultifrontalClique* leaf = pointLeaf(&solver);
  EXPECT(leaf != nullptr);
  solver.load(problem.graph);
  EXPECT(leaf->useQR());
  solver.eliminateInPlace();
  const VectorValues expected =
      problem.graph.eliminateMultifrontal(problem.fullOrdering, EliminateQR)
          ->optimize();
  EXPECT(assert_equal(expected, solver.updateSolution(), 1e-9));
}

// A sole batch factor without direct-update support remains on automatic QR.
TEST(MultifrontalSolver, AutoQrIneligibleBatchRemainsQr) {
  using namespace compact_leaf_fixture;
  const Problem problem = createIneligibleBatchProblem();
  auto params = noMergeParams();
  params.qrAspectRatio = 1.0;
  MultifrontalSolver solver(problem.graph, problem.fullOrdering, params);
  MultifrontalClique* leaf = pointLeaf(&solver);
  EXPECT(leaf != nullptr);
  solver.load(problem.graph);
  EXPECT(leaf->useQR());
}

// Multiple batch factors retain the existing automatic QR behavior.
TEST(MultifrontalSolver, AutoQrMultipleBatchFactorsRemainQr) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem(false, false, true);
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            noMergeParams());
  MultifrontalClique* leaf = pointLeaf(&solver);
  EXPECT(leaf != nullptr);
  solver.load(problem.graph);
  EXPECT(leaf->useQR());
}

// A batch factor mixed with a Jacobian retains automatic QR behavior.
TEST(MultifrontalSolver, AutoQrMixedFactorsRemainQr) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem(true);
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            noMergeParams());
  MultifrontalClique* leaf = pointLeaf(&solver);
  EXPECT(leaf != nullptr);
  solver.load(problem.graph);
  EXPECT(leaf->useQR());
}

// Explicit Force overrides the sole direct batch-factor exception.
TEST(MultifrontalSolver, ForcedQrSingleBatchRemainsQr) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem();
  auto params = noMergeParams();
  params.qrMode = MultifrontalParameters::QRMode::Force;
  MultifrontalSolver solver(problem.graph, problem.fullOrdering, params);
  MultifrontalClique* leaf = pointLeaf(&solver);
  EXPECT(leaf != nullptr);
  solver.load(problem.graph);
  EXPECT(leaf->useQR());
}

// A separator-bearing conventional Jacobian makes a small star candidate use
// ordinary Cholesky, preserving the generic fallback behavior.
TEST(MultifrontalSolver, FusedStarRejectsSeparatorJacobian) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem(false, true);
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            problem.pointOrdering.size(), fusedStarParams());
  solver.load(problem.graph);
  EXPECT_LONGS_EQUAL(0, compactCliqueCount(&solver));

  solver.eliminatePartialInPlace();
  const Matrix expected =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering)
          .second->augmentedHessian(problem.cameraOrdering);
  EXPECT(assert_equal(
      expected,
      solver.remainingFactorGraph().augmentedHessian(problem.cameraOrdering),
      1e-9));
}

// Multiple direct batch factors sharing retained blocks are accumulated before
// one mapped Schur subtraction.
TEST(MultifrontalSolver, FusedStarMultipleBatchFactors) {
  using namespace compact_leaf_fixture;
  const Problem problem = createProblem(false, false, true);
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            problem.pointOrdering.size(), fusedStarParams());
  solver.eliminatePartialInPlace(problem.graph);
  EXPECT_LONGS_EQUAL(1, compactCliqueCount(&solver));
  const Matrix expected =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering)
          .second->augmentedHessian(problem.cameraOrdering);
  EXPECT(assert_equal(
      expected,
      solver.remainingFactorGraph().augmentedHessian(problem.cameraOrdering),
      1e-9));
}

// Same-separator leaf aggregation accepts fused children and preserves their
// retained camera system.
TEST(MultifrontalSolver, FusedStarSameSeparatorAggregation) {
  using namespace compact_leaf_fixture;
  const Problem problem = createSameSeparatorProblem();
  auto params = fusedStarParams();
  params.leafMode = MultifrontalParameters::LeafMode::SameSeparator;
  params.leafAggregationProblemSize = 2048;
  MultifrontalSolver solver(problem.graph, problem.fullOrdering,
                            problem.pointOrdering.size(), params);
  solver.eliminatePartialInPlace(problem.graph);
  EXPECT_LONGS_EQUAL(2, compactCliqueCount(&solver));
  const Matrix expected =
      problem.graph.eliminatePartialMultifrontal(problem.pointOrdering)
          .second->augmentedHessian(problem.cameraOrdering);
  EXPECT(assert_equal(
      expected,
      solver.remainingFactorGraph().augmentedHessian(problem.cameraOrdering),
      1e-9));
}

// The default algebraic merge cap leaves single-factor siblings separate so
// automatic QR can still resolve each direct batch leaf to fused Cholesky.
TEST(MultifrontalSolver, AlgebraicMergePreservesSingleBatchLeaves) {
  using namespace compact_leaf_fixture;
  const Problem problem = createSameSeparatorProblem();
  auto params = noMergeParams();
  params.leafMergeDimCap = 256;
  MultifrontalSolver solver(problem.graph, problem.fullOrdering, params);
  solver.load(problem.graph);

  size_t fusedPointLeaves = 0;
  solver.runTopDown([&](MultifrontalClique& clique) {
    if (clique.children.empty() && clique.separatorDim > 0) {
      ++fusedPointLeaves;
      EXPECT_LONGS_EQUAL(1, clique.numFrontals());
      EXPECT(clique.useCompactCholesky());
      EXPECT_LONGS_EQUAL(0, clique.Ab().rows());
    }
  });
  EXPECT_LONGS_EQUAL(2, fusedPointLeaves);

  solver.eliminateInPlace();
  const VectorValues expected =
      problem.graph.eliminateMultifrontal(problem.fullOrdering)->optimize();
  EXPECT(assert_equal(expected, solver.updateSolution(), 1e-9));
}

/* ************************************************************************* */
// Build the solver and validate initial structure and explicit load.
TEST(MultifrontalSolver, Constructor) {
  MultifrontalSolver solver(chain, chainOrdering, noMergeParams());
  solver.load(chain);

  // Verify roots
  EXPECT(solver.roots().size() == 1);
  auto root = solver.roots()[0];
  EXPECT(root != nullptr);

  // Root should have 1 child {x2, x1}
  EXPECT_LONGS_EQUAL(1, root->children.size());
  auto childClique = root->children[0];

  // Verify matrices in leaf (childClique)
  EXPECT_LONGS_EQUAL(4, childClique->info().nBlocks());
  EXPECT_LONGS_EQUAL(2, childClique->Ab().rows());
  EXPECT_LONGS_EQUAL(4, childClique->Ab().nBlocks());

  // Verify initial load for childClique
  // Block 0 (x2):
  Matrix A0 = childClique->Ab()(0);  // 2x1
  EXPECT(assert_equal(Matrix{{2.}, {1.}}, A0));

  // Block 3 (RHS):
  Matrix Ab = childClique->Ab()(3);  // 2x1
  EXPECT(assert_equal(Matrix{{2.}, {1.}}, Ab));
}

/* ************************************************************************* */
// Build the solver from precomputed data and validate structure and load.
TEST(MultifrontalSolver, ConstructorPrecomputed) {
  auto data = MultifrontalSolver::Precompute(chain, chainOrdering);
  MultifrontalSolver solver(std::move(data), chainOrdering, noMergeParams());
  solver.load(chain);

  // Verify roots
  EXPECT(solver.roots().size() == 1);
  auto root = solver.roots()[0];
  EXPECT(root != nullptr);

  // Root should have 1 child {x2, x1}
  EXPECT_LONGS_EQUAL(1, root->children.size());
  auto childClique = root->children[0];

  // Verify matrices in leaf (childClique)
  CHECK(childClique->useQR() == false);
  EXPECT_LONGS_EQUAL(4, childClique->info().nBlocks());
  EXPECT_LONGS_EQUAL(2, childClique->Ab().rows());
  EXPECT_LONGS_EQUAL(4, childClique->Ab().nBlocks());

  // Verify load for childClique
  Matrix A0 = childClique->Ab()(0);
  EXPECT(assert_equal(Matrix{{2.}, {1.}}, A0));
}

/* ************************************************************************* */
// Reload numerical values and ensure Ab updates match whitening.
TEST(MultifrontalSolver, Load) {
  MultifrontalSolver solver(chain, chainOrdering, noMergeParams());

  // Create a new graph with doubled values
  GaussianFactorGraph chain2;
  for (const auto& factor : chain) {
    auto jacobianFactor = std::dynamic_pointer_cast<JacobianFactor>(factor);
    std::map<Key, Matrix> terms;
    for (auto it = jacobianFactor->begin(); it != jacobianFactor->end(); ++it) {
      terms[*it] = jacobianFactor->getA(it) * 2.0;
    }
    chain2.push_back(std::make_shared<JacobianFactor>(
        terms, jacobianFactor->getb() * 2.0, jacobianFactor->get_model()));
  }

  solver.load(chain2);

  // Verify values in childClique
  auto root = solver.roots()[0];
  auto childClique = root->children[0];

  // Block 0 (x2) should now be doubled, then whitened.
  Matrix A0 = childClique->Ab()(0);
  EXPECT(assert_equal(Matrix{{4.}, {2}}, A0));
}

/* ************************************************************************* */
// Compare solver output against multifrontal elimination baseline.
TEST(MultifrontalSolver, Eliminate) {
  MultifrontalSolver solver(chain, chainOrdering, noMergeParams());
  solver.load(chain);
  solver.eliminateInPlace();

  // Solve
  const VectorValues& actual = solver.updateSolution();

  // Reference elimination and solve
  GaussianBayesTree expectedBT = *chain.eliminateMultifrontal(chainOrdering);
  VectorValues expected = expectedBT.optimize();

  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// deltaError from the solver matches GaussianFactorGraph for the
// solver-produced (optimal) delta.
TEST(MultifrontalSolver, DeltaErrorMatchesGraph) {
  MultifrontalSolver solver(chain, chainOrdering, noMergeParams());
  solver.eliminateInPlace(chain);

  const VectorValues& delta = solver.updateSolution();

  double oldFast = 0.0;
  double newFast = 0.0;
  double deltaFast = solver.deltaError(&oldFast, &newFast);

  double oldRef = 0.0;
  double newRef = 0.0;
  double deltaRef = chain.deltaError(delta, &oldRef, &newRef);

  DOUBLES_EQUAL(oldRef, oldFast, 1e-9);
  DOUBLES_EQUAL(newRef, newFast, 1e-9);
  DOUBLES_EQUAL(deltaRef, deltaFast, 1e-9);
}

/* ************************************************************************* */
// deltaError from the solver matches GaussianFactorGraph on an
// overdetermined system with nonzero residual at the solution.
TEST(MultifrontalSolver, DeltaErrorMatchesGraphInconsistent) {
  const SharedDiagonal noise = noiseModel::Isotropic::Sigma(1, 1.0);
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{1.0}}, noise);
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{-2.0}}, noise);
  const Ordering ordering{x1};
  MultifrontalSolver solver(graph, ordering, noMergeParams());
  solver.eliminateInPlace(graph);

  const VectorValues& delta = solver.updateSolution();

  double oldFast = 0.0;
  double newFast = 0.0;
  double deltaFast = solver.deltaError(&oldFast, &newFast);

  double oldRef = 0.0;
  double newRef = 0.0;
  double deltaRef = graph.deltaError(delta, &oldRef, &newRef);

  DOUBLES_EQUAL(oldRef, oldFast, 1e-9);
  DOUBLES_EQUAL(newRef, newFast, 1e-9);
  DOUBLES_EQUAL(deltaRef, deltaFast, 1e-9);
}

/* ************************************************************************* */
// Load + eliminate in one traversal matches standard elimination.
TEST(MultifrontalSolver, EliminateWithLoad) {
  MultifrontalSolver solver(chain, chainOrdering, noMergeParams());
  solver.eliminateInPlace(chain);

  const VectorValues& actual = solver.updateSolution();

  GaussianBayesTree expectedBT = *chain.eliminateMultifrontal(chainOrdering);
  VectorValues expected = expectedBT.optimize();

  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// deltaError match when QR is forced, exercising the QR leaf RSd_ path.
TEST(MultifrontalSolver, DeltaErrorMatchesGraphQR) {
  auto qrParams = noMergeParams();
  qrParams.qrMode = MultifrontalParameters::QRMode::Force;
  MultifrontalSolver solver(chain, chainOrdering, qrParams);
  solver.eliminateInPlace(chain);

  const VectorValues& delta = solver.updateSolution();

  double oldFast = 0.0;
  double newFast = 0.0;
  double deltaFast = solver.deltaError(&oldFast, &newFast);

  double oldRef = 0.0;
  double newRef = 0.0;
  double deltaRef = chain.deltaError(delta, &oldRef, &newRef);

  DOUBLES_EQUAL(oldRef, oldFast, 1e-9);
  DOUBLES_EQUAL(newRef, newFast, 1e-9);
  DOUBLES_EQUAL(deltaRef, deltaFast, 1e-9);
}

/* ************************************************************************* */
// Forcing QR enables QR on all leaves and matches legacy QR elimination.
TEST(MultifrontalSolver, ForceQRMatchesDenseQR) {
  auto qrParams = noMergeParams();
  qrParams.qrMode = MultifrontalParameters::QRMode::Force;
  MultifrontalSolver solverQR(chain, chainOrdering, qrParams);
  solverQR.eliminateInPlace(chain);

  size_t leafCount = 0;
  size_t qrLeafCount = 0;
  solverQR.runTopDown([&](MultifrontalClique& node) {
    if (node.children.empty()) {
      ++leafCount;
      if (node.useQR()) {
        ++qrLeafCount;
      }
    }
  });
  CHECK(leafCount > 0);
  CHECK(leafCount == qrLeafCount);

  const VectorValues& actual = solverQR.updateSolution();

  VectorValues expected = chain.optimize(chainOrdering, EliminateQR);

  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// Compare marginals from in-place Bayes tree against standard elimination.
TEST(MultifrontalSolver, ComputeBayesTreeMarginals) {
  MultifrontalSolver solver(chain, chainOrdering, noMergeParams());
  solver.load(chain);
  solver.eliminateInPlace();

  GaussianBayesTree actualBT = solver.computeBayesTree();
  GaussianBayesTree expectedBT = *chain.eliminateMultifrontal(chainOrdering);

  EXPECT(assert_equal(expectedBT.marginalCovariance(x2),
                      actualBT.marginalCovariance(x2), 1e-9));
  EXPECT(assert_equal(expectedBT.marginalCovariance(x3),
                      actualBT.marginalCovariance(x3), 1e-9));

  Marginals actualMarginals(std::move(actualBT), solver.updateSolution());
  Marginals expectedMarginals(chain, solver.updateSolution());

  EXPECT(assert_equal(expectedMarginals.marginalCovariance(x2),
                      actualMarginals.marginalCovariance(x2), 1e-9));

  const KeyVector jointKeys{x2, x3};
  const JointMarginal expectedJoint =
      expectedMarginals.jointMarginalCovariance(jointKeys);
  const JointMarginal actualJoint =
      actualMarginals.jointMarginalCovariance(jointKeys);
  EXPECT(
      assert_equal(expectedJoint.fullMatrix(), actualJoint.fullMatrix(), 1e-9));
}

/* ************************************************************************* */
// Compare marginals on a constrained chain against legacy marginals.
TEST(MultifrontalSolver, ComputeBayesTreeMarginalsConstrainedChain) {
  const SharedDiagonal hardConstraint =
      noiseModel::Constrained::MixedSigmas(Vector{{0.0}});
  GaussianFactorGraph constrainedChain = chain;
  constrainedChain.emplace_shared<JacobianFactor>(x2, I_1x1, Vector{{0.0}},
                                                  hardConstraint);

  MultifrontalSolver solver(constrainedChain, chainOrdering, noMergeParams());
  solver.load(constrainedChain);
  solver.eliminateInPlace();

  GaussianBayesTree actualBT = solver.computeBayesTree();
  Marginals actualMarginals(std::move(actualBT), solver.updateSolution());
  Marginals expectedMarginals(constrainedChain, solver.updateSolution());

  const KeyVector keys{x1, x2, x3, x4};
  for (Key key : keys) {
    EXPECT(assert_equal(expectedMarginals.marginalCovariance(key),
                        actualMarginals.marginalCovariance(key), 1e-9));
  }
}

/* ************************************************************************* */
// Verify feasible unary constrained factor clamps the update to zero.
TEST(MultifrontalSolver, ConstrainedNoiseFeasible) {
  const SharedDiagonal hardConstraint =
      noiseModel::Constrained::MixedSigmas(Vector{{0.0}});
  const SharedDiagonal softNoise = noiseModel::Isotropic::Sigma(1, 10.0);
  GaussianFactorGraph graph;
  // Same setup as ConstrainedNoiseUnsupported, but feasible (b == 0).
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{0.0}},
                                       hardConstraint);
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{100.0}}, softNoise);
  const Ordering ordering{x1};

  MultifrontalSolver solver(graph, ordering, noMergeParams());
  solver.load(graph);
  solver.eliminateInPlace();
  const VectorValues& actual = solver.updateSolution();

  EXPECT_DOUBLES_EQUAL(0.0, actual.at(x1)(0), 1e-9);
}

/* ************************************************************************* */
// Infeasible unary constrained factor is rejected.
TEST(MultifrontalSolver, ConstrainedNoiseUnsupported) {
  const SharedDiagonal hardConstraint =
      noiseModel::Constrained::MixedSigmas(Vector{{0.0}});
  const SharedDiagonal softNoise = noiseModel::Isotropic::Sigma(1, 10.0);
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{1.0}},
                                       hardConstraint);
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{100.0}}, softNoise);
  const Ordering ordering{x1};

  CHECK_EXCEPTION(
      { MultifrontalSolver solver(graph, ordering, noMergeParams()); },
      std::runtime_error);
}

/* ************************************************************************* */
// Fully constrained unary factor with All() keeps delta fixed at zero.
TEST(MultifrontalSolver, ConstrainedNoiseUnaryFeasible) {
  const SharedDiagonal hardConstraint = noiseModel::Constrained::All(1);
  const SharedDiagonal softNoise = noiseModel::Isotropic::Sigma(1, 1.0);
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{0.0}},
                                       hardConstraint);
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{5.0}}, softNoise);
  const Ordering ordering{x1};

  MultifrontalSolver solver(graph, ordering, noMergeParams());
  solver.load(graph);
  solver.eliminateInPlace();
  const VectorValues& actual = solver.updateSolution();

  EXPECT_DOUBLES_EQUAL(0.0, actual.at(x1)(0), 1e-9);
}

/* ************************************************************************* */
// Mixed-key constrained factor is not supported.
TEST(MultifrontalSolver, ConstrainedNoiseMixedKeysUnsupported) {
  const SharedDiagonal hardConstraint = noiseModel::Constrained::All(1);
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, x2, I_1x1, Vector{{0.0}},
                                       hardConstraint);
  const Ordering ordering{x1, x2};

  CHECK_EXCEPTION(
      { MultifrontalSolver solver(graph, ordering, noMergeParams()); },
      std::runtime_error);
}

/* ************************************************************************* */
// Weighted scalar measurements produce the expected weighted estimate.
TEST(MultifrontalSolver, WeightedScalarMeasurements) {
  const double w1 = 0.2;
  const double w2 = 0.8;
  const double sigma1 = std::sqrt(1.0 / w1);
  const double sigma2 = std::sqrt(1.0 / w2);

  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{0.0}},
                                       noiseModel::Isotropic::Sigma(1, sigma1));
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{10.0}},
                                       noiseModel::Isotropic::Sigma(1, sigma2));

  const Ordering ordering{x1};
  MultifrontalSolver solver(graph, ordering, noMergeParams());
  solver.load(graph);
  solver.eliminateInPlace();
  const VectorValues& actual = solver.updateSolution();

  EXPECT_DOUBLES_EQUAL(8.0, actual.at(x1)(0), 1e-9);
}

/* ************************************************************************* */
// Compact batch Jacobians solve the same system as equivalent dense factors.
TEST(MultifrontalSolver, BatchJacobianFactor) {
  auto batch = std::make_shared<BatchJacobianFactor<1, 1, 1>>(
      KeyVector{x1, x2}, std::vector<size_t>{1, 1});
  batch->reserve(2);
  std::vector<Matrix> blocks{I_1x1, -I_1x1};
  batch->addRow({0, 1}, blocks, Vector{{1.0}});
  blocks = {2.0 * I_1x1, I_1x1};
  batch->addRow({0, 1}, blocks, Vector{{3.0}});

  GaussianFactorGraph batchGraph;
  batchGraph.push_back(batch);

  GaussianFactorGraph denseGraph;
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, x2, -I_1x1,
                                            Vector{{1.0}});
  denseGraph.emplace_shared<JacobianFactor>(x1, 2.0 * I_1x1, x2, I_1x1,
                                            Vector{{3.0}});

  const Ordering ordering{x1, x2};
  MultifrontalSolver solver(batchGraph, ordering, noMergeParams());
  solver.eliminateInPlace(batchGraph);
  EXPECT(!solver.roots().front()->useQR());
  EXPECT_LONGS_EQUAL(0, solver.roots().front()->Ab().rows());
  const VectorValues& actual = solver.updateSolution();

  const VectorValues expected = denseGraph.optimize(ordering);
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// A mixed Cholesky clique stores only its dense fallback row; direct batch
// rows update the Hessian without occupying Ab.
TEST(MultifrontalSolver, MixedBatchJacobianFallbackStorage) {
  auto batch = std::make_shared<BatchJacobianFactor<1, 1, 1>>(
      KeyVector{x1, x2}, std::vector<size_t>{1, 1});
  batch->addRow({0, 1}, {I_1x1, -I_1x1}, Vector1(1.0));
  batch->addRow({0, 1}, {2.0 * I_1x1, I_1x1}, Vector1(3.0));

  GaussianFactorGraph graph{batch};
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector1(0.5));
  GaussianFactorGraph denseGraph;
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, x2, -I_1x1,
                                            Vector1(1.0));
  denseGraph.emplace_shared<JacobianFactor>(x1, 2.0 * I_1x1, x2, I_1x1,
                                            Vector1(3.0));
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector1(0.5));

  const Ordering ordering{x1, x2};
  MultifrontalSolver solver(graph, ordering, noMergeParams());
  solver.eliminateInPlace(graph);
  EXPECT(!solver.roots().front()->useQR());
  EXPECT_LONGS_EQUAL(1, solver.roots().front()->Ab().rows());
  EXPECT(assert_equal(denseGraph.optimize(ordering), solver.updateSolution(),
                      1e-9));
}

/* ************************************************************************* */
// Forced QR materializes every batch row and reserves one damping row per
// frontal dimension.
TEST(MultifrontalSolver, BatchJacobianFactorForcedQRStorage) {
  auto batch = std::make_shared<BatchJacobianFactor<1, 1, 1>>(
      KeyVector{x1, x2}, std::vector<size_t>{1, 1});
  batch->addRow({0, 1}, {I_1x1, -I_1x1}, Vector1(1.0));
  batch->addRow({0, 1}, {2.0 * I_1x1, I_1x1}, Vector1(3.0));
  const GaussianFactorGraph graph{batch};

  auto params = noMergeParams();
  params.qrMode = MultifrontalParameters::QRMode::Force;
  const Ordering ordering{x1, x2};
  MultifrontalSolver solver(graph, ordering, params);
  solver.load(graph);
  EXPECT(solver.roots().front()->useQR());
  EXPECT_LONGS_EQUAL(4, solver.roots().front()->Ab().rows());
  solver.eliminateInPlace();

  GaussianFactorGraph denseGraph;
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, x2, -I_1x1,
                                            Vector1(1.0));
  denseGraph.emplace_shared<JacobianFactor>(x1, 2.0 * I_1x1, x2, I_1x1,
                                            Vector1(3.0));
  EXPECT(assert_equal(denseGraph.optimize(ordering, EliminateQR),
                      solver.updateSolution(), 1e-9));
}

/* ************************************************************************* */
// Compact batch Jacobians can be densified for legacy QR elimination.
TEST(MultifrontalSolver, BatchJacobianFactorLegacyQR) {
  auto batch = std::make_shared<BatchJacobianFactor<1, 1, 1>>(
      KeyVector{x1, x2}, std::vector<size_t>{1, 1});
  batch->reserve(2);
  std::vector<Matrix> blocks{I_1x1, -I_1x1};
  batch->addRow({0, 1}, blocks, Vector{{1.0}});
  blocks = {2.0 * I_1x1, I_1x1};
  batch->addRow({0, 1}, blocks, Vector{{3.0}});

  GaussianFactorGraph batchGraph;
  batchGraph.push_back(batch);

  GaussianFactorGraph denseGraph;
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, x2, -I_1x1,
                                            Vector{{1.0}});
  denseGraph.emplace_shared<JacobianFactor>(x1, 2.0 * I_1x1, x2, I_1x1,
                                            Vector{{3.0}});

  const Ordering ordering{x1, x2};
  const VectorValues actual = batchGraph.optimize(ordering, EliminateQR);
  const VectorValues expected = denseGraph.optimize(ordering, EliminateQR);

  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// A batch factor with non-unit diagonal weights is equivalent to weighted dense
// factors.
TEST(MultifrontalSolver, BatchJacobianFactorWeightedModel) {
  auto nonUnitModel = noiseModel::Diagonal::Sigmas(Vector{{2.0, 0.5}});
  auto batch = std::make_shared<BatchJacobianFactor<1, 1, 1>>(
      KeyVector{x1, x2}, std::vector<size_t>{1, 1}, nonUnitModel);
  batch->reserve(2);
  std::vector<Matrix> blocks{I_1x1, -I_1x1};
  batch->addRow({0, 1}, blocks, Vector{{1.0}});
  blocks = {2.0 * I_1x1, I_1x1};
  batch->addRow({0, 1}, blocks, Vector{{3.0}});

  GaussianFactorGraph batchGraph;
  batchGraph.push_back(batch);

  GaussianFactorGraph denseGraph;
  denseGraph.emplace_shared<JacobianFactor>(x1, Matrix{{0.5}}, x2,
                                            Matrix{{-0.5}}, Vector{{0.5}});
  denseGraph.emplace_shared<JacobianFactor>(x1, Matrix{{4.0}}, x2,
                                            Matrix{{2.0}}, Vector{{6.0}});

  const Ordering ordering{x1, x2};
  const VectorValues actual = batchGraph.optimize(ordering);
  const VectorValues expected = denseGraph.optimize(ordering);
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// A mixed stack of unit and non-unit batch models with a unary constrained
// factor.
TEST(MultifrontalSolver, BatchJacobianFactorMixedNoiseModels) {
  auto unitModel = noiseModel::Unit::Create(1);
  auto batchUnit = std::make_shared<BatchJacobianFactor<1, 1>>(
      KeyVector{x1}, std::vector<size_t>{1}, unitModel);
  batchUnit->reserve(1);
  batchUnit->addRow({0}, {I_1x1}, Vector{{1.0}});

  auto nonUnitModel = noiseModel::Diagonal::Sigmas(Vector{{2.0}});
  auto batchNonUnit = std::make_shared<BatchJacobianFactor<1, 1>>(
      KeyVector{x1}, std::vector<size_t>{1}, nonUnitModel);
  batchNonUnit->reserve(1);
  batchNonUnit->addRow({0}, {I_1x1}, Vector{{2.0}});

  auto constrainedModel = noiseModel::Constrained::All(1);

  GaussianFactorGraph batchGraph;
  batchGraph.push_back(batchUnit);
  batchGraph.push_back(batchNonUnit);
  batchGraph.emplace_shared<JacobianFactor>(x2, I_1x1, Vector{{0.0}},
                                            constrainedModel);

  GaussianFactorGraph denseGraph;
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{1.0}},
                                            unitModel);
  denseGraph.emplace_shared<JacobianFactor>(x1, I_1x1, Vector{{2.0}},
                                            nonUnitModel);
  denseGraph.emplace_shared<JacobianFactor>(x2, I_1x1, Vector{{0.0}},
                                            constrainedModel);

  const Ordering ordering{x1, x2};
  const VectorValues actual = batchGraph.optimize(ordering);
  const VectorValues expected = denseGraph.optimize(ordering);
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
// Hessian factors are rejected by the multifrontal solver.
TEST(MultifrontalSolver, HessianFactors) {
  GaussianFactorGraph graph;
  graph.emplace_shared<HessianFactor>(x1, Matrix{{4.0}}, Vector{{8.0}}, 0.0);

  const Ordering ordering{x1};
  CHECK_EXCEPTION(
      { MultifrontalSolver solver(graph, ordering, noMergeParams()); },
      std::runtime_error);
}

/* ************************************************************************* */
// Loading a HessianFactor into precomputed Jacobian-only structure is rejected.
TEST(MultifrontalSolver, LoadRejectsHessianFactor) {
  GaussianFactorGraph jacobianGraph;
  jacobianGraph.emplace_shared<JacobianFactor>(
      x1, I_1x1, Vector{{2.0}}, noiseModel::Isotropic::Sigma(1, 1.0));

  GaussianFactorGraph hessianGraph;
  hessianGraph.emplace_shared<HessianFactor>(x1, Matrix{{4.0}}, Vector{{8.0}},
                                             0.0);

  const Ordering ordering{x1};
  auto data = MultifrontalSolver::Precompute(jacobianGraph, ordering);
  MultifrontalSolver solver(std::move(data), ordering, noMergeParams());

  CHECK_EXCEPTION(solver.load(hessianGraph), MultifrontalSolverNotSupported);
}

/* ************************************************************************* */
// Merge threshold changes the clique count.
TEST(MultifrontalSolver, MergeDimCap) {
  MultifrontalSolver::Parameters noMerge = noMergeParams();
  MultifrontalSolver solverNoMerge(chain, chainOrdering, noMerge);
  EXPECT_LONGS_EQUAL(2, solverNoMerge.cliqueCount());

  MultifrontalSolver::Parameters merge = noMergeParams();
  merge.mergeDimCap = 1000;
  MultifrontalSolver solverMerge(chain, chainOrdering, merge);
  EXPECT_LONGS_EQUAL(1, solverMerge.cliqueCount());
}

/* ************************************************************************* */
// End-to-end balanced smoother test with reload.
TEST(MultifrontalSolver, BalancedSmoother) {
  // Create smoother with 7 nodes
  auto [nlfg, poses] = example::createNonlinearSmoother(7);
  poses.update(X(1), Point2(1.1, 0.2));
  GaussianFactorGraph smoother = *nlfg.linearize(poses);

  // Create the Bayes tree ordering
  const Ordering ordering{X(1), X(3), X(5), X(7), X(2), X(6), X(4)};

  MultifrontalSolver solver(smoother, ordering, noMergeParams());
  solver.load(smoother);

  // Verify roots
  EXPECT(solver.roots().size() == 1);
  auto root = solver.roots()[0];

  EXPECT_LONGS_EQUAL(root->Ab().nBlocks(), root->info().nBlocks());

  // Check a leaf clique block structure.
  MultifrontalSolver::CliquePtr leaf = nullptr;
  size_t minBlocks = std::numeric_limits<size_t>::max();
  std::function<void(MultifrontalSolver::CliquePtr)> findLeaf =
      [&](MultifrontalSolver::CliquePtr c) {
        if (!c) return;
        if (c->children.empty()) {
          const size_t blocks = c->info().nBlocks();
          if (blocks < minBlocks) {
            minBlocks = blocks;
            leaf = c;
          }
        }
        for (auto child : c->children) findLeaf(child);
      };
  findLeaf(root);

  EXPECT(leaf != nullptr);
  EXPECT_LONGS_EQUAL(3, minBlocks);

  // Eliminate and solve
  solver.load(smoother);
  solver.eliminateInPlace();
  const VectorValues& actual = solver.updateSolution();

  GaussianBayesTree expectedBT = *smoother.eliminateMultifrontal(ordering);
  VectorValues expected = expectedBT.optimize();
  EXPECT(assert_equal(expected, actual, 1e-9));

  // Eliminate and solve after loading new values
  solver.load(smoother);
  solver.eliminateInPlace();
  const VectorValues& actual2 = solver.updateSolution();
  EXPECT(assert_equal(expected, actual2, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

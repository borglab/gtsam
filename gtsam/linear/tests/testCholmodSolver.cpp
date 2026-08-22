/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2026, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testCholmodSolver.cpp
 * @brief Tests for the reusable optional CHOLMOD Gaussian solver.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/linear/BatchJacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/internal/CholmodSolver.h>
#include <gtsam/linear/linearExceptions.h>

#include <stdexcept>
#include <string>

using namespace gtsam;

namespace {

GaussianFactorGraph jacobianGraph(double rhsScale = 1.0) {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(
      7, (Matrix(3, 2) << 2.0, 0.0, 0.0, 3.0, 1.0, -1.0).finished(), 42,
      (Matrix(3, 1) << 1.0, 2.0, -1.0).finished(),
      rhsScale * (Vector(3) << 1.0, 2.0, -0.5).finished());
  graph.emplace_shared<JacobianFactor>(
      42, (Matrix(2, 1) << 2.0, 1.0).finished(),
      rhsScale * (Vector(2) << 0.5, -1.0).finished());
  return graph;
}

GaussianFactorGraph batchJacobianGraph(
    double rhsScale = 1.0, const SharedDiagonal& model = SharedDiagonal()) {
  using Batch = BatchJacobianFactor<2, 2, 1>;
  auto batch = std::make_shared<Batch>(KeyVector{7, 42},
                                       std::vector<size_t>{2, 1}, model);
  batch->reserve(2);
  batch->addRow(Batch::SlotIndices{0, 1},
                {Matrix22{{1.0, 0.0}, {0.0, 1.0}}, Matrix21{{1.0}, {2.0}}},
                rhsScale * Vector2(1.0, -0.5));
  batch->addRow(Batch::SlotIndices{0, 1},
                {Matrix22{{2.0, 1.0}, {1.0, -1.0}}, Matrix21{{-1.0}, {0.5}}},
                rhsScale * Vector2(0.25, 2.0));
  return GaussianFactorGraph{batch};
}

}  // namespace

/* ************************************************************************* */
TEST(CholmodSolver, JacobianUserOrderingAndReuse) {
  internal::CholmodSolver solver;
  if (!internal::CholmodSolver::available()) {
    CHECK_EXCEPTION(solver.solve(jacobianGraph(), Ordering{42, 7}),
                    std::runtime_error);
    return;
  }

  const Ordering ordering{42, 7};
  for (double scale : {1.0, 2.0}) {
    const GaussianFactorGraph graph = jacobianGraph(scale);
    const VectorValues expected = graph.optimize(ordering);
    const VectorValues actual = solver.solve(graph, ordering);
    EXPECT(assert_equal(expected, actual, 1e-9));
  }
}

/* ************************************************************************* */
TEST(CholmodSolver, HessianFactors) {
  if (!internal::CholmodSolver::available()) return;

  const GaussianFactorGraph jacobians = jacobianGraph();
  GaussianFactorGraph hessians;
  for (const auto& factor : jacobians) {
    hessians.emplace_shared<HessianFactor>(
        *std::dynamic_pointer_cast<JacobianFactor>(factor));
  }

  const Ordering ordering{7, 42};
  internal::CholmodSolver solver;
  EXPECT(assert_equal(jacobians.optimize(ordering),
                      solver.solve(hessians, ordering), 1e-9));
}

/* ************************************************************************* */
TEST(CholmodSolver, MixedJacobianAndHessianFactors) {
  if (!internal::CholmodSolver::available()) return;

  const GaussianFactorGraph reference = jacobianGraph();
  GaussianFactorGraph mixed;
  mixed.push_back(reference.at(0));
  mixed.emplace_shared<HessianFactor>(
      *std::dynamic_pointer_cast<JacobianFactor>(reference.at(1)));

  const Ordering ordering{42, 7};
  internal::CholmodSolver solver;
  EXPECT(assert_equal(reference.optimize(ordering),
                      solver.solve(mixed, ordering), 1e-9));
}

/* ************************************************************************* */
// Verifies direct compact batch assembly, weighting, ordering, and reuse.
TEST(CholmodSolver, BatchJacobianFactors) {
  if (!internal::CholmodSolver::available()) return;

  const Ordering ordering{42, 7};
  const SharedDiagonal model = noiseModel::Diagonal::Sigmas(
      (Vector4() << 0.5, 1.0, 2.0, 0.75).finished());
  internal::CholmodSolver solver;
  for (double scale : {1.0, 2.0}) {
    const GaussianFactorGraph graph = batchJacobianGraph(scale, model);
    const auto batch =
        std::dynamic_pointer_cast<BatchJacobianFactorBase>(graph.at(0));
    GaussianFactorGraph expanded;
    expanded.emplace_shared<JacobianFactor>(batch->toJacobianFactor());
    EXPECT(assert_equal(expanded.optimize(ordering),
                        solver.solve(graph, ordering), 1e-9));
  }
}

/* ************************************************************************* */
TEST(CholmodSolver, InvalidOrdering) {
  if (!internal::CholmodSolver::available()) return;
  internal::CholmodSolver solver;
  CHECK_EXCEPTION(solver.solve(jacobianGraph(), Ordering{7, 7}),
                  std::invalid_argument);
}

/* ************************************************************************* */
TEST(CholmodSolver, RankDeficientSystem) {
  if (!internal::CholmodSolver::available()) return;

  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(7, (Matrix(1, 2) << 1.0, 0.0).finished(),
                                       Vector1(1.0));
  internal::CholmodSolver solver;
  CHECK_EXCEPTION(solver.solve(graph, Ordering{7}),
                  IndeterminateSystemException);
}

/* ************************************************************************* */
TEST(CholmodSolver, ConstrainedSystem) {
  if (!internal::CholmodSolver::available()) return;

  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(7, Matrix1::Identity(), Vector1(1.0),
                                       noiseModel::Constrained::All(1));
  internal::CholmodSolver solver;
  CHECK_EXCEPTION(solver.solve(graph, Ordering{7}), std::invalid_argument);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

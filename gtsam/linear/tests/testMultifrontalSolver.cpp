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
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/MultifrontalClique.h>
#include <gtsam/linear/MultifrontalSolver.h>
#include <gtsam/nonlinear/Marginals.h>
#include <tests/smallExample.h>

#include <cmath>
#include <functional>
#include <limits>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

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
  params.mergeDimCap = 0;
  params.leafMergeDimCap = 0;
  return params;
}

}  // namespace

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
  EXPECT(assert_equal((Matrix(2, 1) << 2., 1.).finished(), A0));

  // Block 3 (RHS):
  Matrix Ab = childClique->Ab()(3);  // 2x1
  EXPECT(assert_equal((Matrix(2, 1) << 2., 1.).finished(), Ab));
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
  EXPECT(assert_equal((Matrix(2, 1) << 2., 1.).finished(), A0));
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
  EXPECT(assert_equal((Matrix(2, 1) << 4., 2.).finished(), A0));
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
  graph.emplace_shared<JacobianFactor>(x1, I_1x1,
                                       (Vector(1) << 1.0).finished(), noise);
  graph.emplace_shared<JacobianFactor>(x1, I_1x1,
                                       (Vector(1) << -2.0).finished(), noise);
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
      noiseModel::Constrained::MixedSigmas((Vector(1) << 0.0).finished());
  GaussianFactorGraph constrainedChain = chain;
  constrainedChain.emplace_shared<JacobianFactor>(
      x2, I_1x1, (Vector(1) << 0.0).finished(), hardConstraint);

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
      noiseModel::Constrained::MixedSigmas((Vector(1) << 0.0).finished());
  const SharedDiagonal softNoise = noiseModel::Isotropic::Sigma(1, 10.0);
  GaussianFactorGraph graph;
  // Same setup as ConstrainedNoiseUnsupported, but feasible (b == 0).
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, (Vector(1) << 0.0).finished(),
                                       hardConstraint);
  graph.emplace_shared<JacobianFactor>(
      x1, I_1x1, (Vector(1) << 100.0).finished(), softNoise);
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
      noiseModel::Constrained::MixedSigmas((Vector(1) << 0.0).finished());
  const SharedDiagonal softNoise = noiseModel::Isotropic::Sigma(1, 10.0);
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, (Vector(1) << 1.0).finished(),
                                       hardConstraint);
  graph.emplace_shared<JacobianFactor>(
      x1, I_1x1, (Vector(1) << 100.0).finished(), softNoise);
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
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, (Vector(1) << 0.0).finished(),
                                       hardConstraint);
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, (Vector(1) << 5.0).finished(),
                                       softNoise);
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
  graph.emplace_shared<JacobianFactor>(
      x1, I_1x1, x2, I_1x1, (Vector(1) << 0.0).finished(), hardConstraint);
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
  graph.emplace_shared<JacobianFactor>(x1, I_1x1, (Vector(1) << 0.0).finished(),
                                       noiseModel::Isotropic::Sigma(1, sigma1));
  graph.emplace_shared<JacobianFactor>(x1, I_1x1,
                                       (Vector(1) << 10.0).finished(),
                                       noiseModel::Isotropic::Sigma(1, sigma2));

  const Ordering ordering{x1};
  MultifrontalSolver solver(graph, ordering, noMergeParams());
  solver.load(graph);
  solver.eliminateInPlace();
  const VectorValues& actual = solver.updateSolution();

  EXPECT_DOUBLES_EQUAL(8.0, actual.at(x1)(0), 1e-9);
}

/* ************************************************************************* */
// Hessian factors are rejected by the multifrontal solver.
TEST(MultifrontalSolver, HessianFactors) {
  GaussianFactorGraph graph;
  graph.emplace_shared<HessianFactor>(x1, (Matrix(1, 1) << 4.0).finished(),
                                      (Vector(1) << 8.0).finished(), 0.0);

  const Ordering ordering{x1};
  CHECK_EXCEPTION(
      { MultifrontalSolver solver(graph, ordering, noMergeParams()); },
      std::runtime_error);
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

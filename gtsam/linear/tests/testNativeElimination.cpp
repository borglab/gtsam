/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/linearExceptions.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;

/* ************************************************************************* */
namespace native_elimination {

GaussianFactorGraph anchoredPair(double sigma) {
  GaussianFactorGraph graph;
  graph.emplace_shared<JacobianFactor>(0, Matrix11{1.0}, Vector1{0.1});
  graph.emplace_shared<JacobianFactor>(
      0, Matrix11{-1.0}, 1, Matrix11{1.0}, Vector1{0.0},
      noiseModel::Isotropic::Sigma(1, sigma));
  return graph;
}

// Direct tree entry points must use the same scale as graph-level elimination.
TEST(NativeElimination, DirectTreesUseOriginalDiagonal) {
  const Ordering ordering{0, 1, 2};
  for (double sigma : {1e-4, 1e-5}) {
    auto graph = anchoredPair(sigma);
    graph.emplace_shared<JacobianFactor>(1, Matrix11{-1.0}, 2,
                                        Matrix11{1.0}, Vector1{0.0});
    GaussianEliminationTree tree(graph, ordering);
    GaussianJunctionTree junctionTree(tree);
    const auto expected = graph.optimize(ordering, EliminateQR);
    for (const GaussianFactorGraph::Eliminate& eliminate : {
             GaussianFactorGraph::Eliminate(EliminateCholesky),
             GaussianFactorGraph::Eliminate(EliminatePreferCholesky),
             GaussianFactorGraph::Eliminate(
                 EliminationTraits<GaussianFactorGraph>::DefaultEliminate)}) {
      if (sigma == 1e-4) {
        EXPECT(assert_equal(expected, tree.eliminate(eliminate).first->optimize(),
                            1e-7));
        EXPECT(assert_equal(
            expected, junctionTree.eliminate(eliminate).first->optimize(), 1e-7));
      } else {
        CHECK_EXCEPTION(tree.eliminate(eliminate), IndeterminateSystemException);
        CHECK_EXCEPTION(junctionTree.eliminate(eliminate),
                        IndeterminateSystemException);
      }
    }
  }
}

// Both internal phases of one marginal query retain the original input scale.
TEST(NativeElimination, MarginalCallsPreserveOriginalDiagonal) {
  for (double sigma : {1e-4, 1e-5}) {
    const auto graph = anchoredPair(sigma);
    const Ordering remaining{1}, eliminated{0};
    const KeyVector unorderedRemaining{1};
    if (sigma == 1e-4) {
      const VectorValues expected{{1, Vector1{0.1}}};
      EXPECT(assert_equal(expected,
          graph.marginalMultifrontalBayesNet(remaining, eliminated)->optimize(),
          1e-7));
      EXPECT(assert_equal(expected,
          graph.marginalMultifrontalBayesTree(remaining, eliminated)->optimize(),
          1e-7));
      EXPECT(assert_equal(expected,
          graph.marginalMultifrontalBayesNet(unorderedRemaining, eliminated)
              ->optimize(), 1e-7));
      EXPECT(assert_equal(expected,
          graph.marginalMultifrontalBayesTree(unorderedRemaining, eliminated)
              ->optimize(), 1e-7));
    } else {
      CHECK_EXCEPTION(graph.marginalMultifrontalBayesNet(remaining, eliminated),
                      IndeterminateSystemException);
      CHECK_EXCEPTION(graph.marginalMultifrontalBayesTree(remaining, eliminated),
                      IndeterminateSystemException);
      CHECK_EXCEPTION(
          graph.marginalMultifrontalBayesNet(unorderedRemaining, eliminated),
          IndeterminateSystemException);
      CHECK_EXCEPTION(
          graph.marginalMultifrontalBayesTree(unorderedRemaining, eliminated),
          IndeterminateSystemException);
    }
  }
}

// Removing a disconnected constraint must not rebind an already chosen local
// fallback policy when a marginal operation starts its second internal phase.
TEST(NativeElimination, ConstrainedMarginalPreservesFallback) {
  auto graph = anchoredPair(1e-5);
  graph.emplace_shared<JacobianFactor>(2, Matrix11{1.0}, Vector1{0.0},
                                      noiseModel::Constrained::All(1));
  const Ordering remaining{0, 1}, eliminated{2};
  const KeyVector unorderedRemaining{0, 1};
  const auto fullSequentialResult = [&](const Ordering& remainingOrdering) {
    Ordering ordering{2};
    ordering.insert(ordering.end(), remainingOrdering.begin(),
                    remainingOrdering.end());
    auto result = graph.eliminateSequential(ordering)->optimize();
    result.erase(2);
    return result;
  };
  EXPECT(assert_equal(fullSequentialResult(remaining),
      graph.marginalMultifrontalBayesNet(remaining, eliminated)->optimize()));
  EXPECT(assert_equal(fullSequentialResult(Ordering::Colamd(anchoredPair(1e-5))),
      graph.marginalMultifrontalBayesNet(unorderedRemaining, eliminated)
          ->optimize()));
}

// Numerical preparation must not replace custom callbacks or impose checks on QR.
TEST(NativeElimination, CustomAndQrCallbacksRemainUnchanged) {
  const auto graph = anchoredPair(1e-5);
  const Ordering ordering{0, 1};
  GaussianEliminationTree tree(graph, ordering);
  size_t calls = 0;
  const auto custom = [&](const GaussianFactorGraph& factors,
                          const Ordering& keys) {
    ++calls;
    return EliminatePreferCholesky(factors, keys);
  };
  // Compare with the raw local policy, whose rounding differs from QR here.
  const auto first = EliminatePreferCholesky(graph, Ordering{0});
  const auto second = EliminatePreferCholesky(
      GaussianFactorGraph{first.second}, Ordering{1});
  const GaussianBayesNet rawSequential{first.first, second.first};
  EXPECT(assert_equal(rawSequential.optimize(),
                      tree.eliminate(custom).first->optimize()));
  EXPECT_LONGS_EQUAL(2, calls);
  EXPECT(assert_equal(graph.optimize(ordering, EliminateQR),
                      tree.eliminate(EliminateQR).first->optimize()));
}

}  // namespace native_elimination
/* ************************************************************************* */

int main() {
  TestResult result;
  return TestRegistry::runAllTests(result);
}

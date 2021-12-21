/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    testDCFactorGraph.cpp
 * @brief   Unit tests for DCFactorGraph
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/DCFactor.h>
#include <gtsam/hybrid/DCFactorGraph.h>
#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

/// Hybrid conditional resulting from elimination
class DCCondtional {
 public:
  using shared_ptr = boost::shared_ptr<DCCondtional>;
};

/// Bayes net
class DCBayesNet : public BayesNet<DCCondtional> {};

/* EliminateableFactorGraph does not play well with hybrid:
// Forward declaration
class MixtureFactorGraph;
class Dummy;

template <>
struct EliminationTraits<MixtureFactorGraph> {
  typedef DCFactor FactorType;
  typedef DCFactorGraph FactorGraphType;
  typedef DCCondtional ConditionalType;
  typedef DCBayesNet BayesNetType;
  typedef Dummy EliminationTreeType;
  typedef Dummy BayesTreeType;
  typedef Dummy JunctionTreeType;

  /// The function type that does a single dense elimination step on a
  static std::pair<DCCondtional::shared_ptr, DCFactor::shared_ptr>
  DefaultEliminate(const DCFactor& factors, const Ordering& ordering) {
    auto conditional = boost::make_shared<DCCondtional>();
    auto factor = boost::make_shared<DCFactor>();
    return {conditional, factor};
  }
};

/// Speical DCFactorGraph that can be eliminated partially
class MixtureFactorGraph : public FactorGraph<DCMixtureFactor>,
                           public EliminateableFactorGraph<MixtureFactorGraph> {
};
*/

/* ****************************************************************************
 * Test elimination on a switching-like hybrid factor graph.
 */
TEST(DiscreteBayesTree, Switching) {
  // Number of time steps.
  const size_t K = 5;

  // Create DiscreteKeys for binary K+1 modes.
  DiscreteKeys modes;
  for (size_t k = 0; k <= K; k++) {
    modes.emplace_back(M(k), 2);
  }

  // Add a prior on X(1).
  using PriorMixture = DCMixtureFactor<PriorFactor<double>>;
  PriorFactor<double> prior(X(1), 0, Isotropic::Sigma(1, 0.1));
  PriorMixture priorMixture({X(1)}, modes[0], {prior, prior});

  DCFactorGraph fg;
  fg.add(priorMixture);

  // Add "motion models".
  for (size_t k = 1; k < K; k++) {
    BetweenFactor<double> still(X(k), X(k + 1), 0.0, Isotropic::Sigma(2, 1.0)),
        moving(X(k), X(k + 1), 1.0, Isotropic::Sigma(2, 1.0));
    using MotionMixture = DCMixtureFactor<BetweenFactor<double>>;
    MotionMixture mixture({X(k), X(k + 1)}, modes[k], {still, moving});
    fg.add(mixture);
  }
  GTSAM_PRINT(fg);
  fg.saveGraph("MixtureFactorGraph.dot");
  
  //   // Add "mode chain"
  //   for (size_t k = 1; k < K1; k++) {
  //     DiscreteKey mode(M(k), 2), mode_plus(M(k + 1), 2);
  //     fg.add(DiscreteConditional(mode_plus, {mode}, "1/2 3/2"));
  //   }

  // eliminate partially:
  //   Ordering ordering;
  //   for (size_t k = 1; k <= K; k++) ordering += X(k);
  //   auto chordal = fg.eliminatePartialSequential(ordering);
  //   GTSAM_PRINT(*chordal);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

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
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

namespace gtsam {

/// A set of GaussianConditionals, indexed by a set of discrete variables.
class DCGaussianConditional {
 public:
  using shared_ptr = boost::shared_ptr<DCGaussianConditional>;
  void print(const std::string& s = "DCGaussianConditional",
             const KeyFormatter& formatter = gtsam::DefaultKeyFormatter) const {
    std::cout << (s.empty() ? "" : s + " ") << std::endl;
  }
};

/// A factor graph containing only Gaussian mixture factors
class DCGaussianMixtureFactorGraph
    : public gtsam::FactorGraph<DCGaussianMixtureFactor> {
 public:
  using shared_ptr = boost::shared_ptr<DCGaussianMixtureFactorGraph>;
  using EliminationResult =
      std::pair<boost::shared_ptr<DCConditional>,
                boost::shared_ptr<DCGaussianMixtureFactor>>;
  using Eliminate = std::function<EliminationResult(
      const DCGaussianMixtureFactorGraph&, const Ordering&)>;

  DCGaussianMixtureFactorGraph() : FactorGraph<DCGaussianMixtureFactor>() {}
};

/// Bayes net
class DCGaussianBayesNet : public BayesNet<DCGaussianConditional> {
 public:
  using ConditionalType = DCGaussianConditional;
  using shared_ptr = boost::shared_ptr<DCGaussianBayesNet>;
};

// Specialize base class
// template class EliminationTree<DCGaussianBayesNet, DCFactorGraph>;

class GTSAM_EXPORT DCGaussianMixtureEliminationTree
    : public EliminationTree<DCGaussianBayesNet, DCGaussianMixtureFactorGraph> {
 public:
  typedef EliminationTree<DCGaussianBayesNet, DCGaussianMixtureFactorGraph>
      Base;                                       ///< Base class
  typedef DCGaussianMixtureEliminationTree This;  ///< This class
  typedef boost::shared_ptr<This> shared_ptr;  ///< Shared pointer to this class

  /**
   * Build the elimination tree of a factor graph using pre-computed column
   * structure.
   * @param factorGraph The factor graph for which to build the elimination tree
   * @param structure The set of factors involving each variable.  If this is
   * not precomputed, you can call the Create(const FactorGraph<DERIVEDFACTOR>&)
   * named constructor instead.
   * @return The elimination tree
   */
  DCGaussianMixtureEliminationTree(
      const DCGaussianMixtureFactorGraph& factorGraph,
      const VariableIndex& structure, const Ordering& order)
      : Base(factorGraph, structure, order) {}

  /** Build the elimination tree of a factor graph.  Note that this has to
   * compute the column structure as a VariableIndex, so if you already have
   * this precomputed, use the other constructor instead.
   * @param factorGraph The factor graph for which to build the elimination tree
   */
  DCGaussianMixtureEliminationTree(
      const DCGaussianMixtureFactorGraph& factorGraph, const Ordering& order)
      : Base(factorGraph, order) {}

  /** Test whether the tree is equal to another */
  // bool equals(const This& other, double tol = 1e-9) const;

 private:
  friend class ::EliminationTreeTester;
};

// Forward declaration
class Dummy;

template <>
struct EliminationTraits<DCGaussianMixtureFactorGraph> {
  typedef DCGaussianMixtureFactor FactorType;
  typedef DCGaussianMixtureFactorGraph FactorGraphType;
  typedef DCGaussianConditional ConditionalType;
  typedef DCGaussianBayesNet BayesNetType;
  typedef DCGaussianMixtureEliminationTree EliminationTreeType;
  typedef Dummy BayesTreeType;
  typedef Dummy JunctionTreeType;

  /// The function type that does a single elimination step on a variable.
  static std::pair<DCGaussianConditional::shared_ptr,
                   DCGaussianMixtureFactor::shared_ptr>
  DefaultEliminate(const DCGaussianMixtureFactorGraph& factors,
                   const Ordering& ordering) {
    // We are getting a number of DCMixtureFactors on a set of continuous
    // variables. They might all have different discrete keys. For every
    // possible combination of the discrete keys, we need a GaussianConditional.
    for (const auto& factor : factors) {
      if (auto p = boost::dynamic_pointer_cast<const DCGaussianMixtureFactor>(
              factor)) {
        GTSAM_PRINT(*p);
      };
    }

    // Create a DCGaussianConditional...
    auto conditional = boost::make_shared<DCGaussianConditional>();

    // Create a resulting DCGaussianMixture on the separator.
    /// auto factor = TODO ...
    return {conditional, nullptr};
  }

  // TODO(dellaert): just does not make sense to return shared pointers.
  static std::pair<DCGaussianBayesNet::shared_ptr,
                   DCGaussianMixtureFactorGraph::shared_ptr>
  eliminatePartialSequential(const DCGaussianMixtureFactorGraph& graph,
                             const Ordering& ordering) {
    // Variable index only knows about continuous variables.
    VariableIndex variableIndex(graph);
    GTSAM_PRINT(variableIndex);

    DCGaussianMixtureEliminationTree etree(graph, variableIndex, ordering);
    GTSAM_PRINT(etree);
    // return etree.eliminate(function);

    auto bayesNet = boost::make_shared<DCGaussianBayesNet>();
    auto factors = boost::make_shared<DCGaussianMixtureFactorGraph>();
    return {bayesNet, factors};
  }
};

/* TODO(dellaert) EliminateableFactorGraph does not play well with hybrid:
/// Speical DCFactorGraph that can be eliminated partially
class MixtureFactorGraph : public FactorGraph<DCMixtureFactor>,
                           public EliminateableFactorGraph<MixtureFactorGraph> {
};
*/
}  // namespace gtsam

using namespace std;
using namespace gtsam;
using noiseModel::Isotropic;
using symbol_shorthand::M;
using symbol_shorthand::X;

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

  // Create hybrid factor graph.
  DCFactorGraph fg;

  // Add a prior on X(1).
  // TODO: make a prior that does not depend on a discrete key.
  using PriorMixture = DCMixtureFactor<PriorFactor<double>>;
  PriorFactor<double> prior(X(1), 0, Isotropic::Sigma(1, 0.1));
  PriorMixture priorMixture({X(1)}, DiscreteKeys{modes[0]}, {prior, prior});
  fg.add(priorMixture);

  // Add "motion models".
  for (size_t k = 1; k < K; k++) {
    BetweenFactor<double> still(X(k), X(k + 1), 0.0, Isotropic::Sigma(2, 1.0)),
        moving(X(k), X(k + 1), 1.0, Isotropic::Sigma(2, 1.0));
    using MotionMixture = DCMixtureFactor<BetweenFactor<double>>;
    MotionMixture mixture({X(k), X(k + 1)}, DiscreteKeys{modes[k]},
                          {still, moving});
    fg.add(mixture);
  }
  GTSAM_PRINT(fg);
  fg.saveGraph("MixtureFactorGraph.dot");

  //   // Add "mode chain": can only be done in HybridFactorGraph
  //   for (size_t k = 1; k < K1; k++) {
  //     DiscreteKey mode(M(k), 2), mode_plus(M(k + 1), 2);
  //     fg.add(DiscreteConditional(mode_plus, {mode}, "1/2 3/2"));
  //   }

  // Linearize here:
  // Values values;
  DCGaussianMixtureFactorGraph dcmfg;  // TODO: = fg.linearize(values);
  GTSAM_PRINT(dcmfg);

  //TODO(Varun) uncomment and finish implementing
  // // Eliminate partially.
  // Ordering ordering;
  // for (size_t k = 1; k <= K; k++) ordering += X(k);
  // using dc_traits = EliminationTraits<DCGaussianMixtureFactorGraph>;
  // auto result = dc_traits::eliminatePartialSequential(dcmfg, ordering);
  // GTSAM_PRINT(*result.first); // DCGaussianBayesNet
  // GTSAM_PRINT(*result.second); // DCGaussianMixtureFactorGraph
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

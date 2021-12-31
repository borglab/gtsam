/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testDCGaussianElimination.cpp
 * @brief   Unit tests for eliminating a hybrid factor graph
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/hybrid/DCMixtureFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/EliminateableFactorGraph.h>
#include <gtsam/inference/EliminationTree-inst.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Include for test suite
#include <CppUnitLite/TestHarness.h>

namespace gtsam {

class GTSAM_EXPORT HybridEliminationTree
    : public EliminationTree<HybridBayesNet, HybridFactorGraph> {
 public:
  typedef EliminationTree<HybridBayesNet, HybridFactorGraph>
      Base;                                    ///< Base class
  typedef HybridEliminationTree This;          ///< This class
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
  HybridEliminationTree(const HybridFactorGraph& factorGraph,
                        const VariableIndex& structure, const Ordering& order)
      : Base(factorGraph, structure, order) {}

  /** Build the elimination tree of a factor graph.  Note that this has to
   * compute the column structure as a VariableIndex, so if you already have
   * this precomputed, use the other constructor instead.
   * @param factorGraph The factor graph for which to build the elimination tree
   */
  HybridEliminationTree(const HybridFactorGraph& factorGraph,
                        const Ordering& order)
      : Base(factorGraph, order) {}

  /** Test whether the tree is equal to another */
  // bool equals(const This& other, double tol = 1e-9) const;

 private:
  friend class ::EliminationTreeTester;
};

// Forward declaration
class Dummy;

template <>
struct EliminationTraits<HybridFactorGraph> {
  typedef Factor FactorType;
  typedef HybridFactorGraph FactorGraphType;
  typedef DCConditional ConditionalType;
  typedef HybridBayesNet BayesNetType;
  typedef HybridEliminationTree EliminationTreeType;
  typedef Dummy BayesTreeType;
  typedef Dummy JunctionTreeType;

  /// The function type that does a single elimination step on a variable.
  static std::pair<DCConditional::shared_ptr, boost::shared_ptr<Factor>>
  DefaultEliminate(const HybridFactorGraph& factors, const Ordering& ordering) {
    // We are getting a number of DCMixtureFactors on a set of continuous
    // variables. They might all have different discrete keys. For every
    // possible combination of the discrete keys, we need a GaussianConditional.
    // for (const auto& factor : factors) {
    //   if (auto p = boost::dynamic_pointer_cast<const
    //   DCGaussianMixtureFactor>(
    //           factor)) {
    //     GTSAM_PRINT(*p);
    //   };
    // }

    std::cout << "DefaultEliminate" << std::endl;

    // Create a DCConditional...
    auto conditional = boost::make_shared<DCConditional>();

    // Create a resulting DCGaussianMixture on the separator.
    /// auto factor = TODO ...
    return {conditional, nullptr};
  }

  // TODO(dellaert): just does not make sense to return shared pointers.
  static std::pair<HybridBayesNet::shared_ptr, HybridFactorGraph::shared_ptr>
  eliminatePartialSequential(const HybridFactorGraph& graph,
                             const Ordering& ordering) {
    std::cout << "eliminatePartialSequential" << std::endl;

    // Variable index only knows about continuous variables.
    VariableIndex variableIndex(graph);
    GTSAM_PRINT(variableIndex);

    HybridEliminationTree etree(graph, variableIndex, ordering);
    GTSAM_PRINT(etree);
    return etree.eliminate(DefaultEliminate);

    // auto bayesNet = boost::make_shared<HybridBayesNet>();
    // auto factors = boost::make_shared<HybridFactorGraph>();
    // return {bayesNet, factors};
  }
};

/* TODO(dellaert) EliminateableFactorGraph does not play well with hybrid:
/// Special DCFactorGraph that can be eliminated partially
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
TEST_UNSAFE(DCGaussianElimination, Switching) {
  // Number of time steps.
  const size_t K = 3;

  // Create DiscreteKeys for binary K modes, modes[0] will not be used.
  DiscreteKeys modes;
  for (size_t k = 0; k <= K; k++) {
    modes.emplace_back(M(k), 2);
  }

  // Create hybrid factor graph.
  HybridFactorGraph fg;

  Values values;
  values.insert<double>(X(1), 0);
  values.insert<double>(X(2), 1);
  values.insert<double>(X(3), 1);

  // Add a prior on X(1).
  PriorFactor<double> prior(X(1), 0, Isotropic::Sigma(1, 0.1));
  auto gaussian = prior.linearize(values);
  fg.push_gaussian(gaussian);

  // Add "motion models".
  for (size_t k = 1; k < K; k++) {
    BetweenFactor<double> still(X(k), X(k + 1), 0.0, Isotropic::Sigma(1, 1.0)),
        moving(X(k), X(k + 1), 1.0, Isotropic::Sigma(1, 1.0));
    using MotionMixture = DCMixtureFactor<BetweenFactor<double>>;
    auto keys = {X(k), X(k + 1)};
    auto components = {still.linearize(values), moving.linearize(values)};
    fg.emplace_shared<DCGaussianMixtureFactor>(keys, DiscreteKeys{modes[k]},
                                               components);
  }

  // Add "mode chain": can only be done in HybridFactorGraph
  fg.push_discrete(DiscreteConditional(modes[1], {}, "1/1"));
  for (size_t k = 1; k < K; k++) {
    auto parents = {modes[k]};
    fg.emplace_shared<DiscreteConditional>(modes[k + 1], parents, "1/2 3/2");
  }

  GTSAM_PRINT(fg);

  // Eliminate partially.
  Ordering ordering;
  for (size_t k = 1; k <= K; k++) ordering += X(k);
  using dc_traits = EliminationTraits<HybridFactorGraph>;
  auto result = dc_traits::eliminatePartialSequential(fg, ordering);
  GTSAM_PRINT(*result.first);   // HybridBayesNet
  GTSAM_PRINT(*result.second);  // HybridFactorGraph
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

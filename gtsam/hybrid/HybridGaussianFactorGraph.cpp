/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianFactorGraph.cpp
 * @brief  Hybrid factor graph that uses type erasure
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 11, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DiscreteEliminationTree.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteJunctionTree.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianFactorGraph.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianEliminationTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianJunctionTree.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

// #define HYBRID_TIMING

namespace gtsam {

/// Specialize EliminateableFactorGraph for HybridGaussianFactorGraph:
template class EliminateableFactorGraph<HybridGaussianFactorGraph>;

/* ************************************************************************ */
static GaussianFactorGraphTree addGaussian(
    const GaussianFactorGraphTree &sum,
    const GaussianFactor::shared_ptr &factor) {
  // If the decision tree is not initialized, then initialize it.
  if (sum.empty()) {
    GaussianFactorGraph result;
    result.push_back(factor);
    return GaussianFactorGraphTree(GraphAndConstant(result, 0.0));

  } else {
    auto add = [&factor](const GraphAndConstant &graph_z) {
      auto result = graph_z.graph;
      result.push_back(factor);
      return GraphAndConstant(result, graph_z.constant);
    };
    return sum.apply(add);
  }
}

/* ************************************************************************ */
// TODO(dellaert): We need to document why deferredFactors need to be
// added last, which I would undo if possible. Implementation-wise, it's
// probably more efficient to first collect the discrete keys, and then loop
// over all assignments to populate a vector.
GaussianFactorGraphTree HybridGaussianFactorGraph::assembleGraphTree() const {
  gttic(assembleGraphTree);

  GaussianFactorGraphTree result;
  std::vector<GaussianFactor::shared_ptr> deferredFactors;

  for (auto &f : factors_) {
    // TODO(dellaert): just use a virtual method defined in HybridFactor.
    if (f->isHybrid()) {
      if (auto gm = boost::dynamic_pointer_cast<GaussianMixtureFactor>(f)) {
        result = gm->add(result);
      }
      if (auto gm = boost::dynamic_pointer_cast<HybridConditional>(f)) {
        result = gm->asMixture()->add(result);
      }

    } else if (f->isContinuous()) {
      if (auto gf = boost::dynamic_pointer_cast<HybridGaussianFactor>(f)) {
        deferredFactors.push_back(gf->inner());
      }
      if (auto cg = boost::dynamic_pointer_cast<HybridConditional>(f)) {
        deferredFactors.push_back(cg->asGaussian());
      }

    } else if (f->isDiscrete()) {
      // Don't do anything for discrete-only factors
      // since we want to eliminate continuous values only.
      continue;

    } else {
      // We need to handle the case where the object is actually an
      // BayesTreeOrphanWrapper!
      auto orphan = boost::dynamic_pointer_cast<
          BayesTreeOrphanWrapper<HybridBayesTree::Clique>>(f);
      if (!orphan) {
        auto &fr = *f;
        throw std::invalid_argument(
            std::string("factor is discrete in continuous elimination ") +
            demangle(typeid(fr).name()));
      }
    }
  }

  for (auto &f : deferredFactors) {
    result = addGaussian(result, f);
  }

  gttoc(assembleGraphTree);

  return result;
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>
continuousElimination(const HybridGaussianFactorGraph &factors,
                      const Ordering &frontalKeys) {
  GaussianFactorGraph gfg;
  for (auto &fp : factors) {
    if (auto ptr = boost::dynamic_pointer_cast<HybridGaussianFactor>(fp)) {
      gfg.push_back(ptr->inner());
    } else if (auto ptr = boost::static_pointer_cast<HybridConditional>(fp)) {
      gfg.push_back(
          boost::static_pointer_cast<GaussianConditional>(ptr->inner()));
    } else {
      // It is an orphan wrapped conditional
    }
  }

  auto result = EliminatePreferCholesky(gfg, frontalKeys);
  return {boost::make_shared<HybridConditional>(result.first),
          boost::make_shared<HybridGaussianFactor>(result.second)};
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>
discreteElimination(const HybridGaussianFactorGraph &factors,
                    const Ordering &frontalKeys) {
  DiscreteFactorGraph dfg;

  for (auto &factor : factors) {
    if (auto p = boost::dynamic_pointer_cast<HybridDiscreteFactor>(factor)) {
      dfg.push_back(p->inner());
    } else if (auto p = boost::static_pointer_cast<HybridConditional>(factor)) {
      auto discrete_conditional =
          boost::static_pointer_cast<DiscreteConditional>(p->inner());
      dfg.push_back(discrete_conditional);
    } else {
      // It is an orphan wrapper
    }
  }

  // TODO(dellaert): This does sum-product. For max-product, use EliminateForMPE
  auto result = EliminateDiscrete(dfg, frontalKeys);

  return {boost::make_shared<HybridConditional>(result.first),
          boost::make_shared<HybridDiscreteFactor>(result.second)};
}

/* ************************************************************************ */
// If any GaussianFactorGraph in the decision tree contains a nullptr, convert
// that leaf to an empty GaussianFactorGraph. Needed since the DecisionTree will
// otherwise create a GFG with a single (null) factor.
GaussianFactorGraphTree removeEmpty(const GaussianFactorGraphTree &sum) {
  auto emptyGaussian = [](const GraphAndConstant &graph_z) {
    bool hasNull =
        std::any_of(graph_z.graph.begin(), graph_z.graph.end(),
                    [](const GaussianFactor::shared_ptr &ptr) { return !ptr; });
    return hasNull ? GraphAndConstant{GaussianFactorGraph(), 0.0} : graph_z;
  };
  return GaussianFactorGraphTree(sum, emptyGaussian);
}
/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>
hybridElimination(const HybridGaussianFactorGraph &factors,
                  const Ordering &frontalKeys,
                  const KeyVector &continuousSeparator,
                  const std::set<DiscreteKey> &discreteSeparatorSet) {
  // NOTE: since we use the special JunctionTree,
  // only possibility is continuous conditioned on discrete.
  DiscreteKeys discreteSeparator(discreteSeparatorSet.begin(),
                                 discreteSeparatorSet.end());

  // Collect all the factors to create a set of Gaussian factor graphs in a
  // decision tree indexed by all discrete keys involved.
  GaussianFactorGraphTree sum = factors.assembleGraphTree();

  // TODO(dellaert): does assembleGraphTree not guarantee we do not need this?
  sum = removeEmpty(sum);

  using EliminationPair = std::pair<boost::shared_ptr<GaussianConditional>,
                                    GaussianMixtureFactor::FactorAndConstant>;

  // This is the elimination method on the leaf nodes
  auto eliminate = [&](const GraphAndConstant &graph_z) -> EliminationPair {
    if (graph_z.graph.empty()) {
      return {nullptr, {nullptr, 0.0}};
    }

#ifdef HYBRID_TIMING
    gttic_(hybrid_eliminate);
#endif

    boost::shared_ptr<GaussianConditional> conditional;
    boost::shared_ptr<GaussianFactor> newFactor;
    boost::tie(conditional, newFactor) =
        EliminatePreferCholesky(graph_z.graph, frontalKeys);

#ifdef HYBRID_TIMING
    gttoc_(hybrid_eliminate);
#endif

    // Get the log of the log normalization constant inverse.
    double logZ = -conditional->logNormalizationConstant();

    // IF this is the last continuous variable to eliminated, we need to
    // calculate the error here: the value of all factors at the mean, see
    // ml_map_rao.pdf.
    if (continuousSeparator.empty()) {
      const auto posterior_mean = conditional->solve(VectorValues());
      logZ += graph_z.graph.error(posterior_mean);
    }
    return {conditional, {newFactor, logZ}};
  };

  // Perform elimination!
  DecisionTree<Key, EliminationPair> eliminationResults(sum, eliminate);

#ifdef HYBRID_TIMING
  tictoc_print_();
  tictoc_reset_();
#endif

  // Separate out decision tree into conditionals and remaining factors.
  GaussianMixture::Conditionals conditionals;
  GaussianMixtureFactor::Factors newFactors;
  std::tie(conditionals, newFactors) = unzip(eliminationResults);

  // Create the GaussianMixture from the conditionals
  auto gaussianMixture = boost::make_shared<GaussianMixture>(
      frontalKeys, continuousSeparator, discreteSeparator, conditionals);

  // If there are no more continuous parents, then we should create a
  // DiscreteFactor here, with the error for each discrete choice.
  if (continuousSeparator.empty()) {
    auto factorProb =
        [&](const GaussianMixtureFactor::FactorAndConstant &factor_z) {
          // This is the probability q(μ) at the MLE point.
          // return exp(-factor_z.error(VectorValues()));
          // TODO(dellaert): this is not correct, since VectorValues() is not
          // the MLE point. But it does not matter, as at the MLE point the
          // error will be zero, hence:
          return exp(-factor_z.constant);
        };
    const DecisionTree<Key, double> fdt(newFactors, factorProb);
    const auto discreteFactor =
        boost::make_shared<DecisionTreeFactor>(discreteSeparator, fdt);

    return {boost::make_shared<HybridConditional>(gaussianMixture),
            boost::make_shared<HybridDiscreteFactor>(discreteFactor)};
  } else {
    // Create a resulting GaussianMixtureFactor on the separator.
    // TODO(dellaert): Keys can be computed from the factors, so we should not
    // need to pass them in.
    return {boost::make_shared<HybridConditional>(gaussianMixture),
            boost::make_shared<GaussianMixtureFactor>(
                continuousSeparator, discreteSeparator, newFactors)};
  }
}

/* ************************************************************************
 * Function to eliminate variables **under the following assumptions**:
 * 1. When the ordering is fully continuous, and the graph only contains
 * continuous and hybrid factors
 * 2. When the ordering is fully discrete, and the graph only contains discrete
 * factors
 *
 * Any usage outside of this is considered incorrect.
 *
 * \warning This function is not meant to be used with arbitrary hybrid factor
 * graphs. For example, if there exists continuous parents, and one tries to
 * eliminate a discrete variable (as specified in the ordering), the result will
 * be INCORRECT and there will be NO error raised.
 */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
EliminateHybrid(const HybridGaussianFactorGraph &factors,
                const Ordering &frontalKeys) {
  // NOTE: Because we are in the Conditional Gaussian regime there are only
  // a few cases:
  // 1. continuous variable, make a Gaussian Mixture if there are hybrid
  // factors;
  // 2. continuous variable, we make a Gaussian Factor if there are no hybrid
  // factors;
  // 3. discrete variable, no continuous factor is allowed
  // (escapes Conditional Gaussian regime), if discrete only we do the discrete
  // elimination.

  // However it is not that simple. During elimination it is possible that the
  // multifrontal needs to eliminate an ordering that contains both Gaussian and
  // hybrid variables, for example x1, c1.
  // In this scenario, we will have a density P(x1, c1) that is a Conditional
  // Linear Gaussian P(x1|c1)P(c1) (see Murphy02).

  // The issue here is that, how can we know which variable is discrete if we
  // unify Values? Obviously we can tell using the factors, but is that fast?

  // In the case of multifrontal, we will need to use a constrained ordering
  // so that the discrete parts will be guaranteed to be eliminated last!
  // Because of all these reasons, we carefully consider how to
  // implement the hybrid factors so that we do not get poor performance.

  // The first thing is how to represent the GaussianMixture.
  // A very possible scenario is that the incoming factors will have different
  // levels of discrete keys. For example, imagine we are going to eliminate the
  // fragment: $\phi(x1,c1,c2)$, $\phi(x1,c2,c3)$, which is perfectly valid.
  // Now we will need to know how to retrieve the corresponding continuous
  // densities for the assignment (c1,c2,c3) (OR (c2,c3,c1), note there is NO
  // defined order!). We also need to consider when there is pruning. Two
  // mixture factors could have different pruning patterns - one could have
  // (c1=0,c2=1) pruned, and another could have (c2=0,c3=1) pruned, and this
  // creates a big problem in how to identify the intersection of non-pruned
  // branches.

  // Our approach is first building the collection of all discrete keys. After
  // that we enumerate the space of all key combinations *lazily* so that the
  // exploration branch terminates whenever an assignment yields NULL in any of
  // the hybrid factors.

  // When the number of assignments is large we may encounter stack overflows.
  // However this is also the case with iSAM2, so no pressure :)

  // PREPROCESS: Identify the nature of the current elimination

  // First, identify the separator keys, i.e. all keys that are not frontal.
  KeySet separatorKeys;
  for (auto &&factor : factors) {
    separatorKeys.insert(factor->begin(), factor->end());
  }
  // remove frontals from separator
  for (auto &k : frontalKeys) {
    separatorKeys.erase(k);
  }

  // Build a map from keys to DiscreteKeys
  std::unordered_map<Key, DiscreteKey> mapFromKeyToDiscreteKey;
  for (auto &&factor : factors) {
    if (!factor->isContinuous()) {
      for (auto &k : factor->discreteKeys()) {
        mapFromKeyToDiscreteKey[k.first] = k;
      }
    }
  }

  // Fill in discrete frontals and continuous frontals.
  std::set<DiscreteKey> discreteFrontals;
  KeySet continuousFrontals;
  for (auto &k : frontalKeys) {
    if (mapFromKeyToDiscreteKey.find(k) != mapFromKeyToDiscreteKey.end()) {
      discreteFrontals.insert(mapFromKeyToDiscreteKey.at(k));
    } else {
      continuousFrontals.insert(k);
    }
  }

  // Fill in discrete discrete separator keys and continuous separator keys.
  std::set<DiscreteKey> discreteSeparatorSet;
  KeyVector continuousSeparator;
  for (auto &k : separatorKeys) {
    if (mapFromKeyToDiscreteKey.find(k) != mapFromKeyToDiscreteKey.end()) {
      discreteSeparatorSet.insert(mapFromKeyToDiscreteKey.at(k));
    } else {
      continuousSeparator.push_back(k);
    }
  }

  // Check if we have any continuous keys:
  const bool discrete_only =
      continuousFrontals.empty() && continuousSeparator.empty();

  // NOTE: We should really defer the product here because of pruning

  if (discrete_only) {
    // Case 1: we are only dealing with discrete
    return discreteElimination(factors, frontalKeys);
  } else if (mapFromKeyToDiscreteKey.empty()) {
    // Case 2: we are only dealing with continuous
    return continuousElimination(factors, frontalKeys);
  } else {
    // Case 3: We are now in the hybrid land!
#ifdef HYBRID_TIMING
    tictoc_reset_();
#endif
    return hybridElimination(factors, frontalKeys, continuousSeparator,
                             discreteSeparatorSet);
  }
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(JacobianFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(std::move(factor)));
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(boost::shared_ptr<JacobianFactor> &factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(factor));
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(DecisionTreeFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridDiscreteFactor>(std::move(factor)));
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::add(DecisionTreeFactor::shared_ptr factor) {
  FactorGraph::add(boost::make_shared<HybridDiscreteFactor>(factor));
}

/* ************************************************************************ */
const Ordering HybridGaussianFactorGraph::getHybridOrdering() const {
  KeySet discrete_keys = discreteKeys();
  for (auto &factor : factors_) {
    for (const DiscreteKey &k : factor->discreteKeys()) {
      discrete_keys.insert(k.first);
    }
  }

  const VariableIndex index(factors_);
  Ordering ordering = Ordering::ColamdConstrainedLast(
      index, KeyVector(discrete_keys.begin(), discrete_keys.end()), true);
  return ordering;
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::error(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree(0.0);

  // Iterate over each factor.
  for (size_t idx = 0; idx < size(); idx++) {
    // TODO(dellaert): just use a virtual method defined in HybridFactor.
    AlgebraicDecisionTree<Key> factor_error;

    if (factors_.at(idx)->isHybrid()) {
      // If factor is hybrid, select based on assignment.
      GaussianMixtureFactor::shared_ptr gaussianMixture =
          boost::static_pointer_cast<GaussianMixtureFactor>(factors_.at(idx));
      // Compute factor error and add it.
      error_tree = error_tree + gaussianMixture->error(continuousValues);

    } else if (factors_.at(idx)->isContinuous()) {
      // If continuous only, get the (double) error
      // and add it to the error_tree
      auto hybridGaussianFactor =
          boost::static_pointer_cast<HybridGaussianFactor>(factors_.at(idx));
      GaussianFactor::shared_ptr gaussian = hybridGaussianFactor->inner();

      // Compute the error of the gaussian factor.
      double error = gaussian->error(continuousValues);
      // Add the gaussian factor error to every leaf of the error tree.
      error_tree = error_tree.apply(
          [error](double leaf_value) { return leaf_value + error; });

    } else if (factors_.at(idx)->isDiscrete()) {
      // If factor at `idx` is discrete-only, we skip.
      continue;
    }
  }

  return error_tree;
}

/* ************************************************************************ */
double HybridGaussianFactorGraph::error(const HybridValues &values) const {
  double error = 0.0;
  for (auto &factor : factors_) {
    error += factor->error(values);
  }
  return error;
}

/* ************************************************************************ */
double HybridGaussianFactorGraph::probPrime(const HybridValues &values) const {
  double error = this->error(values);
  // NOTE: The 0.5 term is handled by each factor
  return std::exp(-error);
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::probPrime(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree = this->error(continuousValues);
  AlgebraicDecisionTree<Key> prob_tree = error_tree.apply([](double error) {
    // NOTE: The 0.5 term is handled by each factor
    return exp(-error);
  });
  return prob_tree;
}

}  // namespace gtsam

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
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
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
#include <utility>
#include <vector>

namespace gtsam {

/// Specialize EliminateableFactorGraph for HybridGaussianFactorGraph:
template class EliminateableFactorGraph<HybridGaussianFactorGraph>;

using OrphanWrapper = BayesTreeOrphanWrapper<HybridBayesTree::Clique>;

using std::dynamic_pointer_cast;

/* ************************************************************************ */
// Throw a runtime exception for method specified in string s, and factor f:
static void throwRuntimeError(const std::string &s,
                              const std::shared_ptr<Factor> &f) {
  auto &fr = *f;
  throw std::runtime_error(s + " not implemented for factor type " +
                           demangle(typeid(fr).name()) + ".");
}

/* ************************************************************************ */
const Ordering HybridOrdering(const HybridGaussianFactorGraph &graph) {
  KeySet discrete_keys = graph.discreteKeySet();
  const VariableIndex index(graph);
  return Ordering::ColamdConstrainedLast(
      index, KeyVector(discrete_keys.begin(), discrete_keys.end()), true);
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::printErrors(
    const HybridValues &values, const std::string &str,
    const KeyFormatter &keyFormatter,
    const std::function<bool(const Factor * /*factor*/,
                             double /*whitenedError*/, size_t /*index*/)>
        &printCondition) const {
  std::cout << str << "size: " << size() << std::endl << std::endl;

  std::stringstream ss;

  for (size_t i = 0; i < factors_.size(); i++) {
    auto &&factor = factors_[i];
    std::cout << "Factor " << i << ": ";

    // Clear the stringstream
    ss.str(std::string());

    if (auto hgf = std::dynamic_pointer_cast<HybridGaussianFactor>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        hgf->operator()(values.discrete())->print(ss.str(), keyFormatter);
        std::cout << "error = " << factor->error(values) << std::endl;
      }
    } else if (auto hc = std::dynamic_pointer_cast<HybridConditional>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        if (hc->isContinuous()) {
          factor->print(ss.str(), keyFormatter);
          std::cout << "error = " << hc->asGaussian()->error(values) << "\n";
        } else if (hc->isDiscrete()) {
          factor->print(ss.str(), keyFormatter);
          std::cout << "error = " << hc->asDiscrete()->error(values.discrete())
                    << "\n";
        } else {
          // Is hybrid
          auto conditionalComponent =
              hc->asHybrid()->operator()(values.discrete());
          conditionalComponent->print(ss.str(), keyFormatter);
          std::cout << "error = " << conditionalComponent->error(values)
                    << "\n";
        }
      }
    } else if (auto gf = std::dynamic_pointer_cast<GaussianFactor>(factor)) {
      const double errorValue = (factor != nullptr ? gf->error(values) : .0);
      if (!printCondition(factor.get(), errorValue, i))
        continue;  // User-provided filter did not pass

      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = " << errorValue << "\n";
      }
    } else if (auto df = std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      if (factor == nullptr) {
        std::cout << "nullptr"
                  << "\n";
      } else {
        factor->print(ss.str(), keyFormatter);
        std::cout << "error = " << df->error(values.discrete()) << std::endl;
      }

    } else {
      continue;
    }

    std::cout << "\n";
  }
  std::cout.flush();
}

/* ************************************************************************ */
static GaussianFactorGraphTree addGaussian(
    const GaussianFactorGraphTree &gfgTree,
    const GaussianFactor::shared_ptr &factor) {
  // If the decision tree is not initialized, then initialize it.
  if (gfgTree.empty()) {
    GaussianFactorGraph result{factor};
    return GaussianFactorGraphTree(result);
  } else {
    auto add = [&factor](const GaussianFactorGraph &graph) {
      auto result = graph;
      result.push_back(factor);
      return result;
    };
    return gfgTree.apply(add);
  }
}

/* ************************************************************************ */
// TODO(dellaert): it's probably more efficient to first collect the discrete
// keys, and then loop over all assignments to populate a vector.
GaussianFactorGraphTree HybridGaussianFactorGraph::assembleGraphTree() const {
  GaussianFactorGraphTree result;

  for (auto &f : factors_) {
    // TODO(dellaert): just use a virtual method defined in HybridFactor.
    if (auto gf = dynamic_pointer_cast<GaussianFactor>(f)) {
      result = addGaussian(result, gf);
    } else if (auto gmf = dynamic_pointer_cast<HybridGaussianFactor>(f)) {
      result = gmf->add(result);
    } else if (auto gm = dynamic_pointer_cast<HybridGaussianConditional>(f)) {
      result = gm->add(result);
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      if (auto gm = hc->asHybrid()) {
        result = gm->add(result);
      } else if (auto g = hc->asGaussian()) {
        result = addGaussian(result, g);
      } else {
        // Has to be discrete.
        // TODO(dellaert): in C++20, we can use std::visit.
        continue;
      }
    } else if (dynamic_pointer_cast<DiscreteFactor>(f)) {
      // Don't do anything for discrete-only factors
      // since we want to eliminate continuous values only.
      continue;
    } else {
      // TODO(dellaert): there was an unattributed comment here: We need to
      // handle the case where the object is actually an BayesTreeOrphanWrapper!
      throwRuntimeError("gtsam::assembleGraphTree", f);
    }
  }

  return result;
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
continuousElimination(const HybridGaussianFactorGraph &factors,
                      const Ordering &frontalKeys) {
  GaussianFactorGraph gfg;
  for (auto &f : factors) {
    if (auto gf = dynamic_pointer_cast<GaussianFactor>(f)) {
      gfg.push_back(gf);
    } else if (auto orphan = dynamic_pointer_cast<OrphanWrapper>(f)) {
      // Ignore orphaned clique.
      // TODO(dellaert): is this correct? If so explain here.
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      auto gc = hc->asGaussian();
      if (!gc) throwRuntimeError("continuousElimination", gc);
      gfg.push_back(gc);
    } else {
      throwRuntimeError("continuousElimination", f);
    }
  }

  auto result = EliminatePreferCholesky(gfg, frontalKeys);
  return {std::make_shared<HybridConditional>(result.first), result.second};
}

/* ************************************************************************ */
/**
 * @brief Exponentiate (not necessarily normalized) negative log-values,
 * normalize, and then return as AlgebraicDecisionTree<Key>.
 *
 * @param logValues DecisionTree of (unnormalized) log values.
 * @return AlgebraicDecisionTree<Key>
 */
static AlgebraicDecisionTree<Key> probabilitiesFromNegativeLogValues(
    const AlgebraicDecisionTree<Key> &logValues) {
  // Perform normalization
  double min_log = logValues.min();
  AlgebraicDecisionTree<Key> probabilities = DecisionTree<Key, double>(
      logValues, [&min_log](const double x) { return exp(-(x - min_log)); });
  probabilities = probabilities.normalize(probabilities.sum());

  return probabilities;
}

/* ************************************************************************ */
static std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
discreteElimination(const HybridGaussianFactorGraph &factors,
                    const Ordering &frontalKeys) {
  DiscreteFactorGraph dfg;

  for (auto &f : factors) {
    if (auto df = dynamic_pointer_cast<DiscreteFactor>(f)) {
      dfg.push_back(df);
    } else if (auto gmf = dynamic_pointer_cast<HybridGaussianFactor>(f)) {
      // Case where we have a HybridGaussianFactor with no continuous keys.
      // In this case, compute discrete probabilities.
      auto logProbability =
          [&](const GaussianFactor::shared_ptr &factor) -> double {
        if (!factor) return 0.0;
        return factor->error(VectorValues());
      };
      AlgebraicDecisionTree<Key> logProbabilities =
          DecisionTree<Key, double>(gmf->factors(), logProbability);

      AlgebraicDecisionTree<Key> probabilities =
          probabilitiesFromNegativeLogValues(logProbabilities);
      dfg.emplace_shared<DecisionTreeFactor>(gmf->discreteKeys(),
                                             probabilities);

    } else if (auto orphan = dynamic_pointer_cast<OrphanWrapper>(f)) {
      // Ignore orphaned clique.
      // TODO(dellaert): is this correct? If so explain here.
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      auto dc = hc->asDiscrete();
      if (!dc) throwRuntimeError("discreteElimination", dc);
      dfg.push_back(hc->asDiscrete());
    } else {
      throwRuntimeError("discreteElimination", f);
    }
  }

  // NOTE: This does sum-product. For max-product, use EliminateForMPE.
  auto result = EliminateDiscrete(dfg, frontalKeys);

  return {std::make_shared<HybridConditional>(result.first), result.second};
}

/* ************************************************************************ */
// If any GaussianFactorGraph in the decision tree contains a nullptr, convert
// that leaf to an empty GaussianFactorGraph. Needed since the DecisionTree will
// otherwise create a GFG with a single (null) factor,
// which doesn't register as null.
GaussianFactorGraphTree removeEmpty(const GaussianFactorGraphTree &sum) {
  auto emptyGaussian = [](const GaussianFactorGraph &graph) {
    bool hasNull =
        std::any_of(graph.begin(), graph.end(),
                    [](const GaussianFactor::shared_ptr &ptr) { return !ptr; });
    return hasNull ? GaussianFactorGraph() : graph;
  };
  return GaussianFactorGraphTree(sum, emptyGaussian);
}

/* ************************************************************************ */
using Result = std::pair<std::shared_ptr<GaussianConditional>,
                         HybridGaussianFactor::sharedFactor>;

/**
 * Compute the probability p(μ;m) = exp(-error(μ;m)) * sqrt(det(2π Σ_m)
 * from the residual error ||b||^2 at the mean μ.
 * The residual error contains no keys, and only
 * depends on the discrete separator if present.
 */
static std::shared_ptr<Factor> createDiscreteFactor(
    const DecisionTree<Key, Result> &eliminationResults,
    const DiscreteKeys &discreteSeparator) {
  auto negLogProbability = [&](const Result &pair) -> double {
    const auto &[conditional, factor] = pair;
    static const VectorValues kEmpty;
    // If the factor is not null, it has no keys, just contains the residual.
    if (!factor) return 1.0;  // TODO(dellaert): not loving this.

    // Negative logspace version of:
    // exp(-factor->error(kEmpty)) / conditional->normalizationConstant();
    // negLogConstant gives `-log(k)`
    // which is `-log(k) = log(1/k) = log(\sqrt{|2πΣ|})`.
    return factor->error(kEmpty) - conditional->negLogConstant();
  };

  AlgebraicDecisionTree<Key> negLogProbabilities(
      DecisionTree<Key, double>(eliminationResults, negLogProbability));
  AlgebraicDecisionTree<Key> probabilities =
      probabilitiesFromNegativeLogValues(negLogProbabilities);

  return std::make_shared<DecisionTreeFactor>(discreteSeparator, probabilities);
}

// Create HybridGaussianFactor on the separator, taking care to correct
// for conditional constants.
static std::shared_ptr<Factor> createHybridGaussianFactor(
    const DecisionTree<Key, Result> &eliminationResults,
    const DiscreteKeys &discreteSeparator) {
  // Correct for the normalization constant used up by the conditional
  auto correct = [&](const Result &pair) -> GaussianFactorValuePair {
    const auto &[conditional, factor] = pair;
    if (factor) {
      auto hf = std::dynamic_pointer_cast<HessianFactor>(factor);
      if (!hf) throw std::runtime_error("Expected HessianFactor!");
      // Add 2.0 term since the constant term will be premultiplied by 0.5
      // as per the Hessian definition,
      // and negative since we want log(k)
      hf->constantTerm() += -2.0 * conditional->negLogConstant();
    }
    return {factor, 0.0};
  };
  DecisionTree<Key, GaussianFactorValuePair> newFactors(eliminationResults,
                                                        correct);

  return std::make_shared<HybridGaussianFactor>(discreteSeparator, newFactors);
}

static std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
hybridElimination(const HybridGaussianFactorGraph &factors,
                  const Ordering &frontalKeys,
                  const std::set<DiscreteKey> &discreteSeparatorSet) {
  // NOTE: since we use the special JunctionTree,
  // only possibility is continuous conditioned on discrete.
  DiscreteKeys discreteSeparator(discreteSeparatorSet.begin(),
                                 discreteSeparatorSet.end());

  // Collect all the factors to create a set of Gaussian factor graphs in a
  // decision tree indexed by all discrete keys involved.
  GaussianFactorGraphTree factorGraphTree = factors.assembleGraphTree();

  // Convert factor graphs with a nullptr to an empty factor graph.
  // This is done after assembly since it is non-trivial to keep track of which
  // FG has a nullptr as we're looping over the factors.
  factorGraphTree = removeEmpty(factorGraphTree);

  // This is the elimination method on the leaf nodes
  bool someContinuousLeft = false;
  auto eliminate = [&](const GaussianFactorGraph &graph) -> Result {
    if (graph.empty()) {
      return {nullptr, nullptr};
    }

    // Expensive elimination of product factor.
    auto result = EliminatePreferCholesky(graph, frontalKeys);

    // Record whether there any continuous variables left
    someContinuousLeft |= !result.second->empty();

    return result;
  };

  // Perform elimination!
  DecisionTree<Key, Result> eliminationResults(factorGraphTree, eliminate);

  // If there are no more continuous parents we create a DiscreteFactor with the
  // error for each discrete choice. Otherwise, create a HybridGaussianFactor
  // on the separator, taking care to correct for conditional constants.
  auto newFactor =
      someContinuousLeft
          ? createHybridGaussianFactor(eliminationResults, discreteSeparator)
          : createDiscreteFactor(eliminationResults, discreteSeparator);

  // Create the HybridGaussianConditional from the conditionals
  HybridGaussianConditional::Conditionals conditionals(
      eliminationResults, [](const Result &pair) { return pair.first; });
  auto hybridGaussian = std::make_shared<HybridGaussianConditional>(
      discreteSeparator, conditionals);

  return {std::make_shared<HybridConditional>(hybridGaussian), newFactor};
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
std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>  //
EliminateHybrid(const HybridGaussianFactorGraph &factors,
                const Ordering &frontalKeys) {
  // NOTE: Because we are in the Conditional Gaussian regime there are only
  // a few cases:
  // 1. continuous variable, make a hybrid Gaussian conditional if there are
  // hybrid factors;
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

  // The first thing is how to represent the HybridGaussianConditional.
  // A very possible scenario is that the incoming factors will have different
  // levels of discrete keys. For example, imagine we are going to eliminate the
  // fragment: $\phi(x1,c1,c2)$, $\phi(x1,c2,c3)$, which is perfectly valid.
  // Now we will need to know how to retrieve the corresponding continuous
  // densities for the assignment (c1,c2,c3) (OR (c2,c3,c1), note there is NO
  // defined order!). We also need to consider when there is pruning. Two
  // hybrid factors could have different pruning patterns - one could have
  // (c1=0,c2=1) pruned, and another could have (c2=0,c3=1) pruned, and this
  // creates a big problem in how to identify the intersection of non-pruned
  // branches.

  // Our approach is first building the collection of all discrete keys. After
  // that we enumerate the space of all key combinations *lazily* so that the
  // exploration branch terminates whenever an assignment yields NULL in any of
  // the hybrid factors.

  // When the number of assignments is large we may encounter stack overflows.
  // However this is also the case with iSAM2, so no pressure :)

  // Check the factors:
  // 1. if all factors are discrete, then we can do discrete elimination:
  // 2. if all factors are continuous, then we can do continuous elimination:
  // 3. if not, we do hybrid elimination:

  bool only_discrete = true, only_continuous = true;
  for (auto &&factor : factors) {
    if (auto hybrid_factor = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      if (hybrid_factor->isDiscrete()) {
        only_continuous = false;
      } else if (hybrid_factor->isContinuous()) {
        only_discrete = false;
      } else if (hybrid_factor->isHybrid()) {
        only_continuous = false;
        only_discrete = false;
      }
    } else if (auto cont_factor =
                   std::dynamic_pointer_cast<GaussianFactor>(factor)) {
      only_discrete = false;
    } else if (auto discrete_factor =
                   std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      only_continuous = false;
    }
  }

  // NOTE: We should really defer the product here because of pruning

  if (only_discrete) {
    // Case 1: we are only dealing with discrete
    return discreteElimination(factors, frontalKeys);
  } else if (only_continuous) {
    // Case 2: we are only dealing with continuous
    return continuousElimination(factors, frontalKeys);
  } else {
    // Case 3: We are now in the hybrid land!
    KeySet frontalKeysSet(frontalKeys.begin(), frontalKeys.end());

    // Find all discrete keys.
    // Since we eliminate all continuous variables first,
    // the discrete separator will be *all* the discrete keys.
    std::set<DiscreteKey> discreteSeparator = factors.discreteKeys();

    return hybridElimination(factors, frontalKeys, discreteSeparator);
  }
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::errorTree(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> error_tree(0.0);
  // Iterate over each factor.
  for (auto &factor : factors_) {
    if (auto f = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      // Check for HybridFactor, and call errorTree
      error_tree = error_tree + f->errorTree(continuousValues);
    } else if (auto f = std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      // Skip discrete factors
      continue;
    } else {
      // Everything else is a continuous only factor
      HybridValues hv(continuousValues, DiscreteValues());
      error_tree = error_tree + AlgebraicDecisionTree<Key>(factor->error(hv));
    }
  }
  return error_tree;
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
  AlgebraicDecisionTree<Key> error_tree = this->errorTree(continuousValues);
  AlgebraicDecisionTree<Key> prob_tree = error_tree.apply([](double error) {
    // NOTE: The 0.5 term is handled by each factor
    return exp(-error);
  });
  return prob_tree;
}

/* ************************************************************************ */
GaussianFactorGraph HybridGaussianFactorGraph::operator()(
    const DiscreteValues &assignment) const {
  GaussianFactorGraph gfg;
  for (auto &&f : *this) {
    if (auto gf = std::dynamic_pointer_cast<GaussianFactor>(f)) {
      gfg.push_back(gf);
    } else if (auto gc = std::dynamic_pointer_cast<GaussianConditional>(f)) {
      gfg.push_back(gf);
    } else if (auto hgf = std::dynamic_pointer_cast<HybridGaussianFactor>(f)) {
      gfg.push_back((*hgf)(assignment));
    } else if (auto hgc = dynamic_pointer_cast<HybridGaussianConditional>(f)) {
      gfg.push_back((*hgc)(assignment));
    } else {
      continue;
    }
  }
  return gfg;
}

}  // namespace gtsam

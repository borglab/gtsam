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
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteValues.h>
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

#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "gtsam/discrete/DecisionTreeFactor.h"

namespace gtsam {

/// Specialize EliminateableFactorGraph for HybridGaussianFactorGraph:
template class EliminateableFactorGraph<HybridGaussianFactorGraph>;

using std::dynamic_pointer_cast;
using OrphanWrapper = BayesTreeOrphanWrapper<HybridBayesTree::Clique>;
using Result =
    std::pair<std::shared_ptr<GaussianConditional>, GaussianFactor::shared_ptr>;
using ResultValuePair = std::pair<Result, double>;
using ResultTree = DecisionTree<Key, ResultValuePair>;

static const VectorValues kEmpty;

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
static void printFactor(const std::shared_ptr<Factor> &factor,
                        const DiscreteValues &assignment,
                        const KeyFormatter &keyFormatter) {
  if (auto hgf = dynamic_pointer_cast<HybridGaussianFactor>(factor)) {
    if (assignment.empty()) {
      hgf->print("HybridGaussianFactor:", keyFormatter);
    } else {
      hgf->operator()(assignment)
          .first->print("HybridGaussianFactor, component:", keyFormatter);
    }
  } else if (auto gf = dynamic_pointer_cast<GaussianFactor>(factor)) {
    factor->print("GaussianFactor:\n", keyFormatter);

  } else if (auto df = dynamic_pointer_cast<DiscreteFactor>(factor)) {
    factor->print("DiscreteFactor:\n", keyFormatter);
  } else if (auto hc = dynamic_pointer_cast<HybridConditional>(factor)) {
    if (hc->isContinuous()) {
      factor->print("GaussianConditional:\n", keyFormatter);
    } else if (hc->isDiscrete()) {
      factor->print("DiscreteConditional:\n", keyFormatter);
    } else {
      if (assignment.empty()) {
        hc->print("HybridConditional:", keyFormatter);
      } else {
        hc->asHybrid()
            ->choose(assignment)
            ->print("HybridConditional, component:\n", keyFormatter);
      }
    }
  } else {
    factor->print("Unknown factor type\n", keyFormatter);
  }
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::print(const std::string &s,
                                      const KeyFormatter &keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ") << std::endl;
  std::cout << "size: " << size() << std::endl;

  for (size_t i = 0; i < factors_.size(); i++) {
    auto &&factor = factors_[i];
    if (factor == nullptr) {
      std::cout << "Factor " << i << ": nullptr\n";
      continue;
    }
    // Print the factor
    std::cout << "Factor " << i << "\n";
    printFactor(factor, {}, keyFormatter);
    std::cout << "\n";
  }
  std::cout.flush();
}

/* ************************************************************************ */
void HybridGaussianFactorGraph::printErrors(
    const HybridValues &values, const std::string &str,
    const KeyFormatter &keyFormatter,
    const std::function<bool(const Factor * /*factor*/,
                             double /*whitenedError*/, size_t /*index*/)>
        &printCondition) const {
  std::cout << str << "size: " << size() << std::endl << std::endl;

  for (size_t i = 0; i < factors_.size(); i++) {
    auto &&factor = factors_[i];
    if (factor == nullptr) {
      std::cout << "Factor " << i << ": nullptr\n";
      continue;
    }
    const double errorValue = factor->error(values);
    if (!printCondition(factor.get(), errorValue, i))
      continue;  // User-provided filter did not pass

    // Print the factor
    std::cout << "Factor " << i << ", error = " << errorValue << "\n";
    printFactor(factor, values.discrete(), keyFormatter);
    std::cout << "\n";
  }
  std::cout.flush();
}

/* ************************************************************************ */
HybridGaussianProductFactor HybridGaussianFactorGraph::collectProductFactor()
    const {
  HybridGaussianProductFactor result;

  for (auto &f : factors_) {
    // TODO(dellaert): can we make this cleaner and less error-prone?
    if (auto orphan = dynamic_pointer_cast<OrphanWrapper>(f)) {
      continue;  // Ignore OrphanWrapper
    } else if (auto gf = dynamic_pointer_cast<GaussianFactor>(f)) {
      result += gf;
    } else if (auto gc = dynamic_pointer_cast<GaussianConditional>(f)) {
      result += gc;
    } else if (auto gmf = dynamic_pointer_cast<HybridGaussianFactor>(f)) {
      result += *gmf;
    } else if (auto gm = dynamic_pointer_cast<HybridGaussianConditional>(f)) {
      result += *gm;  // handled above already?
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      if (auto gm = hc->asHybrid()) {
        result += *gm;
      } else if (auto g = hc->asGaussian()) {
        result += g;
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
      throwRuntimeError("gtsam::collectProductFactor", f);
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
 * @brief Take negative log-values, shift them so that the minimum value is 0,
 * and then exponentiate to create a DecisionTreeFactor (not normalized yet!).
 *
 * @param errors DecisionTree of (unnormalized) errors.
 * @return DecisionTreeFactor::shared_ptr
 */
static DecisionTreeFactor::shared_ptr DiscreteFactorFromErrors(
    const DiscreteKeys &discreteKeys,
    const AlgebraicDecisionTree<Key> &errors) {
  double min_log = errors.min();
  AlgebraicDecisionTree<Key> potentials(
      errors, [&min_log](const double x) { return exp(-(x - min_log)); });
  return std::make_shared<DecisionTreeFactor>(discreteKeys, potentials);
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
      // In this case, compute a discrete factor from the remaining error.
      auto calculateError = [&](const auto &pair) -> double {
        auto [factor, scalar] = pair;
        // If factor is null, it has been pruned, hence return infinite error
        if (!factor) return std::numeric_limits<double>::infinity();
        return scalar + factor->error(kEmpty);
      };
      AlgebraicDecisionTree<Key> errors(gmf->factors(), calculateError);
      dfg.push_back(DiscreteFactorFromErrors(gmf->discreteKeys(), errors));

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
/**
 * Compute the probability p(μ;m) = exp(-error(μ;m)) * sqrt(det(2π Σ_m)
 * from the residual error ||b||^2 at the mean μ.
 * The residual error contains no keys, and only
 * depends on the discrete separator if present.
 */
static std::shared_ptr<Factor> createDiscreteFactor(
    const ResultTree &eliminationResults,
    const DiscreteKeys &discreteSeparator) {
  auto calculateError = [&](const auto &pair) -> double {
    const auto &[conditional, factor] = pair.first;
    const double scalar = pair.second;
    if (conditional && factor) {
      // `error` has the following contributions:
      // - the scalar is the sum of all mode-dependent constants
      // - factor->error(kempty) is the error remaining after elimination
      // - negLogK is what is given to the conditional to normalize
      const double negLogK = conditional->negLogConstant();
      return scalar + factor->error(kEmpty) - negLogK;
    } else if (!conditional && !factor) {
      // If the factor has been pruned, return infinite error
      return std::numeric_limits<double>::infinity();
    } else {
      throw std::runtime_error("createDiscreteFactor has mixed NULLs");
    }
  };

  AlgebraicDecisionTree<Key> errors(eliminationResults, calculateError);
  return DiscreteFactorFromErrors(discreteSeparator, errors);
}

/* *******************************************************************************/
// Create HybridGaussianFactor on the separator, taking care to correct
// for conditional constants.
static std::shared_ptr<Factor> createHybridGaussianFactor(
    const ResultTree &eliminationResults,
    const DiscreteKeys &discreteSeparator) {
  // Correct for the normalization constant used up by the conditional
  auto correct = [&](const ResultValuePair &pair) -> GaussianFactorValuePair {
    const auto &[conditional, factor] = pair.first;
    const double scalar = pair.second;
    if (conditional && factor) {
      const double negLogK = conditional->negLogConstant();
      return {factor, scalar - negLogK};
    } else if (!conditional && !factor) {
      return {nullptr, std::numeric_limits<double>::infinity()};
    } else {
      throw std::runtime_error("createHybridGaussianFactors has mixed NULLs");
    }
  };
  DecisionTree<Key, GaussianFactorValuePair> newFactors(eliminationResults,
                                                        correct);

  return std::make_shared<HybridGaussianFactor>(discreteSeparator, newFactors);
}

/* *******************************************************************************/
/// Get the discrete keys from the HybridGaussianFactorGraph as DiscreteKeys.
static auto GetDiscreteKeys =
    [](const HybridGaussianFactorGraph &hfg) -> DiscreteKeys {
  const std::set<DiscreteKey> discreteKeySet = hfg.discreteKeys();
  return {discreteKeySet.begin(), discreteKeySet.end()};
};

/* *******************************************************************************/
std::pair<HybridConditional::shared_ptr, std::shared_ptr<Factor>>
HybridGaussianFactorGraph::eliminate(const Ordering &keys) const {
  // Since we eliminate all continuous variables first,
  // the discrete separator will be *all* the discrete keys.
  DiscreteKeys discreteSeparator = GetDiscreteKeys(*this);

  // Collect all the factors to create a set of Gaussian factor graphs in a
  // decision tree indexed by all discrete keys involved. Just like any hybrid
  // factor, every assignment also has a scalar error, in this case the sum of
  // all errors in the graph. This error is assignment-specific and accounts for
  // any difference in noise models used.
  HybridGaussianProductFactor productFactor = collectProductFactor();

  // Convert factor graphs with a nullptr to an empty factor graph.
  // This is done after assembly since it is non-trivial to keep track of which
  // FG has a nullptr as we're looping over the factors.
  auto prunedProductFactor = productFactor.removeEmpty();

  // This is the elimination method on the leaf nodes
  bool someContinuousLeft = false;
  auto eliminate = [&](const std::pair<GaussianFactorGraph, double> &pair)
      -> std::pair<Result, double> {
    const auto &[graph, scalar] = pair;

    if (graph.empty()) {
      return {{nullptr, nullptr}, 0.0};
    }

    // Expensive elimination of product factor.
    auto result =
        EliminatePreferCholesky(graph, keys);  /// <<<<<< MOST COMPUTE IS HERE

    // Record whether there any continuous variables left
    someContinuousLeft |= !result.second->empty();

    // We pass on the scalar unmodified.
    return {result, scalar};
  };

  // Perform elimination!
  ResultTree eliminationResults(prunedProductFactor, eliminate);

  // If there are no more continuous parents we create a DiscreteFactor with the
  // error for each discrete choice. Otherwise, create a HybridGaussianFactor
  // on the separator, taking care to correct for conditional constants.
  auto newFactor =
      someContinuousLeft
          ? createHybridGaussianFactor(eliminationResults, discreteSeparator)
          : createDiscreteFactor(eliminationResults, discreteSeparator);

  // Create the HybridGaussianConditional from the conditionals
  HybridGaussianConditional::Conditionals conditionals(
      eliminationResults,
      [](const ResultValuePair &pair) { return pair.first.first; });
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
                const Ordering &keys) {
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
        break;
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
    return discreteElimination(factors, keys);
  } else if (only_continuous) {
    // Case 2: we are only dealing with continuous
    return continuousElimination(factors, keys);
  } else {
    // Case 3: We are now in the hybrid land!
    return factors.eliminate(keys);
  }
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::errorTree(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> result(0.0);
  // Iterate over each factor.
  for (auto &factor : factors_) {
    if (auto hf = std::dynamic_pointer_cast<HybridFactor>(factor)) {
      // Add errorTree for hybrid factors, includes HybridGaussianConditionals!
      result = result + hf->errorTree(continuousValues);
    } else if (auto df = std::dynamic_pointer_cast<DiscreteFactor>(factor)) {
      // If discrete, just add its errorTree as well
      result = result + df->errorTree();
    } else if (auto gf = dynamic_pointer_cast<GaussianFactor>(factor)) {
      // For a continuous only factor, just add its error
      result = result + gf->error(continuousValues);
    } else {
      throwRuntimeError("HybridGaussianFactorGraph::errorTree", factor);
    }
  }
  return result;
}

/* ************************************************************************ */
double HybridGaussianFactorGraph::probPrime(const HybridValues &values) const {
  double error = this->error(values);
  // NOTE: The 0.5 term is handled by each factor
  return std::exp(-error);
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> HybridGaussianFactorGraph::discretePosterior(
    const VectorValues &continuousValues) const {
  AlgebraicDecisionTree<Key> errors = this->errorTree(continuousValues);
  AlgebraicDecisionTree<Key> p = errors.apply([](double error) {
    // NOTE: The 0.5 term is handled by each factor
    return exp(-error);
  });
  return p / p.sum();
}

/* ************************************************************************ */
GaussianFactorGraph HybridGaussianFactorGraph::choose(
    const DiscreteValues &assignment) const {
  GaussianFactorGraph gfg;
  for (auto &&f : *this) {
    if (auto gf = std::dynamic_pointer_cast<GaussianFactor>(f)) {
      gfg.push_back(gf);
    } else if (auto gc = std::dynamic_pointer_cast<GaussianConditional>(f)) {
      gfg.push_back(gf);
    } else if (auto hgf = std::dynamic_pointer_cast<HybridGaussianFactor>(f)) {
      gfg.push_back((*hgf)(assignment).first);
    } else if (auto hgc = dynamic_pointer_cast<HybridGaussianConditional>(f)) {
      gfg.push_back((*hgc)(assignment));
    } else if (auto hc = dynamic_pointer_cast<HybridConditional>(f)) {
      if (auto gc = hc->asGaussian())
        gfg.push_back(gc);
      else if (auto hgc = hc->asHybrid())
        gfg.push_back((*hgc)(assignment));
    } else {
      continue;
    }
  }
  return gfg;
}

}  // namespace gtsam

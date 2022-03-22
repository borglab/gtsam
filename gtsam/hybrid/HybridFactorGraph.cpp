/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridFactorGraph.cpp
 * @brief  Hybrid factor graph that uses type erasure
 * @author Fan Jiang
 * @date   Mar 11, 2022
 */

#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>

#include <iostream>
#include <unordered_map>
#include <utility>

#include "gtsam/inference/Key.h"
#include "gtsam/linear/GaussianFactorGraph.h"
#include "gtsam/linear/HessianFactor.h"

namespace gtsam {

template class EliminateableFactorGraph<HybridFactorGraph>;

static std::string BLACK_BOLD = "\033[1;30m";
static std::string RED_BOLD = "\033[1;31m";
static std::string GREEN = "\033[0;32m";
static std::string GREEN_BOLD = "\033[1;32m";
static std::string RESET = "\033[0m";

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
EliminateHybrid(const HybridFactorGraph &factors, const Ordering &frontalKeys) {
  // NOTE(fan): Because we are in the Conditional Gaussian regime there are only
  // a few cases:
  // continuous variable, we make a GM if there are hybrid factors;
  // continuous variable, we make a GF if there are no hybrid factors;
  // discrete variable, no continuous factor is allowed (escapes CG regime), so
  // we panic, if discrete only we do the discrete elimination.

  // However it is not that simple. During elimination it is possible that the
  // multifrontal needs to eliminate an ordering that contains both Gaussian and
  // hybrid variables, for example x1, c1. In this scenario, we will have a
  // density P(x1, c1) that is a CLG P(x1|c1)P(c1) (see Murphy02)

  // The issue here is that, how can we know which variable is discrete if we
  // unify Values? Obviously we can tell using the factors, but is that fast?

  // In the case of multifrontal, we will need to use a constrained ordering
  // so that the discrete parts will be guaranteed to be eliminated last!

  // Because of all these reasons, we need to think very carefully about how to
  // implement the hybrid factors so that we do not get poor performance.
  // 
  // The first thing is how to represent the GaussianMixture. A very possible
  // scenario is that the incoming factors will have different levels of discrete
  // keys. For example, imagine we are going to eliminate the fragment:
  // $\phi(x1,c1,c2)$, $\phi(x1,c2,c3)$, which is perfectly valid. Now we will need
  // to know how to retrieve the corresponding continuous densities for the assi-
  // -gnment (c1,c2,c3) (OR (c2,c3,c1)! note there is NO defined order!). And we 
  // also need to consider when there is pruning. Two mixture factors could have
  // different pruning patterns-one could have (c1=0,c2=1) pruned, and another
  // could have (c2=0,c3=1) pruned, and this creates a big problem in how to 
  // identify the intersection of non-pruned branches.

  // One possible approach is first building the collection of all discrete keys.
  // After that we enumerate the space of all key combinations *lazily* so that
  // the exploration branch terminates whenever an assignment yields NULL in any
  // of the hybrid factors.

  // When the number of assignments is large we may encounter stack overflows.
  // However this is also the case with iSAM2, so no pressure :)

  // PREPROCESS: Identify the nature of the current elimination
  std::unordered_map<Key, DiscreteKey> discreteCardinalities;
  std::set<DiscreteKey> discreteSeparator;
  std::set<DiscreteKey> discreteFrontals;

  KeySet separatorKeys;
  KeySet allContinuousKeys;
  KeySet continuousFrontals;
  KeySet continuousSeparator;

  // TODO: we do a mock by just doing the correct key thing
  std::cout << RED_BOLD << "Begin Eliminate: " << RESET;
  frontalKeys.print();

  // This initializes separatorKeys and discreteCardinalities
  for (auto &&factor : factors) {
    std::cout << ">>> Adding factor: " << GREEN;
    factor->print();
    std::cout << RESET;
    separatorKeys.insert(factor->begin(), factor->end());
    if (!factor->isContinuous_) {
      for (auto &k : factor->discreteKeys_) {
        discreteCardinalities[k.first] = k;
      }
    }
  }

  // remove frontals from separator
  for (auto &k : frontalKeys) {
    separatorKeys.erase(k);
  }

  // Fill in discrete frontals and continuous frontals for the end result
  for (auto &k : frontalKeys) {
    if (discreteCardinalities.find(k) != discreteCardinalities.end()) {
      discreteFrontals.insert(discreteCardinalities.at(k));
    } else {
      continuousFrontals.insert(k);
    }
  }

  // Fill in discrete frontals and continuous frontals for the end result
  for (auto &k : separatorKeys) {
    if (discreteCardinalities.find(k) != discreteCardinalities.end()) {
      discreteSeparator.insert(discreteCardinalities.at(k));
    } else {
      continuousSeparator.insert(k);
    }
  }

  {
    std::cout << RED_BOLD << "Keys: " << RESET;
    for (auto &f : frontalKeys) {
      if (discreteCardinalities.find(f) != discreteCardinalities.end()) {
        auto &key = discreteCardinalities.at(f);
        std::cout << boost::format(" (%1%,%2%),") %
                         DefaultKeyFormatter(key.first) % key.second;
      } else {
        std::cout << " " << DefaultKeyFormatter(f) << ",";
      }
    }

    if (separatorKeys.size() > 0) {
      std::cout << " | ";
    }

    for (auto &f : separatorKeys) {
      if (discreteCardinalities.find(f) != discreteCardinalities.end()) {
        auto &key = discreteCardinalities.at(f);
        std::cout << boost::format(" (%1%,%2%),") %
                         DefaultKeyFormatter(key.first) % key.second;
      } else {
        std::cout << DefaultKeyFormatter(f) << ",";
      }
    }
    std::cout << "\n" << RESET;
  }

  // NOTE: We should really defer the product here because of pruning
  
  // Case 1: we are only dealing with continuous
  if (discreteCardinalities.empty()) {
    GaussianFactorGraph gfg;
    for (auto &fp : factors) {
      gfg.push_back(boost::static_pointer_cast<HybridGaussianFactor>(fp)->inner);
    }

    auto result = EliminatePreferCholesky(gfg, frontalKeys);
    return std::make_pair(
        boost::make_shared<HybridConditional>(result.first),
        boost::make_shared<HybridGaussianFactor>(result.second));
  }

  // Case 2: we are only dealing with discrete
  if (discreteCardinalities.empty()) {
    GaussianFactorGraph gfg;
    for (auto &fp : factors) {
      gfg.push_back(boost::static_pointer_cast<HybridGaussianFactor>(fp)->inner);
    }

    auto result = EliminatePreferCholesky(gfg, frontalKeys);
    return std::make_pair(
        boost::make_shared<HybridConditional>(result.first),
        boost::make_shared<HybridGaussianFactor>(result.second));
  }

  // PRODUCT: multiply all factors
  gttic(product);

  HybridConditional sum(
      KeyVector(continuousSeparator.begin(), continuousSeparator.end()),
      DiscreteKeys(discreteSeparator.begin(), discreteSeparator.end()),
      separatorKeys.size());

  //  HybridDiscreteFactor product(DiscreteConditional());
  //  for (auto&& factor : factors) product = (*factor) * product;
  gttoc(product);

  // sum out frontals, this is the factor on the separator
  gttic(sum);
  //  HybridFactor::shared_ptr sum = product.sum(frontalKeys);
  gttoc(sum);

  // Ordering keys for the conditional so that frontalKeys are really in front
  Ordering orderedKeys;
  orderedKeys.insert(orderedKeys.end(), frontalKeys.begin(), frontalKeys.end());
  orderedKeys.insert(orderedKeys.end(), sum.keys().begin(), sum.keys().end());

  // now divide product/sum to get conditional
  gttic(divide);
  //  auto conditional =
  //      boost::make_shared<HybridConditional>(product, *sum, orderedKeys);
  gttoc(divide);

  auto conditional = boost::make_shared<HybridConditional>(
      CollectKeys({continuousFrontals.begin(), continuousFrontals.end()},
                  {continuousSeparator.begin(), continuousSeparator.end()}),
      CollectDiscreteKeys({discreteFrontals.begin(), discreteFrontals.end()},
                          {discreteSeparator.begin(), discreteSeparator.end()}),
      continuousFrontals.size() + discreteFrontals.size());
  std::cout << GREEN_BOLD << "[Conditional]\n" << RESET;
  conditional->print();
  std::cout << GREEN_BOLD << "[Separator]\n" << RESET;
  sum.print();
  std::cout << RED_BOLD << "[End Eliminate]\n" << RESET;

  //  return std::make_pair(conditional, sum);
  return std::make_pair(conditional,
                        boost::make_shared<HybridConditional>(std::move(sum)));
}

void HybridFactorGraph::add(JacobianFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(std::move(factor)));
}
}  // namespace gtsam

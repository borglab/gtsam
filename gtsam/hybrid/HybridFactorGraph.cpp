//
// Created by Fan Jiang on 3/11/22.
//

#include "gtsam/inference/Key.h"
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridFactor.h>

#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>

#include <gtsam/inference/EliminateableFactorGraph-inst.h>

#include <iostream>
#include <unordered_map>

namespace gtsam {

template
class EliminateableFactorGraph<HybridFactorGraph>;

static std::string BLACK_BOLD = "\033[1;30m";
static std::string RED_BOLD = "\033[1;31m";
static std::string GREEN = "\033[0;32m";
static std::string GREEN_BOLD = "\033[1;32m";
static std::string RESET = "\033[0m";

/* ************************************************************************ */
std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
EliminateHybrid(const HybridFactorGraph &factors,
                const Ordering &frontalKeys) {
  // NOTE(fan): Because we are in the Conditional Gaussian regime there are only
  // a few cases:
  // continuous variable, we make a GM if there are hybrid factors;
  // continuous variable, we make a GF if there are no hybrid factors;
  // discrete variable, no continuous factor is allowed (escapes CG regime), so
  // we panic, if discrete only we do the discrete elimination.

  // However it is not that simple. During elimination it is possible that the multifrontal needs
  // to eliminate an ordering that contains both Gaussian and hybrid variables, for example x1, c1.
  // In this scenario, we will have a density P(x1, c1) that is a CLG P(x1|c1)P(c1) (see Murphy02)

  // The issue here is that, how can we know which variable is discrete if we
  // unify Values? Obviously we can tell using the factors, but is that fast?

  // In the case of multifrontal, we will need to use a constrained ordering
  // so that the discrete parts will be guaranteed to be eliminated last!

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

  std::cout << RED_BOLD << "Keys: " << RESET;
  for (auto &f : frontalKeys) {
    if (discreteCardinalities.find(f) != discreteCardinalities.end()) {
      auto &key = discreteCardinalities.at(f);
      std::cout << boost::format(" (%1%,%2%),") % DefaultKeyFormatter(key.first) % key.second;
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
      std::cout << boost::format(" (%1%,%2%),") % DefaultKeyFormatter(key.first) % key.second;
    } else {
      std::cout << DefaultKeyFormatter(f) << ",";
    }
  }
  std::cout << "\n" << RESET;
  // PRODUCT: multiply all factors
  gttic(product);

  HybridConditional sum(KeyVector(continuousSeparator.begin(), continuousSeparator.end()),
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
  orderedKeys.insert(orderedKeys.end(), frontalKeys.begin(),
                     frontalKeys.end());
  orderedKeys.insert(orderedKeys.end(), sum.keys().begin(),
                     sum.keys().end());

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
  return std::make_pair(
      conditional,
      boost::make_shared<HybridConditional>(std::move(sum)));
}

void HybridFactorGraph::add(JacobianFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(std::move(factor)));
}
}

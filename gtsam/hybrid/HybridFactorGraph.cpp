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
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 11, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/Assignment.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridEliminationTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridJunctionTree.h>
#include <gtsam/inference/EliminateableFactorGraph-inst.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace gtsam {

template class EliminateableFactorGraph<HybridFactorGraph>;

static std::string BLACK_BOLD = "\033[1;30m";
static std::string RED_BOLD = "\033[1;31m";
static std::string GREEN = "\033[0;32m";
static std::string GREEN_BOLD = "\033[1;32m";
static std::string RESET = "\033[0m";

static bool DEBUG = false;

static GaussianMixtureFactor::Sum &addGaussian(
    GaussianMixtureFactor::Sum &sum, const GaussianFactor::shared_ptr &factor) {
  using Y = GaussianFactorGraph;
  // If the decision tree is not intiialized, then intialize it.
  if (sum.empty()) {
    GaussianFactorGraph result;
    result.push_back(factor);
    sum = GaussianMixtureFactor::Sum(result);

  } else {
    auto add = [&factor](const Y &graph) {
      auto result = graph;
      result.push_back(factor);
      return result;
    };
    sum = sum.apply(add);
  }
  return sum;
}

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
  // scenario is that the incoming factors will have different levels of
  // discrete keys. For example, imagine we are going to eliminate the fragment:
  // $\phi(x1,c1,c2)$, $\phi(x1,c2,c3)$, which is perfectly valid. Now we will
  // need to know how to retrieve the corresponding continuous densities for the
  // assi- -gnment (c1,c2,c3) (OR (c2,c3,c1)! note there is NO defined order!).
  // And we also need to consider when there is pruning. Two mixture factors
  // could have different pruning patterns-one could have (c1=0,c2=1) pruned,
  // and another could have (c2=0,c3=1) pruned, and this creates a big problem
  // in how to identify the intersection of non-pruned branches.

  // One possible approach is first building the collection of all discrete
  // keys. After that we enumerate the space of all key combinations *lazily* so
  // that the exploration branch terminates whenever an assignment yields NULL
  // in any of the hybrid factors.

  // When the number of assignments is large we may encounter stack overflows.
  // However this is also the case with iSAM2, so no pressure :)

  // PREPROCESS: Identify the nature of the current elimination
  std::unordered_map<Key, DiscreteKey> discreteCardinalities;
  std::set<DiscreteKey> discreteSeparatorSet;
  std::set<DiscreteKey> discreteFrontals;

  KeySet separatorKeys;
  KeySet allContinuousKeys;
  KeySet continuousFrontals;
  KeySet continuousSeparator;

  if (DEBUG) {
    std::cout << RED_BOLD << "Begin Eliminate: " << RESET;
    frontalKeys.print();
  }

  // This initializes separatorKeys and discreteCardinalities
  for (auto &&factor : factors) {
    if (DEBUG) {
      std::cout << ">>> Adding factor: " << GREEN;
      factor->print();
      std::cout << RESET;
    }
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
      allContinuousKeys.insert(k);
    }
  }

  // Fill in discrete frontals and continuous frontals for the end result
  for (auto &k : separatorKeys) {
    if (discreteCardinalities.find(k) != discreteCardinalities.end()) {
      discreteSeparatorSet.insert(discreteCardinalities.at(k));
    } else {
      continuousSeparator.insert(k);
      allContinuousKeys.insert(k);
    }
  }

  // Only for printing
  if (DEBUG) {
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
  if (discreteCardinalities.empty() && !allContinuousKeys.empty()) {
    if (DEBUG) {
      std::cout << RED_BOLD << "CONT. ONLY" << RESET << "\n";
    }

    GaussianFactorGraph gfg;
    for (auto &fp : factors) {
      auto ptr = boost::dynamic_pointer_cast<HybridGaussianFactor>(fp);
      if (ptr)
        gfg.push_back(ptr->inner);
      else
        gfg.push_back(boost::static_pointer_cast<GaussianConditional>(
            boost::static_pointer_cast<HybridConditional>(fp)->inner));
    }

    auto result = EliminatePreferCholesky(gfg, frontalKeys);
    return std::make_pair(
        boost::make_shared<HybridConditional>(result.first),
        boost::make_shared<HybridGaussianFactor>(result.second));
  }

  // Case 2: we are only dealing with discrete
  if (allContinuousKeys.empty()) {
    if (DEBUG) {
      std::cout << RED_BOLD << "DISCRETE ONLY" << RESET << "\n";
    }

    DiscreteFactorGraph dfg;
    for (auto &fp : factors) {
      auto ptr = boost::dynamic_pointer_cast<HybridDiscreteFactor>(fp);
      if (ptr)
        dfg.push_back(ptr->inner);
      else
        dfg.push_back(boost::static_pointer_cast<DiscreteConditional>(
            boost::static_pointer_cast<HybridConditional>(fp)->inner));
    }

    auto result = EliminateDiscrete(dfg, frontalKeys);

    return std::make_pair(
        boost::make_shared<HybridConditional>(result.first),
        boost::make_shared<HybridDiscreteFactor>(result.second));
  }

  // Case 3: We are now in the hybrid land!
  // NOTE: since we use the special JunctionTree, only possiblity is cont.
  // conditioned on disc.
  DiscreteKeys discreteSeparator(discreteSeparatorSet.begin(),
                                 discreteSeparatorSet.end());

  // sum out frontals, this is the factor on the separator
  gttic(sum);

  if (DEBUG) {
    std::cout << RED_BOLD << "HYBRID ELIM." << RESET << "\n";
  }

  GaussianMixtureFactor::Sum sum;

  std::vector<GaussianFactor::shared_ptr> deferredFactors;

  for (auto &f : factors) {
    if (f->isHybrid_) {
      auto cgmf = boost::dynamic_pointer_cast<GaussianMixtureFactor>(f);
      if (cgmf) {
        sum = cgmf->addTo(sum);
      }

      auto gm = boost::dynamic_pointer_cast<HybridConditional>(f);
      if (gm) {
        sum = gm->asMixture()->addTo(sum);
      }

    } else if (f->isContinuous_) {
      deferredFactors.push_back(
          boost::dynamic_pointer_cast<HybridGaussianFactor>(f)->inner);
    } else {
      throw std::invalid_argument(
          "factor is discrete in continuous elimination");
    }
  }

  for (auto &f : deferredFactors) {
    if (DEBUG) {
      std::cout << GREEN_BOLD << "Adding Gaussian" << RESET << "\n";
    }
    sum = addGaussian(sum, f);
  }

  if (DEBUG) {
    std::cout << GREEN_BOLD << "[GFG Tree]\n" << RESET;
    sum.print("", DefaultKeyFormatter, [](GaussianFactorGraph gfg) {
      RedirectCout rd;
      gfg.print("");
      return rd.str();
    });
  }

  gttoc(sum);

  using EliminationPair = GaussianFactorGraph::EliminationResult;

  KeyVector keysOfEliminated;  // Not the ordering
  KeyVector keysOfSeparator;   // TODO(frank): Is this just (keys - ordering)?

  auto eliminate = [&](const GaussianFactorGraph &graph)
      -> GaussianFactorGraph::EliminationResult {
    if (graph.empty()) {
      return {nullptr, nullptr};
    }
    auto result = EliminatePreferCholesky(graph, frontalKeys);
    if (keysOfEliminated.empty()) {
      keysOfEliminated =
          result.first->keys();  // Initialize the keysOfEliminated to be the
    }
    // keysOfEliminated of the GaussianConditional
    if (keysOfSeparator.empty()) {
      keysOfSeparator = result.second->keys();
    }
    return result;
  };

  DecisionTree<Key, EliminationPair> eliminationResults(sum, eliminate);

  auto pair = unzip(eliminationResults);

  const GaussianMixture::Conditionals &conditionals = pair.first;
  const GaussianMixtureFactor::Factors &separatorFactors = pair.second;

  // Create the GaussianMixture from the conditionals
  auto conditional = boost::make_shared<GaussianMixture>(
      frontalKeys, keysOfSeparator, discreteSeparator, conditionals);

  if (DEBUG) {
    std::cout << GREEN_BOLD << "[Conditional]\n" << RESET;
    conditional->print();
    std::cout << GREEN_BOLD << "[Separator]\n" << RESET;
    separatorFactors.print("", DefaultKeyFormatter,
                           [](GaussianFactor::shared_ptr gc) {
                             RedirectCout rd;
                             gc->print("");
                             return rd.str();
                           });
    std::cout << RED_BOLD << "[End Eliminate]\n" << RESET;
  }

  // If there are no more continuous parents, then we should create here a
  // DiscreteFactor, with the error for each discrete choice.
  if (keysOfSeparator.empty()) {
    VectorValues empty_values;
    auto factorError = [&](const GaussianFactor::shared_ptr &factor) {
      if (!factor) return 0.0;  // TODO(fan): does this make sense?
      return exp(-factor->error(empty_values));
    };
    DecisionTree<Key, double> fdt(separatorFactors, factorError);
    auto discreteFactor =
        boost::make_shared<DecisionTreeFactor>(discreteSeparator, fdt);

    return {boost::make_shared<HybridConditional>(conditional),
            boost::make_shared<HybridDiscreteFactor>(discreteFactor)};

  } else {
    // Create a resulting DCGaussianMixture on the separator.
    auto factor = boost::make_shared<GaussianMixtureFactor>(
        frontalKeys, discreteSeparator, separatorFactors);
    return {boost::make_shared<HybridConditional>(conditional), factor};
  }
}

void HybridFactorGraph::add(JacobianFactor &&factor) {
  FactorGraph::add(boost::make_shared<HybridGaussianFactor>(std::move(factor)));
}
}  // namespace gtsam

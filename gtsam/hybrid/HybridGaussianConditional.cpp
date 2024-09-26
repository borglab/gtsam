/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianConditional.cpp
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianConditional.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/Conditional-inst.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {
/* *******************************************************************************/
struct HybridGaussianConditional::ConstructorHelper {
  KeyVector frontals, parents;
  HybridGaussianFactor::FactorValuePairs pairs;
  double negLogConstant;
  /// Compute all variables needed for the private constructor below.
  ConstructorHelper(const Conditionals &conditionals) {
    negLogConstant = std::numeric_limits<double>::infinity();

    auto func = [&](const GaussianConditional::shared_ptr &c)
        -> GaussianFactorValuePair {
      double value = 0.0;
      if (c) {
        if (frontals.empty()) {
          frontals = KeyVector(c->frontals().begin(), c->frontals().end());
          parents = KeyVector(c->parents().begin(), c->parents().end());
        }
        value = c->negLogConstant();
        negLogConstant = std::min(negLogConstant, value);
      }
      return {std::dynamic_pointer_cast<GaussianFactor>(c), value};
    };
    pairs = HybridGaussianFactor::FactorValuePairs(conditionals, func);
  }
};

/* *******************************************************************************/
HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKeys &discreteParents,
    const HybridGaussianConditional::Conditionals &conditionals,
    const ConstructorHelper &helper)
    : BaseFactor(discreteParents, helper.pairs),
      BaseConditional(helper.frontals.size()),
      conditionals_(conditionals),
      negLogConstant_(helper.negLogConstant) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKeys &discreteParents,
    const HybridGaussianConditional::Conditionals &conditionals)
    : HybridGaussianConditional(discreteParents, conditionals,
                                ConstructorHelper(conditionals)) {}

HybridGaussianConditional::HybridGaussianConditional(
    const DiscreteKey &discreteParent,
    const std::vector<GaussianConditional::shared_ptr> &conditionals)
    : HybridGaussianConditional(DiscreteKeys{discreteParent},
                                Conditionals({discreteParent}, conditionals)) {}

/* *******************************************************************************/
const HybridGaussianConditional::Conditionals &
HybridGaussianConditional::conditionals() const {
  return conditionals_;
}

/* *******************************************************************************/
GaussianFactorGraphTree HybridGaussianConditional::asGaussianFactorGraphTree()
    const {
  auto wrap = [this](const GaussianConditional::shared_ptr &gc) {
    // First check if conditional has not been pruned
    if (gc) {
      const double Cgm_Kgcm = gc->negLogConstant() - this->negLogConstant_;
      // If there is a difference in the covariances, we need to account for
      // that since the error is dependent on the mode.
      if (Cgm_Kgcm > 0.0) {
        // We add a constant factor which will be used when computing
        // the probability of the discrete variables.
        Vector c(1);
        c << std::sqrt(2.0 * Cgm_Kgcm);
        auto constantFactor = std::make_shared<JacobianFactor>(c);
        return GaussianFactorGraph{gc, constantFactor};
      }
    }
    return GaussianFactorGraph{gc};
  };
  return {conditionals_, wrap};
}

/* *******************************************************************************/
size_t HybridGaussianConditional::nrComponents() const {
  size_t total = 0;
  conditionals_.visit([&total](const GaussianFactor::shared_ptr &node) {
    if (node) total += 1;
  });
  return total;
}

/* *******************************************************************************/
GaussianConditional::shared_ptr HybridGaussianConditional::operator()(
    const DiscreteValues &discreteValues) const {
  auto &ptr = conditionals_(discreteValues);
  if (!ptr) return nullptr;
  auto conditional = std::dynamic_pointer_cast<GaussianConditional>(ptr);
  if (conditional)
    return conditional;
  else
    throw std::logic_error(
        "A HybridGaussianConditional unexpectedly contained a non-conditional");
}

/* *******************************************************************************/
bool HybridGaussianConditional::equals(const HybridFactor &lf,
                                       double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr) return false;

  // This will return false if either conditionals_ is empty or e->conditionals_
  // is empty, but not if both are empty or both are not empty:
  if (conditionals_.empty() ^ e->conditionals_.empty()) return false;

  // Check the base and the factors:
  return BaseFactor::equals(*e, tol) &&
         conditionals_.equals(e->conditionals_,
                              [tol](const GaussianConditional::shared_ptr &f1,
                                    const GaussianConditional::shared_ptr &f2) {
                                return f1->equals(*(f2), tol);
                              });
}

/* *******************************************************************************/
void HybridGaussianConditional::print(const std::string &s,
                                      const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  if (isContinuous()) std::cout << "Continuous ";
  if (isDiscrete()) std::cout << "Discrete ";
  if (isHybrid()) std::cout << "Hybrid ";
  BaseConditional::print("", formatter);
  std::cout << " Discrete Keys = ";
  for (auto &dk : discreteKeys()) {
    std::cout << "(" << formatter(dk.first) << ", " << dk.second << "), ";
  }
  std::cout << std::endl
            << " logNormalizationConstant: " << -negLogConstant() << std::endl
            << std::endl;
  conditionals_.print(
      "", [&](Key k) { return formatter(k); },
      [&](const GaussianConditional::shared_ptr &gf) -> std::string {
        RedirectCout rd;
        if (gf && !gf->empty()) {
          gf->print("", formatter);
          return rd.str();
        } else {
          return "nullptr";
        }
      });
}

/* ************************************************************************* */
KeyVector HybridGaussianConditional::continuousParents() const {
  // Get all parent keys:
  const auto range = parents();
  KeyVector continuousParentKeys(range.begin(), range.end());
  // Loop over all discrete keys:
  for (const auto &discreteKey : discreteKeys()) {
    const Key key = discreteKey.first;
    // remove that key from continuousParentKeys:
    continuousParentKeys.erase(std::remove(continuousParentKeys.begin(),
                                           continuousParentKeys.end(), key),
                               continuousParentKeys.end());
  }
  return continuousParentKeys;
}

/* ************************************************************************* */
bool HybridGaussianConditional::allFrontalsGiven(
    const VectorValues &given) const {
  for (auto &&kv : given) {
    if (given.find(kv.first) == given.end()) {
      return false;
    }
  }
  return true;
}

/* ************************************************************************* */
std::shared_ptr<HybridGaussianFactor> HybridGaussianConditional::likelihood(
    const VectorValues &given) const {
  if (!allFrontalsGiven(given)) {
    throw std::runtime_error(
        "HybridGaussianConditional::likelihood: given values are missing some "
        "frontals.");
  }

  const DiscreteKeys discreteParentKeys = discreteKeys();
  const KeyVector continuousParentKeys = continuousParents();
  const HybridGaussianFactor::FactorValuePairs likelihoods(
      conditionals_,
      [&](const GaussianConditional::shared_ptr &conditional)
          -> GaussianFactorValuePair {
        const auto likelihood_m = conditional->likelihood(given);
        const double Cgm_Kgcm = conditional->negLogConstant() - negLogConstant_;
        if (Cgm_Kgcm == 0.0) {
          return {likelihood_m, 0.0};
        } else {
          // Add a constant to the likelihood in case the noise models
          // are not all equal.
          return {likelihood_m, Cgm_Kgcm};
        }
      });
  return std::make_shared<HybridGaussianFactor>(discreteParentKeys,
                                                likelihoods);
}

/* ************************************************************************* */
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &discreteKeys) {
  std::set<DiscreteKey> s(discreteKeys.begin(), discreteKeys.end());
  return s;
}

/* ************************************************************************* */
/**
 * @brief Helper function to get the pruner functional.
 *
 * @param discreteProbs The probabilities of only discrete keys.
 * @return std::function<GaussianConditional::shared_ptr(
 * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
 */
std::function<GaussianConditional::shared_ptr(
    const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
HybridGaussianConditional::prunerFunc(const DecisionTreeFactor &discreteProbs) {
  // Get the discrete keys as sets for the decision tree
  // and the hybrid gaussian conditional.
  auto discreteProbsKeySet = DiscreteKeysAsSet(discreteProbs.discreteKeys());
  auto hybridGaussianCondKeySet = DiscreteKeysAsSet(this->discreteKeys());

  auto pruner = [discreteProbs, discreteProbsKeySet, hybridGaussianCondKeySet](
                    const Assignment<Key> &choices,
                    const GaussianConditional::shared_ptr &conditional)
      -> GaussianConditional::shared_ptr {
    // typecast so we can use this to get probability value
    const DiscreteValues values(choices);

    // Case where the hybrid gaussian conditional has the same
    // discrete keys as the decision tree.
    if (hybridGaussianCondKeySet == discreteProbsKeySet) {
      if (discreteProbs(values) == 0.0) {
        // empty aka null pointer
        std::shared_ptr<GaussianConditional> null;
        return null;
      } else {
        return conditional;
      }
    } else {
      std::vector<DiscreteKey> set_diff;
      std::set_difference(
          discreteProbsKeySet.begin(), discreteProbsKeySet.end(),
          hybridGaussianCondKeySet.begin(), hybridGaussianCondKeySet.end(),
          std::back_inserter(set_diff));

      const std::vector<DiscreteValues> assignments =
          DiscreteValues::CartesianProduct(set_diff);
      for (const DiscreteValues &assignment : assignments) {
        DiscreteValues augmented_values(values);
        augmented_values.insert(assignment);

        // If any one of the sub-branches are non-zero,
        // we need this conditional.
        if (discreteProbs(augmented_values) > 0.0) {
          return conditional;
        }
      }
      // If we are here, it means that all the sub-branches are 0,
      // so we prune.
      return nullptr;
    }
  };
  return pruner;
}

/* *******************************************************************************/
void HybridGaussianConditional::prune(const DecisionTreeFactor &discreteProbs) {
  // Functional which loops over all assignments and create a set of
  // GaussianConditionals
  auto pruner = prunerFunc(discreteProbs);

  auto pruned_conditionals = conditionals_.apply(pruner);
  conditionals_.root_ = pruned_conditionals.root_;
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> HybridGaussianConditional::logProbability(
    const VectorValues &continuousValues) const {
  // functor to calculate (double) logProbability value from
  // GaussianConditional.
  auto probFunc =
      [continuousValues](const GaussianConditional::shared_ptr &conditional) {
        if (conditional) {
          return conditional->logProbability(continuousValues);
        } else {
          // Return arbitrarily small logProbability if conditional is null
          // Conditional is null if it is pruned out.
          return -1e20;
        }
      };
  return DecisionTree<Key, double>(conditionals_, probFunc);
}

/* *******************************************************************************/
double HybridGaussianConditional::logProbability(
    const HybridValues &values) const {
  auto conditional = conditionals_(values.discrete());
  return conditional->logProbability(values.continuous());
}

/* *******************************************************************************/
double HybridGaussianConditional::evaluate(const HybridValues &values) const {
  auto conditional = conditionals_(values.discrete());
  return conditional->evaluate(values.continuous());
}

}  // namespace gtsam

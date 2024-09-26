/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianFactor.cpp
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include "gtsam/hybrid/HybridFactor.h"

namespace gtsam {

/* *******************************************************************************/
HybridGaussianFactor::Factors HybridGaussianFactor::augment(
    const FactorValuePairs &factors) {
  // Find the minimum value so we can "proselytize" to positive values.
  // Done because we can't have sqrt of negative numbers.
  Factors gaussianFactors;
  AlgebraicDecisionTree<Key> valueTree;
  std::tie(gaussianFactors, valueTree) = unzip(factors);

  // Compute minimum value for normalization.
  double min_value = valueTree.min();

  // Finally, update the [A|b] matrices.
  auto update = [&min_value](const GaussianFactorValuePair &gfv) {
    auto [gf, value] = gfv;

    auto jf = std::dynamic_pointer_cast<JacobianFactor>(gf);
    if (!jf) return gf;

    double normalized_value = value - min_value;

    // If the value is 0, do nothing
    if (normalized_value == 0.0) return gf;

    GaussianFactorGraph gfg;
    gfg.push_back(jf);

    Vector c(1);
    // When hiding c inside the `b` vector, value == 0.5*c^2
    c << std::sqrt(2.0 * normalized_value);
    auto constantFactor = std::make_shared<JacobianFactor>(c);

    gfg.push_back(constantFactor);
    return std::dynamic_pointer_cast<GaussianFactor>(
        std::make_shared<JacobianFactor>(gfg));
  };
  return Factors(factors, update);
}

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(
    const DiscreteKey &discreteKey,
    const std::vector<GaussianFactor::shared_ptr> &factors)
    : Base(HybridFactor::Category::Hybrid) {
  // Extract continuous keys from first-null factor and verify all others
  KeyVector continuousKeys;
  for (const auto &factor : factors) {
    if (!factor) continue;
    if (continuousKeys.empty()) {
      continuousKeys = factor->keys();
    } else if (factor->keys() != continuousKeys) {
      throw std::invalid_argument("All factors must have the same keys");
    }
  }

  // Check that this worked.
  if (continuousKeys.empty()) {
    throw std::invalid_argument("Need at least one non-null factor.");
  }

  // Initialize the base class
  Factor::keys_ = continuousKeys;
  Factor::keys_.push_back(discreteKey.first);
  Base::discreteKeys_ = {discreteKey};
  Base::continuousKeys_ = continuousKeys;

  // Build the DecisionTree from factor vector
  factors_ = Factors({discreteKey}, factors);
}

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(
    const DiscreteKey &discreteKey,
    const std::vector<GaussianFactorValuePair> &factorPairs)
    : Base(HybridFactor::Category::Hybrid) {
  // Extract continuous keys from first-null factor and verify all others
  KeyVector continuousKeys;
  for (const auto &pair : factorPairs) {
    if (!pair.first) continue;
    if (continuousKeys.empty()) {
      continuousKeys = pair.first->keys();
    } else if (pair.first->keys() != continuousKeys) {
      throw std::invalid_argument("All factors must have the same keys");
    }
  }

  // Check that this worked.
  if (continuousKeys.empty()) {
    throw std::invalid_argument("Need at least one non-null factor.");
  }

  // Initialize the base class
  Factor::keys_ = continuousKeys;
  Factor::keys_.push_back(discreteKey.first);
  Base::discreteKeys_ = {discreteKey};
  Base::continuousKeys_ = continuousKeys;

  // Build the FactorValuePairs DecisionTree
  FactorValuePairs pairTree({discreteKey}, factorPairs);

  // Assign factors_ after calling augment
  factors_ = augment(pairTree);
}

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(const DiscreteKeys &discreteKeys,
                                           const FactorValuePairs &factorPairs)
    : Base(HybridFactor::Category::Hybrid) {
  // Verify that all factors have the same keys
  KeyVector continuousKeys;
  factorPairs.visit([&](const GaussianFactorValuePair &pair) {
    if (pair.first) {
      if (continuousKeys.empty()) {
        continuousKeys = pair.first->keys();
      } else if (pair.first->keys() != continuousKeys) {
        throw std::invalid_argument("All factors must have the same keys");
      }
    }
  });

  // Check that this worked.
  if (continuousKeys.empty()) {
    throw std::invalid_argument("Need at least one non-null factor.");
  }

  // Initialize the base class
  Factor::keys_ = continuousKeys;
  for (const auto &discreteKey : discreteKeys) {
    Factor::keys_.push_back(discreteKey.first);
  }
  Base::discreteKeys_ = discreteKeys;
  Base::continuousKeys_ = continuousKeys;

  // Assign factors_ after calling augment
  factors_ = augment(factorPairs);
}

/* *******************************************************************************/
bool HybridGaussianFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr) return false;

  // This will return false if either factors_ is empty or e->factors_ is
  // empty, but not if both are empty or both are not empty:
  if (factors_.empty() ^ e->factors_.empty()) return false;

  // Check the base and the factors:
  return Base::equals(*e, tol) &&
         factors_.equals(e->factors_,
                         [tol](const sharedFactor &f1, const sharedFactor &f2) {
                           return f1->equals(*f2, tol);
                         });
}

/* *******************************************************************************/
void HybridGaussianFactor::print(const std::string &s,
                                 const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  std::cout << "HybridGaussianFactor" << std::endl;
  HybridFactor::print("", formatter);
  std::cout << "{\n";
  if (factors_.empty()) {
    std::cout << "  empty" << std::endl;
  } else {
    factors_.print(
        "", [&](Key k) { return formatter(k); },
        [&](const sharedFactor &gf) -> std::string {
          RedirectCout rd;
          std::cout << ":\n";
          if (gf) {
            gf->print("", formatter);
            return rd.str();
          } else {
            return "nullptr";
          }
        });
  }
  std::cout << "}" << std::endl;
}

/* *******************************************************************************/
HybridGaussianFactor::sharedFactor HybridGaussianFactor::operator()(
    const DiscreteValues &assignment) const {
  return factors_(assignment);
}

/* *******************************************************************************/
GaussianFactorGraphTree HybridGaussianFactor::add(
    const GaussianFactorGraphTree &sum) const {
  using Y = GaussianFactorGraph;
  auto add = [](const Y &graph1, const Y &graph2) {
    auto result = graph1;
    result.push_back(graph2);
    return result;
  };
  const auto tree = asGaussianFactorGraphTree();
  return sum.empty() ? tree : sum.apply(tree, add);
}

/* *******************************************************************************/
GaussianFactorGraphTree HybridGaussianFactor::asGaussianFactorGraphTree()
    const {
  auto wrap = [](const sharedFactor &gf) { return GaussianFactorGraph{gf}; };
  return {factors_, wrap};
}

/* *******************************************************************************/
double HybridGaussianFactor::potentiallyPrunedComponentError(
    const sharedFactor &gf, const VectorValues &values) const {
  // Check if valid pointer
  if (gf) {
    return gf->error(values);
  } else {
    // If not valid, pointer, it means this component was pruned,
    // so we return maximum error.
    // This way the negative exponential will give
    // a probability value close to 0.0.
    return std::numeric_limits<double>::max();
  }
}
/* *******************************************************************************/
AlgebraicDecisionTree<Key> HybridGaussianFactor::errorTree(
    const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc = [this, &continuousValues](const sharedFactor &gf) {
    return this->potentiallyPrunedComponentError(gf, continuousValues);
  };
  DecisionTree<Key, double> error_tree(factors_, errorFunc);
  return error_tree;
}

/* *******************************************************************************/
double HybridGaussianFactor::error(const HybridValues &values) const {
  // Directly index to get the component, no need to build the whole tree.
  const sharedFactor gf = factors_(values.discrete());
  return potentiallyPrunedComponentError(gf, values.continuous());
}

}  // namespace gtsam

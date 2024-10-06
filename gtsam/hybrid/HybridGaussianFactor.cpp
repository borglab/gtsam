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

#include <gtsam/base/types.h>
#include <gtsam/base/utilities.h>
#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridGaussianProductFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

/* *******************************************************************************/
HybridGaussianFactor::FactorValuePairs
HybridGaussianFactor::augment(const FactorValuePairs &factors) {
  // Find the minimum value so we can "proselytize" to positive values.
  // Done because we can't have sqrt of negative numbers.
  DecisionTree<Key, GaussianFactor::shared_ptr> gaussianFactors;
  AlgebraicDecisionTree<Key> valueTree;
  std::tie(gaussianFactors, valueTree) = unzip(factors);

  // Compute minimum value for normalization.
  double min_value = valueTree.min();

  // Finally, update the [A|b] matrices.
  auto update = [&min_value](const auto &gfv) -> GaussianFactorValuePair {
    auto [gf, value] = gfv;

    auto jf = std::dynamic_pointer_cast<JacobianFactor>(gf);
    if (!jf)
      return {gf, 0.0}; // should this be zero or infinite?

    double normalized_value = value - min_value;

    // If the value is 0, do nothing
    if (normalized_value == 0.0)
      return {gf, 0.0};

    GaussianFactorGraph gfg;
    gfg.push_back(jf);

    Vector c(1);
    // When hiding c inside the `b` vector, value == 0.5*c^2
    c << std::sqrt(2.0 * normalized_value);
    auto constantFactor = std::make_shared<JacobianFactor>(c);

    gfg.push_back(constantFactor);
    return {std::make_shared<JacobianFactor>(gfg), normalized_value};
  };
  return FactorValuePairs(factors, update);
}

/* *******************************************************************************/
struct HybridGaussianFactor::ConstructorHelper {
  KeyVector continuousKeys;  // Continuous keys extracted from factors
  DiscreteKeys discreteKeys; // Discrete keys provided to the constructors
  FactorValuePairs pairs;    // The decision tree with factors and scalars

  ConstructorHelper(const DiscreteKey &discreteKey,
                    const std::vector<GaussianFactor::shared_ptr> &factors)
      : discreteKeys({discreteKey}) {
    // Extract continuous keys from the first non-null factor
    for (const auto &factor : factors) {
      if (factor && continuousKeys.empty()) {
        continuousKeys = factor->keys();
        break;
      }
    }
    // Build the FactorValuePairs DecisionTree
    pairs = FactorValuePairs(
        DecisionTree<Key, GaussianFactor::shared_ptr>(discreteKeys, factors),
        [](const auto &f) {
          return std::pair{f, 0.0};
        });
  }

  ConstructorHelper(const DiscreteKey &discreteKey,
                    const std::vector<GaussianFactorValuePair> &factorPairs)
      : discreteKeys({discreteKey}) {
    // Extract continuous keys from the first non-null factor
    for (const auto &pair : factorPairs) {
      if (pair.first && continuousKeys.empty()) {
        continuousKeys = pair.first->keys();
        break;
      }
    }

    // Build the FactorValuePairs DecisionTree
    pairs = FactorValuePairs(discreteKeys, factorPairs);
  }

  ConstructorHelper(const DiscreteKeys &discreteKeys,
                    const FactorValuePairs &factorPairs)
      : discreteKeys(discreteKeys) {
    // Extract continuous keys from the first non-null factor
    // TODO: just stop after first non-null factor
    factorPairs.visit([&](const GaussianFactorValuePair &pair) {
      if (pair.first && continuousKeys.empty()) {
        continuousKeys = pair.first->keys();
      }
    });

    // Build the FactorValuePairs DecisionTree
    pairs = factorPairs;
  }
};

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(const ConstructorHelper &helper)
    : Base(helper.continuousKeys, helper.discreteKeys),
      factors_(augment(helper.pairs)) {}

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(
    const DiscreteKey &discreteKey,
    const std::vector<GaussianFactor::shared_ptr> &factorPairs)
    : HybridGaussianFactor(ConstructorHelper(discreteKey, factorPairs)) {}

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(
    const DiscreteKey &discreteKey,
    const std::vector<GaussianFactorValuePair> &factorPairs)
    : HybridGaussianFactor(ConstructorHelper(discreteKey, factorPairs)) {}

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(const DiscreteKeys &discreteKeys,
                                           const FactorValuePairs &factorPairs)
    : HybridGaussianFactor(ConstructorHelper(discreteKeys, factorPairs)) {}

/* *******************************************************************************/
bool HybridGaussianFactor::equals(const HybridFactor &lf, double tol) const {
  const This *e = dynamic_cast<const This *>(&lf);
  if (e == nullptr)
    return false;

  // This will return false if either factors_ is empty or e->factors_ is
  // empty, but not if both are empty or both are not empty:
  if (factors_.empty() ^ e->factors_.empty())
    return false;

  // Check the base and the factors:
  auto compareFunc = [tol](const auto &pair1, const auto &pair2) {
    auto f1 = pair1.first, f2 = pair2.first;
    bool match = (!f1 && !f2) || (f1 && f2 && f1->equals(*f2, tol));
    return match && gtsam::equal(pair1.second, pair2.second, tol);
  };
  return Base::equals(*e, tol) && factors_.equals(e->factors_, compareFunc);
}

/* *******************************************************************************/
void HybridGaussianFactor::print(const std::string &s,
                                 const KeyFormatter &formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  HybridFactor::print("", formatter);
  std::cout << "{\n";
  if (factors_.empty()) {
    std::cout << "  empty" << std::endl;
  } else {
    factors_.print(
        "", [&](Key k) { return formatter(k); },
        [&](const auto &pair) -> std::string {
          RedirectCout rd;
          std::cout << ":\n";
          if (pair.first) {
            pair.first->print("", formatter);
            std::cout << "scalar: " << pair.second << "\n";
            return rd.str();
          } else {
            return "nullptr";
          }
        });
  }
  std::cout << "}" << std::endl;
}

/* *******************************************************************************/
HybridGaussianFactor::sharedFactor
HybridGaussianFactor::operator()(const DiscreteValues &assignment) const {
  return factors_(assignment).first;
}

/* *******************************************************************************/
HybridGaussianProductFactor HybridGaussianFactor::asProductFactor() const {
  return {{factors_,
           [](const auto &pair) { return GaussianFactorGraph{pair.first}; }}};
}

/* *******************************************************************************/
/// Helper method to compute the error of a component.
static double
PotentiallyPrunedComponentError(const GaussianFactor::shared_ptr &gf,
                                const VectorValues &values) {
  // Check if valid pointer
  if (gf) {
    return gf->error(values);
  } else {
    // If nullptr this component was pruned, so we return maximum error. This
    // way the negative exponential will give a probability value close to 0.0.
    return std::numeric_limits<double>::max();
  }
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key>
HybridGaussianFactor::errorTree(const VectorValues &continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc = [&continuousValues](const auto &pair) {
    return PotentiallyPrunedComponentError(pair.first, continuousValues);
  };
  DecisionTree<Key, double> error_tree(factors_, errorFunc);
  return error_tree;
}

/* *******************************************************************************/
double HybridGaussianFactor::error(const HybridValues &values) const {
  // Directly index to get the component, no need to build the whole tree.
  const auto pair = factors_(values.discrete());
  return PotentiallyPrunedComponentError(pair.first, values.continuous());
}

} // namespace gtsam

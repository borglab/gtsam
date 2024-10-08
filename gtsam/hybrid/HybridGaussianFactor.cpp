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
struct HybridGaussianFactor::ConstructorHelper {
  KeyVector continuousKeys;   // Continuous keys extracted from factors
  DiscreteKeys discreteKeys;  // Discrete keys provided to the constructors
  FactorValuePairs pairs;     // The decision tree with factors and scalars

  /// Constructor for a single discrete key and a vector of Gaussian factors
  ConstructorHelper(const DiscreteKey& discreteKey,
                    const std::vector<GaussianFactor::shared_ptr>& factors)
      : discreteKeys({discreteKey}) {
    // Extract continuous keys from the first non-null factor
    for (const auto& factor : factors) {
      if (factor && continuousKeys.empty()) {
        continuousKeys = factor->keys();
        break;
      }
    }
    // Build the FactorValuePairs DecisionTree
    pairs = FactorValuePairs(
        DecisionTree<Key, GaussianFactor::shared_ptr>(discreteKeys, factors),
        [](const auto& f) {
          return std::pair{f,
                           f ? 0.0 : std::numeric_limits<double>::infinity()};
        });
  }

  /// Constructor for a single discrete key and a vector of
  /// GaussianFactorValuePairs
  ConstructorHelper(const DiscreteKey& discreteKey,
                    const std::vector<GaussianFactorValuePair>& factorPairs)
      : discreteKeys({discreteKey}) {
    // Extract continuous keys from the first non-null factor
    for (const auto& pair : factorPairs) {
      if (pair.first && continuousKeys.empty()) {
        continuousKeys = pair.first->keys();
        break;
      }
    }

    // Build the FactorValuePairs DecisionTree
    pairs = FactorValuePairs(discreteKeys, factorPairs);
  }

  /// Constructor for a vector of discrete keys and a vector of
  /// GaussianFactorValuePairs
  ConstructorHelper(const DiscreteKeys& discreteKeys,
                    const FactorValuePairs& factorPairs)
      : discreteKeys(discreteKeys) {
    // Extract continuous keys from the first non-null factor
    // TODO: just stop after first non-null factor
    factorPairs.visit([&](const GaussianFactorValuePair& pair) {
      if (pair.first && continuousKeys.empty()) {
        continuousKeys = pair.first->keys();
      }
    });

    // Build the FactorValuePairs DecisionTree
    pairs = factorPairs;
  }
};

/* *******************************************************************************/
HybridGaussianFactor::HybridGaussianFactor(const ConstructorHelper& helper)
    : Base(helper.continuousKeys, helper.discreteKeys),
      factors_(helper.pairs) {}

HybridGaussianFactor::HybridGaussianFactor(
    const DiscreteKey& discreteKey,
    const std::vector<GaussianFactor::shared_ptr>& factors)
    : HybridGaussianFactor(ConstructorHelper(discreteKey, factors)) {}

HybridGaussianFactor::HybridGaussianFactor(
    const DiscreteKey& discreteKey,
    const std::vector<GaussianFactorValuePair>& factorPairs)
    : HybridGaussianFactor(ConstructorHelper(discreteKey, factorPairs)) {}

HybridGaussianFactor::HybridGaussianFactor(const DiscreteKeys& discreteKeys,
                                           const FactorValuePairs& factors)
    : HybridGaussianFactor(ConstructorHelper(discreteKeys, factors)) {}

/* *******************************************************************************/
bool HybridGaussianFactor::equals(const HybridFactor& lf, double tol) const {
  const This* e = dynamic_cast<const This*>(&lf);
  if (e == nullptr) return false;

  // This will return false if either factors_ is empty or e->factors_ is
  // empty, but not if both are empty or both are not empty:
  if (factors_.empty() ^ e->factors_.empty()) return false;

  // Check the base and the factors:
  auto compareFunc = [tol](const auto& pair1, const auto& pair2) {
    auto f1 = pair1.first, f2 = pair2.first;
    bool match = (!f1 && !f2) || (f1 && f2 && f1->equals(*f2, tol));
    return match && gtsam::equal(pair1.second, pair2.second, tol);
  };
  return Base::equals(*e, tol) && factors_.equals(e->factors_, compareFunc);
}

/* *******************************************************************************/
void HybridGaussianFactor::print(const std::string& s,
                                 const KeyFormatter& formatter) const {
  std::cout << (s.empty() ? "" : s + "\n");
  HybridFactor::print("", formatter);
  std::cout << "{\n";
  if (factors_.empty()) {
    std::cout << "  empty" << std::endl;
  } else {
    factors_.print(
        "", [&](Key k) { return formatter(k); },
        [&](const auto& pair) -> std::string {
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
GaussianFactorValuePair HybridGaussianFactor::operator()(
    const DiscreteValues& assignment) const {
  return factors_(assignment);
}

/* *******************************************************************************/
HybridGaussianProductFactor HybridGaussianFactor::asProductFactor() const {
  // Implemented by creating a new DecisionTree where:
  // - The structure (keys and assignments) is preserved from factors_
  // - Each leaf converted to a GaussianFactorGraph with just the factor and its
  // scalar.
  return {{factors_,
           [](const auto& pair) -> std::pair<GaussianFactorGraph, double> {
             return {GaussianFactorGraph{pair.first}, pair.second};
           }}};
}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> HybridGaussianFactor::errorTree(
    const VectorValues& continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc = [&continuousValues](const auto& pair) {
    return pair.first ? pair.first->error(continuousValues) + pair.second
                      : std::numeric_limits<double>::infinity();
  };
  DecisionTree<Key, double> error_tree(factors_, errorFunc);
  return error_tree;
}

/* *******************************************************************************/
double HybridGaussianFactor::error(const HybridValues& values) const {
  // Directly index to get the component, no need to build the whole tree.
  const auto pair = factors_(values.discrete());
  return pair.first ? pair.first->error(values.continuous()) + pair.second
                    : std::numeric_limits<double>::infinity();
}

}  // namespace gtsam

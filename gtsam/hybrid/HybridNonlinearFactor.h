/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactor.h
 * @brief  Nonlinear Mixture factor of continuous and discrete.
 * @author Kevin Doherty, kdoherty@mit.edu
 * @author Varun Agrawal
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace gtsam {

/// Alias for a NonlinearFactor shared pointer and double scalar pair.
using NonlinearFactorValuePair = std::pair<NonlinearFactor::shared_ptr, double>;

/**
 * @brief Implementation of a discrete conditional mixture factor.
 *
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a mixture component corresponding to a NonlinearFactor
 * type of measurement.
 *
 * This class stores all factors as HybridFactors which can then be typecast to
 * one of (NonlinearFactor, GaussianFactor) which can then be checked to perform
 * the correct operation.
 */
class HybridNonlinearFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = HybridNonlinearFactor;
  using shared_ptr = std::shared_ptr<HybridNonlinearFactor>;
  using sharedFactor = std::shared_ptr<NonlinearFactor>;

  /**
   * @brief typedef for DecisionTree which has Keys as node labels and
   * pairs of NonlinearFactor & an arbitrary scalar as leaf nodes.
   */
  using Factors = DecisionTree<Key, NonlinearFactorValuePair>;

 private:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  Factors factors_;

 public:
  HybridNonlinearFactor() = default;

  /**
   * @brief Construct from Decision tree.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Decision tree with of shared factors.
   */
  HybridNonlinearFactor(const KeyVector& keys, const DiscreteKeys& discreteKeys,
                        const Factors& factors)
      : Base(keys, discreteKeys), factors_(factors) {}

  /**
   * @brief Convenience constructor that generates the underlying factor
   * decision tree for us.
   *
   * Here it is important that the vector of factors has the correct number of
   * elements based on the number of discrete keys and the cardinality of the
   * keys, so that the decision tree is constructed appropriately.
   *
   * @tparam FACTOR The type of the factor shared pointers being passed in.
   * Will be typecast to NonlinearFactor shared pointers.
   * @param keys Vector of keys for continuous factors.
   * @param discreteKey The discrete key indexing each component factor.
   * @param factors Vector of nonlinear factor and scalar pairs.
   * Same size as the cardinality of discreteKey.
   */
  template <typename FACTOR>
  HybridNonlinearFactor(
      const KeyVector& keys, const DiscreteKey& discreteKey,
      const std::vector<std::pair<std::shared_ptr<FACTOR>, double>>& factors)
      : Base(keys, {discreteKey}) {
    std::vector<NonlinearFactorValuePair> nonlinear_factors;
    KeySet continuous_keys_set(keys.begin(), keys.end());
    KeySet factor_keys_set;
    for (auto&& [f, val] : factors) {
      // Insert all factor continuous keys in the continuous keys set.
      std::copy(f->keys().begin(), f->keys().end(),
                std::inserter(factor_keys_set, factor_keys_set.end()));

      if (auto nf = std::dynamic_pointer_cast<NonlinearFactor>(f)) {
        nonlinear_factors.emplace_back(nf, val);
      } else {
        throw std::runtime_error(
            "Factors passed into HybridNonlinearFactor need to be nonlinear!");
      }
    }
    factors_ = Factors({discreteKey}, nonlinear_factors);

    if (continuous_keys_set != factor_keys_set) {
      throw std::runtime_error(
          "The specified continuous keys and the keys in the factors don't "
          "match!");
    }
  }

  /**
   * @brief Compute error of the HybridNonlinearFactor as a tree.
   *
   * @param continuousValues The continuous values for which to compute the
   * error.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factor, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> errorTree(const Values& continuousValues) const {
    // functor to convert from sharedFactor to double error value.
    auto errorFunc =
        [continuousValues](const std::pair<sharedFactor, double>& f) {
          auto [factor, val] = f;
          return factor->error(continuousValues) + (0.5 * val);
        };
    DecisionTree<Key, double> result(factors_, errorFunc);
    return result;
  }

  /**
   * @brief Compute error of factor given both continuous and discrete values.
   *
   * @param continuousValues The continuous Values.
   * @param discreteValues The discrete Values.
   * @return double The error of this factor.
   */
  double error(const Values& continuousValues,
               const DiscreteValues& discreteValues) const {
    // Retrieve the factor corresponding to the assignment in discreteValues.
    auto [factor, val] = factors_(discreteValues);
    // Compute the error for the selected factor
    const double factorError = factor->error(continuousValues);
    return factorError + (0.5 * val);
  }

  /**
   * @brief Compute error of factor given hybrid values.
   *
   * @param values The continuous Values and the discrete assignment.
   * @return double The error of this factor.
   */
  double error(const HybridValues& values) const override {
    return error(values.nonlinear(), values.discrete());
  }

  /**
   * @brief Get the dimension of the factor (number of rows on linearization).
   * Returns the dimension of the first component factor.
   * @return size_t
   */
  size_t dim() const {
    const auto assignments = DiscreteValues::CartesianProduct(discreteKeys_);
    auto [factor, val] = factors_(assignments.at(0));
    return factor->dim();
  }

  /// Testable
  /// @{

  /// print to stdout
  void print(
      const std::string& s = "",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    std::cout << (s.empty() ? "" : s + " ");
    Base::print("", keyFormatter);
    std::cout << "\nHybridNonlinearFactor\n";
    auto valueFormatter = [](const std::pair<sharedFactor, double>& v) {
      auto [factor, val] = v;
      if (factor) {
        return "Nonlinear factor on " + std::to_string(factor->size()) +
               " keys";
      } else {
        return std::string("nullptr");
      }
    };
    factors_.print("", keyFormatter, valueFormatter);
  }

  /// Check equality
  bool equals(const HybridFactor& other, double tol = 1e-9) const override {
    // We attempt a dynamic cast from HybridFactor to HybridNonlinearFactor. If
    // it fails, return false.
    if (!dynamic_cast<const HybridNonlinearFactor*>(&other)) return false;

    // If the cast is successful, we'll properly construct a
    // HybridNonlinearFactor object from `other`
    const HybridNonlinearFactor& f(
        static_cast<const HybridNonlinearFactor&>(other));

    // Ensure that this HybridNonlinearFactor and `f` have the same `factors_`.
    auto compare = [tol](const std::pair<sharedFactor, double>& a,
                         const std::pair<sharedFactor, double>& b) {
      return traits<NonlinearFactor>::Equals(*a.first, *b.first, tol) &&
             (a.second == b.second);
    };
    if (!factors_.equals(f.factors_, compare)) return false;

    // If everything above passes, and the keys_ and discreteKeys_
    // member variables are identical, return true.
    return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
            (discreteKeys_ == f.discreteKeys_));
  }

  /// @}

  /// Linearize specific nonlinear factors based on the assignment in
  /// discreteValues.
  GaussianFactor::shared_ptr linearize(
      const Values& continuousValues,
      const DiscreteValues& discreteValues) const {
    auto factor = factors_(discreteValues).first;
    return factor->linearize(continuousValues);
  }

  /// Linearize all the continuous factors to get a HybridGaussianFactor.
  std::shared_ptr<HybridGaussianFactor> linearize(
      const Values& continuousValues) const {
    // functional to linearize each factor in the decision tree
    auto linearizeDT =
        [continuousValues](const std::pair<sharedFactor, double>& f)
        -> GaussianFactorValuePair {
      auto [factor, val] = f;
      return {factor->linearize(continuousValues), val};
    };

    DecisionTree<Key, std::pair<GaussianFactor::shared_ptr, double>>
        linearized_factors(factors_, linearizeDT);

    return std::make_shared<HybridGaussianFactor>(
        continuousKeys_, discreteKeys_, linearized_factors);
  }
};

}  // namespace gtsam

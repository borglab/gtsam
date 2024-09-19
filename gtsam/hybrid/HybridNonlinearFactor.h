/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactor.h
 * @brief  A set of nonlinear factors indexed by a set of discrete keys.
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
 * @brief Implementation of a discrete-conditioned hybrid factor.
 *
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a hybrid component corresponding to a NonlinearFactor.
 *
 * This class stores all factors as HybridFactors which can then be typecast to
 * one of (NonlinearFactor, GaussianFactor) which can then be checked to perform
 * the correct operation.
 *
 * In factor graphs the error function typically returns 0.5*|h(x)-z|^2, i.e.,
 * the negative log-likelihood for a Gaussian noise model.
 * In hybrid factor graphs we allow *adding* an arbitrary scalar dependent on
 * the discrete assignment.
 * For example, adding a 70/30 mode probability is supported by providing the
 * scalars $-log(.7)$ and $-log(.3)$.
 * Note that adding a common constant will not make any difference in the
 * optimization, so $-log(70)$ and $-log(30)$ work just as well.
 *
 * @ingroup hybrid
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
                        const Factors& factors);

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
  AlgebraicDecisionTree<Key> errorTree(const Values& continuousValues) const;

  /**
   * @brief Compute error of factor given both continuous and discrete values.
   *
   * @param continuousValues The continuous Values.
   * @param discreteValues The discrete Values.
   * @return double The error of this factor.
   */
  double error(const Values& continuousValues,
               const DiscreteValues& discreteValues) const;

  /**
   * @brief Compute error of factor given hybrid values.
   *
   * @param values The continuous Values and the discrete assignment.
   * @return double The error of this factor.
   */
  double error(const HybridValues& values) const override;

  /**
   * @brief Get the dimension of the factor (number of rows on linearization).
   * Returns the dimension of the first component factor.
   * @return size_t
   */
  size_t dim() const;

  /// Testable
  /// @{

  /// print to stdout
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  /// Check equality
  bool equals(const HybridFactor& other, double tol = 1e-9) const override;

  /// @}

  /// Linearize specific nonlinear factors based on the assignment in
  /// discreteValues.
  GaussianFactor::shared_ptr linearize(
      const Values& continuousValues,
      const DiscreteValues& discreteValues) const;

  /// Linearize all the continuous factors to get a HybridGaussianFactor.
  std::shared_ptr<HybridGaussianFactor> linearize(
      const Values& continuousValues) const;
};

}  // namespace gtsam

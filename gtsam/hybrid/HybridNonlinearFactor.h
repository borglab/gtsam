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
class GTSAM_EXPORT HybridNonlinearFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = HybridNonlinearFactor;
  using shared_ptr = std::shared_ptr<HybridNonlinearFactor>;
  using sharedFactor = std::shared_ptr<NonlinearFactor>;

  /**
   * @brief typedef for DecisionTree which has Keys as node labels and
   * pairs of NonlinearFactor & an arbitrary scalar as leaf nodes.
   */
  using FactorValuePairs = DecisionTree<Key, NonlinearFactorValuePair>;

 private:
  /// Decision tree of nonlinear factors indexed by discrete keys.
  FactorValuePairs factors_;

  /// HybridFactor method implementation. Should not be used.
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues& continuousValues) const override {
    throw std::runtime_error(
        "HybridNonlinearFactor::error does not take VectorValues.");
  }

 public:
  /// Default constructor, mainly for serialization.
  HybridNonlinearFactor() = default;

  /**
   * @brief Construct a new HybridNonlinearFactor on a single discrete key,
   * providing the factors for each mode m as a vector of factors ϕ_m(x).
   * The value ϕ(x,m) for the factor is simply ϕ_m(x).
   *
   * @param continuousKeys Vector of keys for continuous factors.
   * @param discreteKey The discrete key for the "mode", indexing components.
   * @param factors Vector of gaussian factors, one for each mode.
   */
  HybridNonlinearFactor(
      const KeyVector& continuousKeys, const DiscreteKey& discreteKey,
      const std::vector<NonlinearFactor::shared_ptr>& factors);

  /**
   * @brief Construct a new HybridNonlinearFactor on a single discrete key,
   * including a scalar error value for each mode m. The factors and scalars are
   * provided as a vector of pairs (ϕ_m(x), E_m).
   * The value ϕ(x,m) for the factor is now ϕ_m(x) + E_m.
   *
   * @param continuousKeys Vector of keys for continuous factors.
   * @param discreteKey The discrete key for the "mode", indexing components.
   * @param pairs Vector of gaussian factor-scalar pairs, one per mode.
   */
  HybridNonlinearFactor(const KeyVector& continuousKeys,
                        const DiscreteKey& discreteKey,
                        const std::vector<NonlinearFactorValuePair>& pairs);

  /**
   * @brief Construct a new HybridNonlinearFactor on a several discrete keys M,
   * including a scalar error value for each assignment m. The factors and
   * scalars are provided as a DecisionTree<Key> of pairs (ϕ_M(x), E_M).
   * The value ϕ(x,M) for the factor is again ϕ_m(x) + E_m.
   *
   * @param continuousKeys A vector of keys representing continuous variables.
   * @param discreteKeys Discrete variables and their cardinalities.
   * @param factors The decision tree of nonlinear factor/scalar pairs.
   */
  HybridNonlinearFactor(const KeyVector& continuousKeys,
                        const DiscreteKeys& discreteKeys,
                        const FactorValuePairs& factors);
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

// traits
template <>
struct traits<HybridNonlinearFactor> : public Testable<HybridNonlinearFactor> {
};

}  // namespace gtsam

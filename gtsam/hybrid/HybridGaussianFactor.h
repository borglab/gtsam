/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianFactor.h
 * @brief  A set of GaussianFactors, indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/AlgebraicDecisionTree.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

namespace gtsam {

class HybridValues;
class DiscreteValues;
class VectorValues;

/// Alias for pair of GaussianFactor::shared_pointer and a double value.
using GaussianFactorValuePair = std::pair<GaussianFactor::shared_ptr, double>;

/**
 * @brief Implementation of a discrete-conditioned hybrid factor.
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a component corresponding to a GaussianFactor.
 *
 * Represents the underlying hybrid Gaussian components as a Decision Tree,
 * where the set of discrete variables indexes to
 * the continuous gaussian distribution.
 *
 * In factor graphs the error function typically returns 0.5*|A*x - b|^2, i.e.,
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
class GTSAM_EXPORT HybridGaussianFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = HybridGaussianFactor;
  using shared_ptr = std::shared_ptr<This>;

  using sharedFactor = std::shared_ptr<GaussianFactor>;

  /// typedef for Decision Tree of Gaussian factors and arbitrary value.
  using FactorValuePairs = DecisionTree<Key, GaussianFactorValuePair>;
  /// typedef for Decision Tree of Gaussian factors.
  using Factors = DecisionTree<Key, sharedFactor>;

 private:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  Factors factors_;

  /**
   * @brief Helper function to return factors and functional to create a
   * DecisionTree of Gaussian Factor Graphs.
   *
   * @return GaussianFactorGraphTree
   */
  GaussianFactorGraphTree asGaussianFactorGraphTree() const;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor, mainly for serialization.
  HybridGaussianFactor() = default;

  /**
   * @brief Construct a new HybridGaussianFactor on a single discrete key,
   * providing the factors for each mode m as a vector of factors ϕ_m(x).
   * The value ϕ(x,m) for the factor is simply ϕ_m(x).
   *
   * @param continuousKeys Vector of keys for continuous factors.
   * @param discreteKey The discrete key for the "mode", indexing components.
   * @param factors Vector of gaussian factors, one for each mode.
   */
  HybridGaussianFactor(const KeyVector &continuousKeys,
                       const DiscreteKey &discreteKey,
                       const std::vector<GaussianFactor::shared_ptr> &factors)
      : Base(continuousKeys, {discreteKey}), factors_({discreteKey}, factors) {}

  /**
   * @brief Construct a new HybridGaussianFactor on a single discrete key,
   * including a scalar error value for each mode m. The factors and scalars are
   * provided as a vector of pairs (ϕ_m(x), E_m).
   * The value ϕ(x,m) for the factor is now ϕ_m(x) + E_m.
   *
   * @param continuousKeys Vector of keys for continuous factors.
   * @param discreteKey The discrete key for the "mode", indexing components.
   * @param factors Vector of gaussian factor-scalar pairs, one per mode.
   */
  HybridGaussianFactor(const KeyVector &continuousKeys,
                       const DiscreteKey &discreteKey,
                       const std::vector<GaussianFactorValuePair> &factors)
      : HybridGaussianFactor(continuousKeys, {discreteKey},
                             FactorValuePairs({discreteKey}, factors)) {}

  /**
   * @brief Construct a new HybridGaussianFactor on a several discrete keys M,
   * including a scalar error value for each assignment m. The factors and
   * scalars are provided as a DecisionTree<Key> of pairs (ϕ_M(x), E_M).
   * The value ϕ(x,M) for the factor is again ϕ_m(x) + E_m.
   *
   * @param continuousKeys A vector of keys representing continuous variables.
   * @param discreteKeys Discrete variables and their cardinalities.
   * @param factors The decision tree of Gaussian factor/scalar pairs.
   */
  HybridGaussianFactor(const KeyVector &continuousKeys,
                       const DiscreteKeys &discreteKeys,
                       const FactorValuePairs &factors);

  /// @}
  /// @name Testable
  /// @{

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(const std::string &s = "", const KeyFormatter &formatter =
                                            DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard API
  /// @{

  /// Get factor at a given discrete assignment.
  sharedFactor operator()(const DiscreteValues &assignment) const;

  /**
   * @brief Combine the Gaussian Factor Graphs in `sum` and `this` while
   * maintaining the original tree structure.
   *
   * @param sum Decision Tree of Gaussian Factor Graphs indexed by the
   * variables.
   * @return Sum
   */
  GaussianFactorGraphTree add(const GaussianFactorGraphTree &sum) const;

  /**
   * @brief Compute error of the HybridGaussianFactor as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factors involved, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues &continuousValues) const override;

  /**
   * @brief Compute the log-likelihood, including the log-normalizing constant.
   * @return double
   */
  double error(const HybridValues &values) const override;

  /// Getter for GaussianFactor decision tree
  const Factors &factors() const { return factors_; }

  /// Add HybridNonlinearFactor to a Sum, syntactic sugar.
  friend GaussianFactorGraphTree &operator+=(
      GaussianFactorGraphTree &sum, const HybridGaussianFactor &factor) {
    sum = factor.add(sum);
    return sum;
  }
  /// @}

 private:
  /// Helper method to compute the error of a component.
  double potentiallyPrunedComponentError(
      const sharedFactor &gf, const VectorValues &continuousValues) const;

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar &BOOST_SERIALIZATION_NVP(factors_);
  }
#endif
};

// traits
template <>
struct traits<HybridGaussianFactor> : public Testable<HybridGaussianFactor> {};

}  // namespace gtsam

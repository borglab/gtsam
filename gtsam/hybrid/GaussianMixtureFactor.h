/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureFactor.h
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

/**
 * @brief Implementation of a discrete conditional mixture factor.
 * Implements a joint discrete-continuous factor where the discrete variable
 * serves to "select" a mixture component corresponding to a GaussianFactor type
 * of measurement.
 *
 * Represents the underlying Gaussian Mixture as a Decision Tree, where the set
 * of discrete variables indexes to the continuous gaussian distribution.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT GaussianMixtureFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = GaussianMixtureFactor;
  using shared_ptr = boost::shared_ptr<This>;

  using sharedFactor = boost::shared_ptr<GaussianFactor>;

  /// Gaussian factor and log of normalizing constant.
  struct FactorAndConstant {
    sharedFactor factor;
    double constant;

    // Return error with constant correction.
    double error(const VectorValues &values) const {
      // Note: constant is log of normalization constant for probabilities.
      // Errors is the negative log-likelihood,
      // hence we subtract the constant here.
      if (!factor) return 0.0;  // If nullptr, return 0.0 error
      return factor->error(values) - constant;
    }

    // Check pointer equality.
    bool operator==(const FactorAndConstant &other) const {
      return factor == other.factor && constant == other.constant;
    }
  };

  /// typedef for Decision Tree of Gaussian factors and log-constant.
  using Factors = DecisionTree<Key, FactorAndConstant>;
  using Mixture = DecisionTree<Key, sharedFactor>;

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
  GaussianMixtureFactor() = default;

  /**
   * @brief Construct a new Gaussian Mixture Factor object.
   *
   * @param continuousKeys A vector of keys representing continuous variables.
   * @param discreteKeys A vector of keys representing discrete variables and
   * their cardinalities.
   * @param factors The decision tree of Gaussian factors stored as the mixture
   * density.
   */
  GaussianMixtureFactor(const KeyVector &continuousKeys,
                        const DiscreteKeys &discreteKeys,
                        const Mixture &factors);

  GaussianMixtureFactor(const KeyVector &continuousKeys,
                        const DiscreteKeys &discreteKeys,
                        const Factors &factors_and_z)
      : Base(continuousKeys, discreteKeys), factors_(factors_and_z) {}

  /**
   * @brief Construct a new GaussianMixtureFactor object using a vector of
   * GaussianFactor shared pointers.
   *
   * @param continuousKeys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Vector of gaussian factor shared pointers.
   */
  GaussianMixtureFactor(const KeyVector &continuousKeys,
                        const DiscreteKeys &discreteKeys,
                        const std::vector<sharedFactor> &factors)
      : GaussianMixtureFactor(continuousKeys, discreteKeys,
                              Mixture(discreteKeys, factors)) {}

  /// @}
  /// @name Testable
  /// @{

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(
      const std::string &s = "GaussianMixtureFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard API
  /// @{

  /// Get factor at a given discrete assignment.
  sharedFactor factor(const DiscreteValues &assignment) const;

  /// Get constant at a given discrete assignment.
  double constant(const DiscreteValues &assignment) const;

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
   * @brief Compute error of the GaussianMixtureFactor as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factors involved, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> error(const VectorValues &continuousValues) const;

  /**
   * @brief Compute the log-likelihood, including the log-normalizing constant.
   * @return double
   */
  double error(const HybridValues &values) const override;

  /// Add MixtureFactor to a Sum, syntactic sugar.
  friend GaussianFactorGraphTree &operator+=(
      GaussianFactorGraphTree &sum, const GaussianMixtureFactor &factor) {
    sum = factor.add(sum);
    return sum;
  }
  /// @}
};

// traits
template <>
struct traits<GaussianMixtureFactor> : public Testable<GaussianMixtureFactor> {
};

}  // namespace gtsam

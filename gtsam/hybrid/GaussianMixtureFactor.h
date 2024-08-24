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
 * Represents the underlying Gaussian mixture as a Decision Tree, where the set
 * of discrete variables indexes to the continuous gaussian distribution.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT GaussianMixtureFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = GaussianMixtureFactor;
  using shared_ptr = std::shared_ptr<This>;

  using sharedFactor = std::shared_ptr<GaussianFactor>;

  /// typedef for Decision Tree of Gaussian factors and log-constant.
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
  GaussianMixtureFactor() = default;

  /**
   * @brief Construct a new Gaussian mixture factor.
   *
   * @param continuousKeys A vector of keys representing continuous variables.
   * @param discreteKeys A vector of keys representing discrete variables and
   * their cardinalities.
   * @param factors The decision tree of Gaussian factors stored as the mixture
   * density.
   */
  GaussianMixtureFactor(const KeyVector &continuousKeys,
                        const DiscreteKeys &discreteKeys,
                        const Factors &factors);

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
                              Factors(discreteKeys, factors)) {}

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
   * @brief Compute error of the GaussianMixtureFactor as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factors involved, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues &continuousValues) const;

  /**
   * @brief Compute the log-likelihood, including the log-normalizing constant.
   * @return double
   */
  double error(const HybridValues &values) const override;

  /// Getter for GaussianFactor decision tree
  const Factors &factors() const { return factors_; }

  /// Add MixtureFactor to a Sum, syntactic sugar.
  friend GaussianFactorGraphTree &operator+=(
      GaussianFactorGraphTree &sum, const GaussianMixtureFactor &factor) {
    sum = factor.add(sum);
    return sum;
  }
  /// @}

 private:
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
struct traits<GaussianMixtureFactor> : public Testable<GaussianMixtureFactor> {
};

}  // namespace gtsam

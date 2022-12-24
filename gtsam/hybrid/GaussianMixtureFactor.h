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
#include <gtsam/discrete/DiscreteValues.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/VectorValues.h>

namespace gtsam {

class GaussianFactorGraph;

// Needed for wrapper.
using GaussianFactorVector = std::vector<gtsam::GaussianFactor::shared_ptr>;

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

  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  /// typedef for Decision Tree of Gaussian Factors
  using Factors = DecisionTree<Key, GaussianFactor::shared_ptr>;

 private:
  /// Decision tree of Gaussian factors indexed by discrete keys.
  Factors factors_;

  /**
   * @brief Helper function to return factors and functional to create a
   * DecisionTree of Gaussian Factor Graphs.
   *
   * @return Sum (DecisionTree<Key, GaussianFactorGraph>)
   */
  Sum asGaussianFactorGraphTree() const;

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
   * @param factors The decision tree of Gaussian Factors stored as the mixture
   * density.
   */
  GaussianMixtureFactor(const KeyVector &continuousKeys,
                        const DiscreteKeys &discreteKeys,
                        const Factors &factors);

  /**
   * @brief Construct a new GaussianMixtureFactor object using a vector of
   * GaussianFactor shared pointers.
   *
   * @param keys Vector of keys for continuous factors.
   * @param discreteKeys Vector of discrete keys.
   * @param factors Vector of gaussian factor shared pointers.
   */
  GaussianMixtureFactor(const KeyVector &keys, const DiscreteKeys &discreteKeys,
                        const std::vector<GaussianFactor::shared_ptr> &factors)
      : GaussianMixtureFactor(keys, discreteKeys,
                              Factors(discreteKeys, factors)) {}

  static This FromFactors(
      const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys,
      const std::vector<GaussianFactor::shared_ptr> &factors);

  /// @}
  /// @name Testable
  /// @{

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(
      const std::string &s = "GaussianMixtureFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;
  /// @}

  /// Getter for the underlying Gaussian Factor Decision Tree.
  const Factors &factors();

  /**
   * @brief Combine the Gaussian Factor Graphs in `sum` and `this` while
   * maintaining the original tree structure.
   *
   * @param sum Decision Tree of Gaussian Factor Graphs indexed by the
   * variables.
   * @return Sum
   */
  Sum add(const Sum &sum) const;

  /**
   * @brief Compute error of the GaussianMixtureFactor as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the factors involved, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> error(const VectorValues &continuousValues) const;

  /**
   * @brief Compute the error of this Gaussian Mixture given the continuous
   * values and a discrete assignment.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @param discreteValues The discrete assignment for a specific mode sequence.
   * @return double
   */
  double error(const VectorValues &continuousValues,
               const DiscreteValues &discreteValues) const;

  /// Add MixtureFactor to a Sum, syntactic sugar.
  friend Sum &operator+=(Sum &sum, const GaussianMixtureFactor &factor) {
    sum = factor.add(sum);
    return sum;
  }
};

// traits
template <>
struct traits<GaussianMixtureFactor> : public Testable<GaussianMixtureFactor> {
};

}  // namespace gtsam

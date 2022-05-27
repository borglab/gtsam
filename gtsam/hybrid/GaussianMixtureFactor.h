/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureFactor.h
 * @brief  A factor that is a function of discrete and continuous variables.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {

class GaussianFactorGraph;

using GaussianFactorVector = std::vector<gtsam::GaussianFactor::shared_ptr>;

/**
 * @brief A linear factor that is a function of both discrete and continuous
 * variables, i.e. P(X, M | Z) where X is the set of continuous variables, M is
 * the set of discrete variables and Z is the measurement set.
 *
 * Represents the underlying Gaussian Mixture as a Decision Tree, where the set
 * of discrete variables indexes to the continuous gaussian distribution.
 *
 */
class GaussianMixtureFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = GaussianMixtureFactor;
  using shared_ptr = boost::shared_ptr<This>;

  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  /// typedef for Decision Tree of Gaussian Factors
  using Factors = DecisionTree<Key, GaussianFactor::shared_ptr>;

 private:
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

  static This FromFactors(
      const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys,
      const std::vector<GaussianFactor::shared_ptr> &factors);

  /// @}
  /// @name Testable
  /// @{

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(
      const std::string &s = "HybridFactor\n",
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
};

}  // namespace gtsam

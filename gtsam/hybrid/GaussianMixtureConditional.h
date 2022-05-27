/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureConditional.h
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @author Varun Agrawal
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

/**
 * @brief A conditional of gaussian mixtures indexed by discrete variables.
 *
 * Represents the conditional density P(X | M, Z) where X is a continuous random
 * variable, M is the discrete variable and Z is the set of measurements.
 *
 */
class GaussianMixtureConditional
    : public HybridFactor,
      public Conditional<HybridFactor, GaussianMixtureConditional> {
 public:
  using This = GaussianMixtureConditional;
  using shared_ptr = boost::shared_ptr<GaussianMixtureConditional>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, GaussianMixtureConditional>;

  /// Alias for DecisionTree of GaussianFactorGraphs
  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  /// typedef for Decision Tree of Gaussian Conditionals
  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

 private:
  Conditionals conditionals_;

  /**
   * @brief Convert a DecisionTree of factors into a DT of Gaussian FGs.
   */
  Sum asGaussianFactorGraphTree() const;

 public:
  /// @name Constructors
  /// @{

  /// Defaut constructor, mainly for serialization.
  GaussianMixtureConditional() = default;
  /**
   * @brief Construct a new GaussianMixtureConditional object
   *
   * @param continuousFrontals the continuous frontals.
   * @param continuousParents the continuous parents.
   * @param discreteParents the discrete parents. Will be placed last.
   * @param conditionals a decision tree of GaussianConditionals.
   */
  GaussianMixtureConditional(const KeyVector &continuousFrontals,
                             const KeyVector &continuousParents,
                             const DiscreteKeys &discreteParents,
                             const Conditionals &conditionals);

  /**
   * @brief Make a Gaussian Mixture from a list of Gaussian conditionals
   *
   * @param continuousFrontals The continuous frontal variables
   * @param continuousParents The continuous parent variables
   * @param discreteParents Discrete parents variables
   * @param conditionals List of conditionals
   */
  static This FromConditionals(
      const KeyVector &continuousFrontals, const KeyVector &continuousParents,
      const DiscreteKeys &discreteParents,
      const std::vector<GaussianConditional::shared_ptr> &conditionals);

  /// @}
  /// @name Testable
  /// @{

  /// Test equality with base HybridFactor
  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  /* print utility */
  void print(
      const std::string &s = "GaussianMixtureConditional\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}

  /// Getter for the underlying Conditionals DecisionTree
  const Conditionals &conditionals();

  /**
   * @brief Merge the Gaussian Factor Graphs in `this` and `sum` while
   * maintaining the decision tree structure.
   *
   * @param sum Decision Tree of Gaussian Factor Graphs
   * @return Sum
   */
  Sum add(const Sum &sum) const;
};

}  // namespace gtsam

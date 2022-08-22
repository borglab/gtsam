/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixture.h
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @author Varun Agrawal
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/DecisionTree-inl.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

/**
 * @brief A conditional of gaussian mixtures indexed by discrete variables, as
 * part of a Bayes Network.
 *
 * Represents the conditional density P(X | M, Z) where X is a continuous random
 * variable, M is the selection of discrete variables corresponding to a subset
 * of the Gaussian variables and Z is parent of this node
 *
 * The probability P(x|y,z,...) is proportional to
 * \f$ \sum_i k_i \exp - \frac{1}{2} |R_i x - (d_i - S_i y - T_i z - ...)|^2 \f$
 * where i indexes the components and k_i is a component-wise normalization
 * constant.
 *
 */
class GTSAM_EXPORT GaussianMixture
    : public HybridFactor,
      public Conditional<HybridFactor, GaussianMixture> {
 public:
  using This = GaussianMixture;
  using shared_ptr = boost::shared_ptr<GaussianMixture>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, GaussianMixture>;

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
  GaussianMixture() = default;

  /**
   * @brief Construct a new GaussianMixture object.
   *
   * @param continuousFrontals the continuous frontals.
   * @param continuousParents the continuous parents.
   * @param discreteParents the discrete parents. Will be placed last.
   * @param conditionals a decision tree of GaussianConditionals. The number of
   * conditionals should be C^(number of discrete parents), where C is the
   * cardinality of the DiscreteKeys in discreteParents, since the
   * discreteParents will be used as the labels in the decision tree.
   */
  GaussianMixture(const KeyVector &continuousFrontals,
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
  /// @name Standard API
  /// @{

  GaussianConditional::shared_ptr operator()(
      const DiscreteValues &discreteVals) const;

  /// Returns the total number of continuous components
  size_t nrComponents() const;

  /// @}
  /// @name Testable
  /// @{

  /// Test equality with base HybridFactor
  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  /* print utility */
  void print(
      const std::string &s = "GaussianMixture\n",
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

// traits
template <>
struct traits<GaussianMixture> : public Testable<GaussianMixture> {};

}  // namespace gtsam

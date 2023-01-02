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
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

class HybridValues;

/**
 * @brief A conditional of gaussian mixtures indexed by discrete variables, as
 * part of a Bayes Network. This is the result of the elimination of a
 * continuous variable in a hybrid scheme, such that the remaining variables are
 * discrete+continuous.
 *
 * Represents the conditional density P(X | M, Z) where X is the set of
 * continuous random variables, M is the selection of discrete variables
 * corresponding to a subset of the Gaussian variables and Z is parent of this
 * node .
 *
 * The probability P(x|y,z,...) is proportional to
 * \f$ \sum_i k_i \exp - \frac{1}{2} |R_i x - (d_i - S_i y - T_i z - ...)|^2 \f$
 * where i indexes the components and k_i is a component-wise normalization
 * constant.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT GaussianMixture
    : public HybridFactor,
      public Conditional<HybridFactor, GaussianMixture> {
 public:
  using This = GaussianMixture;
  using shared_ptr = boost::shared_ptr<GaussianMixture>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, GaussianMixture>;

  /// typedef for Decision Tree of Gaussian Conditionals
  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

 private:
  Conditionals conditionals_;

  /**
   * @brief Convert a DecisionTree of factors into a DT of Gaussian FGs.
   */
  GaussianFactorGraphTree asGaussianFactorGraphTree() const;

  /**
   * @brief Helper function to get the pruner functor.
   *
   * @param decisionTree The pruned discrete probability decision tree.
   * @return std::function<GaussianConditional::shared_ptr(
   * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
   */
  std::function<GaussianConditional::shared_ptr(
      const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
  prunerFunc(const DecisionTreeFactor &decisionTree);

 public:
  /// @name Constructors
  /// @{

  /// Default constructor, mainly for serialization.
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
  GaussianMixture(
      const KeyVector &continuousFrontals, const KeyVector &continuousParents,
      const DiscreteKeys &discreteParents,
      const std::vector<GaussianConditional::shared_ptr> &conditionals);

  /// @}
  /// @name Testable
  /// @{

  /// Test equality with base HybridFactor
  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  /// Print utility
  void print(
      const std::string &s = "GaussianMixture\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard API
  /// @{

  /// @brief Return the conditional Gaussian for the given discrete assignment.
  GaussianConditional::shared_ptr operator()(
      const DiscreteValues &discreteValues) const;

  /// Returns the total number of continuous components
  size_t nrComponents() const;

  /// Returns the continuous keys among the parents.
  KeyVector continuousParents() const;

  // Create a likelihood factor for a Gaussian mixture, return a Mixture factor
  // on the parents.
  boost::shared_ptr<GaussianMixtureFactor> likelihood(
      const VectorValues &frontals) const;

  /// Getter for the underlying Conditionals DecisionTree
  const Conditionals &conditionals() const;

  /**
   * @brief Compute error of the GaussianMixture as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the conditionals, and leaf values as the error.
   */
  AlgebraicDecisionTree<Key> error(const VectorValues &continuousValues) const;

  /**
   * @brief Compute the error of this Gaussian Mixture given the continuous
   * values and a discrete assignment.
   *
   * @param values Continuous values and discrete assignment.
   * @return double
   */
  double error(const HybridValues &values) const override;

  //   /// Calculate probability density for given values `x`.
  //   double evaluate(const HybridValues &values) const;

  //   /// Evaluate probability density, sugar.
  //   double operator()(const HybridValues &values) const { return
  //   evaluate(values); }

  //   /// Calculate log-density for given values `x`.
  //   double logDensity(const HybridValues &values) const;

  /**
   * @brief Prune the decision tree of Gaussian factors as per the discrete
   * `decisionTree`.
   *
   * @param decisionTree A pruned decision tree of discrete keys where the
   * leaves are probabilities.
   */
  void prune(const DecisionTreeFactor &decisionTree);

  /**
   * @brief Merge the Gaussian Factor Graphs in `this` and `sum` while
   * maintaining the decision tree structure.
   *
   * @param sum Decision Tree of Gaussian Factor Graphs
   * @return GaussianFactorGraphTree
   */
  GaussianFactorGraphTree add(const GaussianFactorGraphTree &sum) const;
  /// @}
};

/// Return the DiscreteKey vector as a set.
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &discreteKeys);

// traits
template <>
struct traits<GaussianMixture> : public Testable<GaussianMixture> {};

}  // namespace gtsam

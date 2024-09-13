/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridGaussianConditional.h
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
#include <gtsam/hybrid/HybridGaussianFactor.h>
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
class GTSAM_EXPORT HybridGaussianConditional
    : public HybridFactor,
      public Conditional<HybridFactor, HybridGaussianConditional> {
 public:
  using This = HybridGaussianConditional;
  using shared_ptr = std::shared_ptr<HybridGaussianConditional>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, HybridGaussianConditional>;

  /// typedef for Decision Tree of Gaussian Conditionals
  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

 private:
  Conditionals conditionals_;  ///< a decision tree of Gaussian conditionals.
  double logConstant_;         ///< log of the normalization constant.

  /**
   * @brief Convert a HybridGaussianConditional of conditionals into
   * a DecisionTree of Gaussian factor graphs.
   */
  GaussianFactorGraphTree asGaussianFactorGraphTree() const;

  /**
   * @brief Helper function to get the pruner functor.
   *
   * @param discreteProbs The pruned discrete probabilities.
   * @return std::function<GaussianConditional::shared_ptr(
   * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
   */
  std::function<GaussianConditional::shared_ptr(
      const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
  prunerFunc(const DecisionTreeFactor &discreteProbs);

 public:
  /// @name Constructors
  /// @{

  /// Default constructor, mainly for serialization.
  HybridGaussianConditional() = default;

  /**
   * @brief Construct a new HybridGaussianConditional object.
   *
   * @param continuousFrontals the continuous frontals.
   * @param continuousParents the continuous parents.
   * @param discreteParents the discrete parents. Will be placed last.
   * @param conditionals a decision tree of GaussianConditionals. The number of
   * conditionals should be C^(number of discrete parents), where C is the
   * cardinality of the DiscreteKeys in discreteParents, since the
   * discreteParents will be used as the labels in the decision tree.
   */
  HybridGaussianConditional(const KeyVector &continuousFrontals,
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
  HybridGaussianConditional(KeyVector &&continuousFrontals, KeyVector &&continuousParents,
                  DiscreteKeys &&discreteParents,
                  std::vector<GaussianConditional::shared_ptr> &&conditionals);

  /**
   * @brief Make a Gaussian Mixture from a list of Gaussian conditionals
   *
   * @param continuousFrontals The continuous frontal variables
   * @param continuousParents The continuous parent variables
   * @param discreteParents Discrete parents variables
   * @param conditionals List of conditionals
   */
  HybridGaussianConditional(
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
      const std::string &s = "HybridGaussianConditional\n",
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

  /// The log normalization constant is max of the the individual
  /// log-normalization constants.
  double logNormalizationConstant() const override { return logConstant_; }

  /**
   * Create a likelihood factor for a Gaussian mixture, return a Mixture factor
   * on the parents.
   */
  std::shared_ptr<HybridGaussianFactor> likelihood(
      const VectorValues &given) const;

  /// Getter for the underlying Conditionals DecisionTree
  const Conditionals &conditionals() const;

  /**
   * @brief Compute logProbability of the HybridGaussianConditional as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree with the same keys
   * as the conditionals, and leaf values as the logProbability.
   */
  AlgebraicDecisionTree<Key> logProbability(
      const VectorValues &continuousValues) const;

  /**
   * @brief Compute the error of this Gaussian Mixture.
   *
   * This requires some care, as different mixture components may have
   * different normalization constants. Let's consider p(x|y,m), where m is
   * discrete. We need the error to satisfy the invariant:
   *
   *    error(x;y,m) = K - log(probability(x;y,m))
   *
   * For all x,y,m. But note that K, the (log) normalization constant defined
   * in Conditional.h, should not depend on x, y, or m, only on the parameters
   * of the density. Hence, we delegate to the underlying Gaussian
   * conditionals, indexed by m, which do satisfy:
   *
   *    log(probability_m(x;y)) = K_m - error_m(x;y)
   *
   * We resolve by having K == max(K_m) and
   *
   *    error(x;y,m) = error_m(x;y) + K - K_m
   *
   * which also makes error(x;y,m) >= 0 for all x,y,m.
   *
   * @param values Continuous values and discrete assignment.
   * @return double
   */
  double error(const HybridValues &values) const override;

  /**
   * @brief Compute error of the HybridGaussianConditional as a tree.
   *
   * @param continuousValues The continuous VectorValues.
   * @return AlgebraicDecisionTree<Key> A decision tree on the discrete keys
   * only, with the leaf values as the error for each assignment.
   */
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues &continuousValues) const;

  /**
   * @brief Compute the logProbability of this Gaussian Mixture.
   *
   * @param values Continuous values and discrete assignment.
   * @return double
   */
  double logProbability(const HybridValues &values) const override;

  /// Calculate probability density for given `values`.
  double evaluate(const HybridValues &values) const override;

  /// Evaluate probability density, sugar.
  double operator()(const HybridValues &values) const {
    return evaluate(values);
  }

  /**
   * @brief Prune the decision tree of Gaussian factors as per the discrete
   * `discreteProbs`.
   *
   * @param discreteProbs A pruned set of probabilities for the discrete keys.
   */
  void prune(const DecisionTreeFactor &discreteProbs);

  /**
   * @brief Merge the Gaussian Factor Graphs in `this` and `sum` while
   * maintaining the decision tree structure.
   *
   * @param sum Decision Tree of Gaussian Factor Graphs
   * @return GaussianFactorGraphTree
   */
  GaussianFactorGraphTree add(const GaussianFactorGraphTree &sum) const;
  /// @}

 private:
  /// Check whether `given` has values for all frontal keys.
  bool allFrontalsGiven(const VectorValues &given) const;

  /// Helper method to compute the error of a conditional.
  double conditionalError(const GaussianConditional::shared_ptr &conditional,
                          const VectorValues &continuousValues) const;

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseFactor);
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(BaseConditional);
    ar &BOOST_SERIALIZATION_NVP(conditionals_);
  }
#endif
};

/// Return the DiscreteKey vector as a set.
std::set<DiscreteKey> DiscreteKeysAsSet(const DiscreteKeys &discreteKeys);

// traits
template <>
struct traits<HybridGaussianConditional> : public Testable<HybridGaussianConditional> {};

}  // namespace gtsam

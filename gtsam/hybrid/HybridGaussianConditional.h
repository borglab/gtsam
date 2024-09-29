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
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

class HybridValues;

/**
 * @brief A conditional of gaussian conditionals indexed by discrete variables,
 * as part of a Bayes Network. This is the result of the elimination of a
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
    : public HybridGaussianFactor,
      public Conditional<HybridGaussianFactor, HybridGaussianConditional> {
 public:
  using This = HybridGaussianConditional;
  using shared_ptr = std::shared_ptr<This>;
  using BaseFactor = HybridGaussianFactor;
  using BaseConditional = Conditional<BaseFactor, HybridGaussianConditional>;

  /// typedef for Decision Tree of Gaussian Conditionals
  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

 private:
  Conditionals conditionals_;  ///< a decision tree of Gaussian conditionals.

  ///< Negative-log of the normalization constant (log(\sqrt(|2πΣ|))).
  ///< Take advantage of the neg-log space so everything is a minimization
  double negLogConstant_;

 public:
  /// @name Constructors
  /// @{

  /// Default constructor, mainly for serialization.
  HybridGaussianConditional() = default;

  /**
   * @brief Construct from one discrete key and vector of conditionals.
   *
   * @param mode Single discrete parent variable
   * @param conditionals Vector of conditionals with the same size as the
   * cardinality of the discrete parent.
   */
  HybridGaussianConditional(
      const DiscreteKey &mode,
      const std::vector<GaussianConditional::shared_ptr> &conditionals);

  /**
   * @brief Construct from vector of means and sigmas.
   *
   * @param x The continuous key.
   * @param mode The discrete key.
   * @param parameters The means and sigmas for the Gaussian conditionals.
   */
  HybridGaussianConditional(
      Key x, const DiscreteKey mode,
      const std::vector<std::pair<Vector, double>> &parameters);

  /**
   * @brief Construct from multiple discrete keys and conditional tree.
   *
   * @param discreteParents the discrete parents. Will be placed last.
   * @param conditionals a decision tree of GaussianConditionals. The number of
   * conditionals should be C^(number of discrete parents), where C is the
   * cardinality of the DiscreteKeys in discreteParents, since the
   * discreteParents will be used as the labels in the decision tree.
   */
  HybridGaussianConditional(const DiscreteKeys &discreteParents,
                            const Conditionals &conditionals);

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

  /**
   * @brief Return log normalization constant in negative log space.
   *
   * The log normalization constant is the min of the individual
   * log-normalization constants.
   *
   * @return double
   */
  inline double negLogConstant() const override { return negLogConstant_; }

  /**
   * Create a likelihood factor for a hybrid Gaussian conditional,
   * return a hybrid Gaussian factor on the parents.
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
   * @brief Compute the logProbability of this hybrid Gaussian conditional.
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

  /// @}

 private:
  /// Helper struct for private constructor.
  struct ConstructorHelper;

  /// Private constructor that uses helper struct above.
  HybridGaussianConditional(const DiscreteKeys &discreteParents,
                            const ConstructorHelper &helper);

  /// Convert to a DecisionTree of Gaussian factor graphs.
  GaussianFactorGraphTree asGaussianFactorGraphTree() const;

  /**
   * @brief Get the pruner function from discrete probabilities.
   *
   * @param discreteProbs The probabilities of only discrete keys.
   * @return std::function<GaussianConditional::shared_ptr(
   * const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
   */
  std::function<GaussianConditional::shared_ptr(
      const Assignment<Key> &, const GaussianConditional::shared_ptr &)>
  prunerFunc(const DecisionTreeFactor &prunedProbabilities);

  /// Check whether `given` has values for all frontal keys.
  bool allFrontalsGiven(const VectorValues &given) const;

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
struct traits<HybridGaussianConditional>
    : public Testable<HybridGaussianConditional> {};

}  // namespace gtsam

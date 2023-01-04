/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesNet.h
 * @brief   A Bayes net of Gaussian Conditionals indexed by discrete keys.
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#pragma once

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/global_includes.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridValues.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/linear/GaussianBayesNet.h>

namespace gtsam {

/**
 * A hybrid Bayes net is a collection of HybridConditionals, which can have
 * discrete conditionals, Gaussian mixtures, or pure Gaussian conditionals.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridBayesNet : public BayesNet<HybridConditional> {
 public:
  using Base = BayesNet<HybridConditional>;
  using This = HybridBayesNet;
  using ConditionalType = HybridConditional;
  using shared_ptr = boost::shared_ptr<HybridBayesNet>;
  using sharedConditional = boost::shared_ptr<ConditionalType>;

  /// @name Standard Constructors
  /// @{

  /** Construct empty Bayes net */
  HybridBayesNet() = default;

  /// @}
  /// @name Testable
  /// @{

  /** Check equality */
  bool equals(const This &bn, double tol = 1e-9) const {
    return Base::equals(bn, tol);
  }

  /// print graph
  void print(
      const std::string &s = "",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override {
    Base::print(s, formatter);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// Add HybridConditional to Bayes Net
  using Base::add;

  /// Add a Gaussian Mixture to the Bayes Net.
  void addMixture(const GaussianMixture::shared_ptr &ptr) {
    push_back(HybridConditional(ptr));
  }

  /// Add a Gaussian conditional to the Bayes Net.
  void addGaussian(const GaussianConditional::shared_ptr &ptr) {
    push_back(HybridConditional(ptr));
  }

  /// Add a discrete conditional to the Bayes Net.
  void addDiscrete(const DiscreteConditional::shared_ptr &ptr) {
    push_back(HybridConditional(ptr));
  }

  /// Add a Gaussian Mixture to the Bayes Net.
  template <typename... T>
  void emplaceMixture(T &&...args) {
    push_back(HybridConditional(
        boost::make_shared<GaussianMixture>(std::forward<T>(args)...)));
  }

  /// Add a Gaussian conditional to the Bayes Net.
  template <typename... T>
  void emplaceGaussian(T &&...args) {
    push_back(HybridConditional(
        boost::make_shared<GaussianConditional>(std::forward<T>(args)...)));
  }

  /// Add a discrete conditional to the Bayes Net.
  template <typename... T>
  void emplaceDiscrete(T &&...args) {
    push_back(HybridConditional(
        boost::make_shared<DiscreteConditional>(std::forward<T>(args)...)));
  }

  using Base::push_back;

  /// Get a specific Gaussian mixture by index `i`.
  GaussianMixture::shared_ptr atMixture(size_t i) const;

  /// Get a specific Gaussian conditional by index `i`.
  GaussianConditional::shared_ptr atGaussian(size_t i) const;

  /// Get a specific discrete conditional by index `i`.
  DiscreteConditional::shared_ptr atDiscrete(size_t i) const;

  /**
   * @brief Get the Gaussian Bayes Net which corresponds to a specific discrete
   * value assignment.
   *
   * @param assignment The discrete value assignment for the discrete keys.
   * @return GaussianBayesNet
   */
  GaussianBayesNet choose(const DiscreteValues &assignment) const;

  /// Evaluate hybrid probability density for given HybridValues.
  double evaluate(const HybridValues &values) const;

  /// Evaluate hybrid probability density for given HybridValues, sugar.
  double operator()(const HybridValues &values) const {
    return evaluate(values);
  }

  /**
   * @brief Solve the HybridBayesNet by first computing the MPE of all the
   * discrete variables and then optimizing the continuous variables based on
   * the MPE assignment.
   *
   * @return HybridValues
   */
  HybridValues optimize() const;

  /**
   * @brief Given the discrete assignment, return the optimized estimate for the
   * selected Gaussian BayesNet.
   *
   * @param assignment An assignment of discrete values.
   * @return Values
   */
  VectorValues optimize(const DiscreteValues &assignment) const;

  /**
   * @brief Get all the discrete conditionals as a decision tree factor.
   *
   * @return DecisionTreeFactor::shared_ptr
   */
  DecisionTreeFactor::shared_ptr discreteConditionals() const;

  /**
   * @brief Sample from an incomplete BayesNet, given missing variables.
   *
   * Example:
   *   std::mt19937_64 rng(42);
   *   VectorValues given = ...;
   *   auto sample = bn.sample(given, &rng);
   *
   * @param given Values of missing variables.
   * @param rng The pseudo-random number generator.
   * @return HybridValues
   */
  HybridValues sample(const HybridValues &given, std::mt19937_64 *rng) const;

  /**
   * @brief Sample using ancestral sampling.
   *
   * Example:
   *   std::mt19937_64 rng(42);
   *   auto sample = bn.sample(&rng);
   *
   * @param rng The pseudo-random number generator.
   * @return HybridValues
   */
  HybridValues sample(std::mt19937_64 *rng) const;

  /**
   * @brief Sample from an incomplete BayesNet, use default rng.
   *
   * @param given Values of missing variables.
   * @return HybridValues
   */
  HybridValues sample(const HybridValues &given) const;

  /**
   * @brief Sample using ancestral sampling, use default rng.
   *
   * @return HybridValues
   */
  HybridValues sample() const;

  /// Prune the Hybrid Bayes Net such that we have at most maxNrLeaves leaves.
  HybridBayesNet prune(size_t maxNrLeaves);

  /**
   * @brief 0.5 * sum of squared Mahalanobis distances
   * for a specific discrete assignment.
   *
   * @param values Continuous values and discrete assignment.
   * @return double
   */
  double error(const HybridValues &values) const;

  /**
   * @brief Compute conditional error for each discrete assignment,
   * and return as a tree.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> error(const VectorValues &continuousValues) const;

  /**
   * @brief Compute unnormalized probability q(μ|M),
   * for each discrete assignment, and return as a tree.
   * q(μ|M) is the unnormalized probability at the MLE point μ,
   * conditioned on the discrete variables.
   *
   * @param continuousValues Continuous values at which to compute the
   * probability.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> probPrime(
      const VectorValues &continuousValues) const;

  /// @}

 private:
  /**
   * @brief Update the discrete conditionals with the pruned versions.
   *
   * @param prunedDecisionTree
   */
  void updateDiscreteConditionals(
      const DecisionTreeFactor::shared_ptr &prunedDecisionTree);

  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

/// traits
template <>
struct traits<HybridBayesNet> : public Testable<HybridBayesNet> {};

}  // namespace gtsam

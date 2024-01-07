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
  using shared_ptr = std::shared_ptr<HybridBayesNet>;
  using sharedConditional = std::shared_ptr<ConditionalType>;

  /// @name Standard Constructors
  /// @{

  /** Construct empty Bayes net */
  HybridBayesNet() = default;

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style printing
  void print(const std::string &s = "", const KeyFormatter &formatter =
                                            DefaultKeyFormatter) const override;

  /// GTSAM-style equals
  bool equals(const This &fg, double tol = 1e-9) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /**
   * @brief Add a hybrid conditional using a shared_ptr.
   *
   * This is the "native" push back, as this class stores hybrid conditionals.
   */
  void push_back(std::shared_ptr<HybridConditional> conditional) {
    factors_.push_back(conditional);
  }

  /**
   * Preferred: add a conditional directly using a pointer.
   *
   * Examples:
   *   hbn.emplace_back(new GaussianMixture(...)));
   *   hbn.emplace_back(new GaussianConditional(...)));
   *   hbn.emplace_back(new DiscreteConditional(...)));
   */
  template <class Conditional>
  void emplace_back(Conditional *conditional) {
    factors_.push_back(std::make_shared<HybridConditional>(
        std::shared_ptr<Conditional>(conditional)));
  }

  /**
   * Add a conditional using a shared_ptr, using implicit conversion to
   * a HybridConditional.
   *
   * This is useful when you create a conditional shared pointer as you need it
   * somewhere else.
   *
   * Example:
   *   auto shared_ptr_to_a_conditional =
   *     std::make_shared<GaussianMixture>(...);
   *  hbn.push_back(shared_ptr_to_a_conditional);
   */
  void push_back(HybridConditional &&conditional) {
    factors_.push_back(
        std::make_shared<HybridConditional>(std::move(conditional)));
  }

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
   * @brief Assemble a DecisionTree of (GaussianBayesNet, double) leaves for
   * each discrete assignment.
   * The included double value is used to make
   * constructing the model selection term cleaner and more efficient.
   *
   * @return GaussianBayesNetValTree
   */
  GaussianBayesNetValTree assembleTree() const;

  /*
    Compute L(M;Z), the likelihood of the discrete model M
    given the measurements Z.
    This is called the model selection term.

    To do so, we perform the integration of L(M;Z) ∝ L(X;M,Z)P(X|M).

    By Bayes' rule, P(X|M,Z) ∝ L(X;M,Z)P(X|M),
    hence L(X;M,Z)P(X|M) is the unnormalized probabilty of
    the joint Gaussian distribution.

    This can be computed by multiplying all the exponentiated errors
    of each of the conditionals.
    
    Return a tree where each leaf value is L(M_i;Z).
  */
  AlgebraicDecisionTree<Key> modelSelection() const;

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
   * @brief Compute conditional error for each discrete assignment,
   * and return as a tree.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues &continuousValues) const;

  /**
   * @brief Error method using HybridValues which returns specific error for
   * assignment.
   */
  using Base::error;

  /**
   * @brief Compute log probability for each discrete assignment,
   * and return as a tree.
   *
   * @param continuousValues Continuous values at which
   * to compute the log probability.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> logProbability(
      const VectorValues &continuousValues) const;

  using BayesNet::logProbability;  // expose HybridValues version

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
  AlgebraicDecisionTree<Key> evaluate(
      const VectorValues &continuousValues) const;

  /**
   * Convert a hybrid Bayes net to a hybrid Gaussian factor graph by converting
   * all conditionals with instantiated measurements into likelihood factors.
   */
  HybridGaussianFactorGraph toFactorGraph(
      const VectorValues &measurements) const;
  /// @}

 private:
  /**
   * @brief Prune all the discrete conditionals.
   *
   * @param maxNrLeaves
   */
  DecisionTreeFactor pruneDiscreteConditionals(size_t maxNrLeaves);

#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

/// traits
template <>
struct traits<HybridBayesNet> : public Testable<HybridBayesNet> {};

}  // namespace gtsam

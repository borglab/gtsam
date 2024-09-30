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
 * discrete conditionals, hybrid Gaussian conditionals,
 * or pure Gaussian conditionals.
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

  /// Construct empty Bayes net.
  HybridBayesNet() = default;

  /// Constructor that takes an initializer list of shared pointers.
  HybridBayesNet(
      std::initializer_list<HybridConditional::shared_ptr> conditionals)
      : Base(conditionals) {}

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
   * Add a conditional using a shared_ptr, using implicit conversion to
   * a HybridConditional.
   *
   * This is useful when you create a conditional shared pointer as you need it
   * somewhere else.
   *
   * Example:
   *   auto shared_ptr_to_a_conditional =
   *     std::make_shared<HybridGaussianConditional>(...);
   *  hbn.push_back(shared_ptr_to_a_conditional);
   */
  void push_back(HybridConditional &&conditional) {
    factors_.push_back(
        std::make_shared<HybridConditional>(std::move(conditional)));
  }

  /**
   * @brief Add a conditional to the Bayes net.
   * Implicitly convert to a HybridConditional.
   *
   * E.g.
   * hbn.push_back(std::make_shared<DiscreteConditional>(m, "1/1"));
   *
   * @tparam CONDITIONAL Type of conditional. This is shared_ptr version.
   * @param conditional The conditional as a shared pointer.
   */
  template <class CONDITIONAL>
  void push_back(const std::shared_ptr<CONDITIONAL> &conditional) {
    factors_.push_back(std::make_shared<HybridConditional>(conditional));
  }

  /**
   * Preferred: Emplace a conditional directly using arguments.
   *
   * Examples:
   *   hbn.emplace_shared<HybridGaussianConditional>(...)));
   *   hbn.emplace_shared<GaussianConditional>(...)));
   *   hbn.emplace_shared<DiscreteConditional>(...)));
   */
  template <class CONDITIONAL, class... Args>
  void emplace_shared(Args &&...args) {
    auto cond = std::allocate_shared<CONDITIONAL>(
        Eigen::aligned_allocator<CONDITIONAL>(), std::forward<Args>(args)...);
    factors_.push_back(std::make_shared<HybridConditional>(std::move(cond)));
  }

  /**
   * @brief Get the Gaussian Bayes Net which corresponds to a specific discrete
   * value assignment. Note this corresponds to the Gaussian posterior p(X|M=m)
   * of the continuous variables given the discrete assignment M=m.
   *
   * @note Be careful, as any factors not Gaussian are ignored.
   *
   * @param assignment The discrete value assignment for the discrete keys.
   * @return Gaussian posterior as a GaussianBayesNet
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

  /**
   * @brief Prune the Bayes Net such that we have at most maxNrLeaves leaves.
   *
   * @param maxNrLeaves Continuous values at which to compute the error.
   * @return A pruned HybridBayesNet
   */
  HybridBayesNet prune(size_t maxNrLeaves) const;

  /**
   * @brief Error method using HybridValues which returns specific error for
   * assignment.
   */
  using Base::error;

  /**
   * @brief Compute the negative log posterior log P'(M|x) of all assignments up
   * to a constant, returning the result as an algebraic decision tree.
   *
   * @note The joint P(X,M) is p(X|M) P(M)
   * Then the posterior on M given X=x is is P(M|x) = p(x|M) P(M) / p(x).
   * Ideally we want log P(M|x) = log p(x|M) + log P(M) - log P(x), but
   * unfortunately log p(x) is expensive, so we compute the log of the
   * unnormalized posterior log P'(M|x) = log p(x|M) + log P(M)
   *
   * @param continuousValues Continuous values x at which to compute log P'(M|x)
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> errorTree(
      const VectorValues &continuousValues) const;

  using BayesNet::logProbability;  // expose HybridValues version

  /**
   * @brief Compute normalized posterior P(M|X=x) and return as a tree.
   *
   * @note Not a DiscreteConditional as the cardinalities of the DiscreteKeys,
   * which we would need, are hard to recover.
   *
   * @param continuousValues Continuous values x to condition P(M|X=x) on.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> discretePosterior(
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

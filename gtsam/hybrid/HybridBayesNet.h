/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesNet.h
 * @brief   A bayes net of Gaussian Conditionals indexed by discrete keys.
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

  /** Construct empty bayes net */
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

  /// Add a discrete conditional to the Bayes Net.
  void add(const DiscreteKey &key, const std::string &table) {
    push_back(
        HybridConditional(boost::make_shared<DiscreteConditional>(key, table)));
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

 protected:
  /**
   * @brief Get all the discrete conditionals as a decision tree factor.
   *
   * @return DecisionTreeFactor::shared_ptr
   */
  DecisionTreeFactor::shared_ptr discreteConditionals() const;

 public:
  /// Prune the Hybrid Bayes Net such that we have at most maxNrLeaves leaves.
  HybridBayesNet prune(size_t maxNrLeaves) const;

  /**
   * @brief 0.5 * sum of squared Mahalanobis distances
   * for a specific discrete assignment.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @param discreteValues Discrete assignment for a specific mode sequence.
   * @return double
   */
  double error(const VectorValues &continuousValues,
               const DiscreteValues &discreteValues) const;

  /**
   * @brief Compute conditional error for each discrete assignment,
   * and return as a tree.
   *
   * @param continuousValues Continuous values at which to compute the error.
   * @return AlgebraicDecisionTree<Key>
   */
  AlgebraicDecisionTree<Key> error(const VectorValues &continuousValues) const;

  /// @}

 private:
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

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

  /** Construct empty bayes net */
  HybridBayesNet() = default;

  /// Prune the Hybrid Bayes Net given the discrete decision tree.
  HybridBayesNet prune(
      const DecisionTreeFactor::shared_ptr &discreteFactor) const;

  /// Add HybridConditional to Bayes Net
  using Base::add;

  /// Add a discrete conditional to the Bayes Net.
  void add(const DiscreteKey &key, const std::string &table) {
    push_back(
        HybridConditional(boost::make_shared<DiscreteConditional>(key, table)));
  }

  /// Get a specific Gaussian mixture by index `i`.
  GaussianMixture::shared_ptr atGaussian(size_t i) const;

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

  /// Solve the HybridBayesNet by back-substitution.
  /// TODO(Shangjie) do we need to create a HybridGaussianBayesNet class, and
  /// put this method there?
  HybridValues optimize() const;

  /**
   * @brief Given the discrete assignment, return the optimized estimate for the
   * selected Gaussian BayesNet.
   *
   * @param assignment An assignment of discrete values.
   * @return Values
   */
  VectorValues optimize(const DiscreteValues &assignment) const;
};

}  // namespace gtsam

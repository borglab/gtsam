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

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>

#include <iostream>  // TODO!

namespace gtsam {

/**
 * A hybrid Bayes net can have discrete conditionals, Gaussian mixtures,
 * or pure Gaussian conditionals.
 */
class GTSAM_EXPORT HybridBayesNet : public BayesNet<AbstractConditional> {
 public:
  using Base = BayesNet<AbstractConditional>;
  using This = HybridBayesNet;
  using ConditionalType = AbstractConditional;
  using shared_ptr = boost::shared_ptr<HybridBayesNet>;
  using sharedConditional = boost::shared_ptr<ConditionalType>;

  /** Construct empty bayes net */
  HybridBayesNet() : Base() {}

  void add(const DiscreteKey &key, const std::string &table) {
    DiscreteConditional dc(key, table);
    // TODO(fan): implement this method
    push_back(dc);
  }

  /**
   * Get a specific Gaussian mixture factor by index
   * (this checks array bounds and may throw an exception, as opposed to
   * operator[] which does not).
   */
  GaussianMixture::shared_ptr atGaussian(size_t i) const;

  /**
   * @brief Set gaussian mixture at specific index.
   *
   * @param i The index to set the gaussian mixture at.
   * @param gaussian The gaussian mixture shared pointer to add.
   */
  void setGaussian(size_t i, const GaussianMixture::shared_ptr &gaussian);

  /**
   * Get a specific Gaussian mixture factor by index
   * (this checks array bounds and may throw an exception, as opposed to
   * operator[] which does not).
   */
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
   * @brief Prune the conditionals in the bayes net as per the pruning
   * structure in `discreteFactor`.
   *
   * @param discreteFactor A DecisionTreeFactor shared pointer which has a
   * pruned tree structure.
   * @return HybridBayesNet::shared_ptr
   */
  shared_ptr prune(const DecisionTreeFactor::shared_ptr &discreteFactor) const;
};

}  // namespace gtsam

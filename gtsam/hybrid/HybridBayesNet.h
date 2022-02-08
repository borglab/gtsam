/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    HybridBayesNet.h
 * @brief   A set of GaussianFactors, indexed by a set of discrete keys.
 * @author  Varun Agrawal
 * @author  Fan Jiang
 * @author  Frank Dellaert
 * @date    December 2021
 */

#pragma once

#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/linear/GaussianConditional.h>

#include <iostream>  // TODO!

namespace gtsam {

/**
 * A hybrid Bayes net can have discrete conditionals, Gaussian mixtures,
 * or pure Gaussian conditionals.
 */
class GTSAM_EXPORT HybridBayesNet : public BayesNet<AbstractConditional> {
 public:
  using ConditionalType = AbstractConditional;
  using shared_ptr = boost::shared_ptr<HybridBayesNet>;

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
  GaussianMixture::shared_ptr atGaussian(size_t i);

  /**
   * Get a specific Gaussian mixture factor by index
   * (this checks array bounds and may throw an exception, as opposed to
   * operator[] which does not).
   */
  DiscreteConditional::shared_ptr atDiscrete(size_t i);

};

}  // namespace gtsam

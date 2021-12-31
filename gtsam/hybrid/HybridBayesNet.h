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
#include <gtsam/hybrid/DCConditional.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/linear/GaussianConditional.h>

#include <iostream>  // TODO!

namespace gtsam {

/// Bayes net
class HybridBayesNet : public BayesNet<DCConditional> {
 public:
  using ConditionalType = DCConditional;
  using shared_ptr = boost::shared_ptr<HybridBayesNet>;
};

}  // namespace gtsam

/* ----------------------------------------------------------------------------
 * Copyright 2021 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   CGMixtureFactor.h
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Varun Agrawal
 * @author Fan Jiang
 * @author Frank Dellaert
 * @date   December 2021
 */

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>

namespace gtsam {

class CGMixtureFactor : public HybridFactor {
public:
  using Base = HybridFactor;
  using This = CGMixtureFactor;
  using shared_ptr = boost::shared_ptr<This>;

  using Factors = DecisionTree<Key, GaussianFactor::shared_ptr>;

  Factors factors_;

  CGMixtureFactor() = default;

  CGMixtureFactor(const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys, const Factors &factors) : Base(continuousKeys, discreteKeys) {

  }
};

}

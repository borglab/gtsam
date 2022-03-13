/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   CGMixtureFactor.h
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
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

  CGMixtureFactor(const KeyVector &continuousKeys,
                  const DiscreteKeys &discreteKeys,
                  const Factors &factors);

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(const std::string &s = "HybridFactor\n",
             const KeyFormatter &formatter = DefaultKeyFormatter) const override;
};

}

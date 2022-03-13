/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   CLGaussianConditional.h
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @date   Mar 12, 2022
 */

#include <gtsam/inference/Conditional.h>
#include <gtsam/hybrid/HybridFactor.h>

#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/discrete/DecisionTree.h>

namespace gtsam {
class CLGaussianConditional : public HybridFactor, public Conditional<HybridFactor, CLGaussianConditional> {
public:
  using This = CLGaussianConditional;
  using shared_ptr = boost::shared_ptr<CLGaussianConditional>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, CLGaussianConditional>;

  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

public:

  CLGaussianConditional(const KeyVector &continuousFrontals,
                        const KeyVector &continuousParents,
                        const DiscreteKeys &discreteParents,
                        const Conditionals &factors);

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(
      const std::string &s = "CLGaussianConditional\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;
};
}
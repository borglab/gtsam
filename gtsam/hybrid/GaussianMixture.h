/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixture.h
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {
class GaussianMixture
    : public HybridFactor,
      public Conditional<HybridFactor, GaussianMixture> {
 public:
  using This = GaussianMixture;
  using shared_ptr = boost::shared_ptr<GaussianMixture>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, GaussianMixture>;

  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

  Conditionals conditionals_;

 public:
  GaussianMixture(const KeyVector &continuousFrontals,
                        const KeyVector &continuousParents,
                        const DiscreteKeys &discreteParents,
                        const Conditionals &conditionals);

  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  const Conditionals& conditionals();

  /* *******************************************************************************/
  Sum addTo(const Sum &sum) const;

  /* *******************************************************************************/
  Sum wrappedConditionals() const;

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(
      const std::string &s = "GaussianMixture\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;
};
}  // namespace gtsam
/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureFactor.h
 * @brief  A set of Gaussian factors indexed by a set of discrete keys.
 * @author Fan Jiang
 * @author Varun Agrawal
 * @author Frank Dellaert
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/linear/GaussianFactor.h>

namespace gtsam {

class GaussianFactorGraph;

typedef std::vector<gtsam::GaussianFactor::shared_ptr> GaussianFactorVector;

class GaussianMixtureFactor : public HybridFactor {
 public:
  using Base = HybridFactor;
  using This = GaussianMixtureFactor;
  using shared_ptr = boost::shared_ptr<This>;

  using Factors = DecisionTree<Key, GaussianFactor::shared_ptr>;

  Factors factors_;

  GaussianMixtureFactor() = default;

  GaussianMixtureFactor(const KeyVector &continuousKeys,
                        const DiscreteKeys &discreteKeys,
                        const Factors &factors);

  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  const Factors &factors();

  static This FromFactorList(
      const KeyVector &continuousKeys, const DiscreteKeys &discreteKeys,
      const std::vector<GaussianFactor::shared_ptr> &factors);

  /* *******************************************************************************/
  Sum addTo(const Sum &sum) const;

  /* *******************************************************************************/
  Sum wrappedFactors() const;

  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

  void print(
      const std::string &s = "HybridFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;
};

}  // namespace gtsam

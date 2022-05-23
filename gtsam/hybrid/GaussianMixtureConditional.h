/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixtureConditional.h
 * @brief  A hybrid conditional in the Conditional Linear Gaussian scheme
 * @author Fan Jiang
 * @author Varun Agrawal
 * @date   Mar 12, 2022
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {
class GaussianMixtureConditional : public HybridFactor,
                        public Conditional<HybridFactor, GaussianMixtureConditional> {
 public:
  using This = GaussianMixtureConditional;
  using shared_ptr = boost::shared_ptr<GaussianMixtureConditional>;
  using BaseFactor = HybridFactor;
  using BaseConditional = Conditional<HybridFactor, GaussianMixtureConditional>;

  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;

  Conditionals conditionals_;

 public:
  /**
   * @brief Construct a new Gaussian Mixture object
   *
   * @param continuousFrontals the continuous frontals.
   * @param continuousParents the continuous parents.
   * @param discreteParents the discrete parents. Will be placed last.
   * @param conditionals a decision tree of GaussianConditionals.
   */
  GaussianMixtureConditional(const KeyVector &continuousFrontals,
                  const KeyVector &continuousParents,
                  const DiscreteKeys &discreteParents,
                  const Conditionals &conditionals);

  using Sum = DecisionTree<Key, GaussianFactorGraph>;

  const Conditionals &conditionals();

  /**
   * @brief Combine Decision Trees
   */
  Sum add(const Sum &sum) const;

	/**
	 * @brief Convert a DecisionTree of factors into a DT of Gaussian FGs.
	 */
  Sum asGraph() const;

	/**
	 * @brief Make a Gaussian Mixture from a list of Gaussian conditionals
	 * 
	 * @param continuousFrontals The continuous frontal variables
	 * @param continuousParents The continuous parent variables
	 * @param discreteParents Discrete parents variables
	 * @param conditionals List of conditionals
	 */
  static This FromConditionalList(
      const KeyVector &continuousFrontals, const KeyVector &continuousParents,
      const DiscreteKeys &discreteParents,
      const std::vector<GaussianConditional::shared_ptr> &conditionals);

	/* TODO: this is only a stub */
  bool equals(const HybridFactor &lf, double tol = 1e-9) const override;

	/* print utility */
  void print(
      const std::string &s = "GaussianMixtureConditional\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;
};
}  // namespace gtsam

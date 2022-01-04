/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   GaussianMixture.h
 * @brief  Discrete-continuous conditional density
 * @author Frank Dellaert
 * @date   December 2021
 */

#pragma once

#include <gtsam/discrete/DecisionTree.h>
#include <gtsam/discrete/DiscreteKey.h>
#include <gtsam/hybrid/DCGaussianMixtureFactor.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

/**
 * @brief A discrete-continuous conditional.
 *
 * keys_ member variable stores keys for *continuous* variables.
 * discreteKeys_ contains the keys (plus cardinalities) for *discrete*
 * variables.
 */
class GaussianMixture
    : public DCGaussianMixtureFactor,
      public Conditional<DCGaussianMixtureFactor, GaussianMixture> {
 protected:
  // Set of DiscreteKeys for this factor.
  DiscreteKeys discreteKeys_;

 public:
  using This = GaussianMixture;
  using shared_ptr = boost::shared_ptr<This>;
  using BaseFactor = DCGaussianMixtureFactor;
  using BaseConditional = Conditional<BaseFactor, This>;

  // Decision trees
  using Conditionals = DecisionTree<Key, GaussianConditional::shared_ptr>;
  using Factors = DCGaussianMixtureFactor::Factors;

  /// @name Constructors
  /// @{

  /// Default constructor.
  GaussianMixture() = default;

  /**
   * @brief Construct a new GaussianMixture object.
   * @param conditionals A decision tree of GaussianConditional instances.
   * TODO(Frank): (possibly wrongly) assumes nrFrontals is one
   */
  GaussianMixture(const KeyVector& keys, const DiscreteKeys& discreteKeys,
                  const Conditionals& conditionals)
      : BaseFactor(
            keys, discreteKeys,
            Factors(conditionals,
                    [](const GaussianConditional::shared_ptr& p) {
                      return boost::dynamic_pointer_cast<GaussianFactor>(p);
                    })),
        BaseConditional(1) {}

  /// @}
  /// @name Standard API
  /// @{

  GaussianConditional::shared_ptr operator()(
      DiscreteValues& discreteVals) const {
    auto conditional = boost::dynamic_pointer_cast<GaussianConditional>(
        factors_(discreteVals));
    if (conditional)
      return conditional;
    else
      throw std::logic_error(
          "A GaussianMixture unexpectedly contained a non-conditional");
  }
  /// @}
  /// @name Testable
  /// @{

  void print(
      const std::string& s = "GaussianMixture",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override {
    DCGaussianMixtureFactor::print(s, keyFormatter);
  }

  /// @}
};
}  // namespace gtsam

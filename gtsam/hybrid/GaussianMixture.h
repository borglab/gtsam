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
   * @param nrFrontals - the number of frontal keys
   * @param continuousKeys - the keys for *continuous* variables
   * @param discreteKeys - the keys for *discrete* variables
   * @param conditionals A decision tree of GaussianConditional instances.
   * TODO(Frank): (possibly wrongly) assumes nrFrontals is one
   * TODO(Frank): should pass frontal keys, cont. parent keys, discrete parent
   * keys, cannot be a conditional on a discrete key.
   * TODO: desired API =
   * GaussianConditionalMixture(const Conditionals& conditionals,
   *                            const DiscreteKeys& discreteParentKeys)
   */
  GaussianMixture(size_t nrFrontals, const KeyVector& continuousKeys,
                  const DiscreteKeys& discreteKeys,
                  const Conditionals& conditionals);

  /// @}
  /// @name Standard API
  /// @{

  GaussianConditional::shared_ptr operator()(
      DiscreteValues& discreteVals) const;

  /// @}
  /// @name Testable
  /// @{

  void print(
      const std::string& s = "GaussianMixture",
      const KeyFormatter& keyFormatter = DefaultKeyFormatter) const override;

  bool equals(const DCFactor& f, double tol) const override;

  /// @}
};

/// traits
template <>
struct traits<GaussianMixture> : public Testable<GaussianMixture> {};

}  // namespace gtsam

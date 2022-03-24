/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridConditional.h
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#pragma once

#include <gtsam/discrete/DiscreteConditional.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/inference/Conditional.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>
#include "gtsam/hybrid/GaussianMixture.h"

#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianConditional.h>

namespace gtsam {

class HybridFactorGraph;

/**
 * Hybrid Conditional Density
 *
 * As a type-erased variant of:
 * - DiscreteConditional
 * - GaussianConditional
 * - GaussianMixture
 */
class GTSAM_EXPORT HybridConditional
    : public HybridFactor,
      public Conditional<HybridFactor, HybridConditional> {
 public:
  // typedefs needed to play nice with gtsam
  typedef HybridConditional This;              ///< Typedef to this class
  typedef boost::shared_ptr<This> shared_ptr;  ///< shared_ptr to this class
  typedef HybridFactor BaseFactor;  ///< Typedef to our factor base class
  typedef Conditional<BaseFactor, This>
      BaseConditional;  ///< Typedef to our conditional base class

 protected:
  // Type-erased pointer to the inner type
  boost::shared_ptr<Factor> inner;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  HybridConditional() = default;

  HybridConditional(const KeyVector& continuousKeys,
                    const DiscreteKeys& discreteKeys, size_t nFrontals)
      : BaseFactor(continuousKeys, discreteKeys), BaseConditional(nFrontals) {}

  HybridConditional(const KeyVector& continuousFrontals,
                    const DiscreteKeys& discreteFrontals,
                    const KeyVector& continuousParents,
                    const DiscreteKeys& discreteParents);

  HybridConditional(boost::shared_ptr<GaussianConditional> continuousConditional);

  HybridConditional(boost::shared_ptr<DiscreteConditional> discreteConditional);
  
  HybridConditional(boost::shared_ptr<GaussianMixture> gaussianMixture);

  GaussianMixture::shared_ptr asMixture() {
      if (!isHybrid_) throw std::invalid_argument("Not a mixture");
      return boost::static_pointer_cast<GaussianMixture>(inner);
  }

  /// @}
  /// @name Testable
  /// @{

  /// GTSAM-style print
  void print(
      const std::string& s = "Hybrid Conditional: ",
      const KeyFormatter& formatter = DefaultKeyFormatter) const override;

  /// GTSAM-style equals
  bool equals(const HybridFactor& other, double tol = 1e-9) const override;

  /// @}

  friend std::pair<HybridConditional::shared_ptr, HybridFactor::shared_ptr>  //
  EliminateHybrid(const HybridFactorGraph& factors,
                  const Ordering& frontalKeys);
};
// DiscreteConditional

// traits
template <>
struct traits<HybridConditional> : public Testable<DiscreteConditional> {};

}  // namespace gtsam

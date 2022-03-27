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
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/inference/Conditional.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/GaussianConditional.h>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include <gtsam/hybrid/GaussianMixture.h>

namespace gtsam {

class HybridFactorGraph;

/**
 * Hybrid Conditional Density
 *
 * As a type-erased variant of:
 * - DiscreteConditional
 * - GaussianConditional
 * - GaussianMixture
 *
 * The reason why this is important is that `Conditional<T>` is a CRTP class.
 * CRTP is static polymorphism such that all CRTP classes, while bearing the
 * same name, are different classes not sharing a vtable. This prevents them
 * from being contained in any container, and thus it is impossible to
 * dynamically cast between them. A better option, as illustrated here, is
 * treating them as an implementation detail - such that the hybrid mechanism
 * does not know what is inside the HybridConditional. This prevents us from
 * having diamond inheritances, and neutralized the need to change other
 * components of GTSAM to make hybrid elimination work.
 *
 * A great reference to the type-erasure pattern is Edurado Madrid's CppCon
 * talk.
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

  HybridConditional(
      boost::shared_ptr<GaussianConditional> continuousConditional);

  HybridConditional(boost::shared_ptr<DiscreteConditional> discreteConditional);

  HybridConditional(boost::shared_ptr<GaussianMixture> gaussianMixture);

  GaussianMixture::shared_ptr asMixture() {
    if (!isHybrid_) throw std::invalid_argument("Not a mixture");
    return boost::static_pointer_cast<GaussianMixture>(inner);
  }

  boost::shared_ptr<Factor> getInner() {
      return inner;
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

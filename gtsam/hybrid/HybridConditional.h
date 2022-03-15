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
#include <string>
#include <vector>

namespace gtsam {

/**
 * Hybrid Conditional Density
 *
 * As a type-erased variant of:
 * - DiscreteConditional
 * - CLGaussianConditional
 * - GaussianConditional
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

 private:
  // Type-erased pointer to the inner type
  std::unique_ptr<Factor> inner;

 public:
  /// @name Standard Constructors
  /// @{

  /// Default constructor needed for serialization.
  HybridConditional() = default;

  HybridConditional(const KeyVector& continuousKeys,
                    const DiscreteKeys& discreteKeys, size_t nFrontals)
      : BaseFactor(continuousKeys, discreteKeys), BaseConditional(nFrontals) {}

  /**
   * @brief Combine two conditionals, yielding a new conditional with the union
   * of the frontal keys, ordered by gtsam::Key.
   *
   * The two conditionals must make a valid Bayes net fragment, i.e.,
   * the frontal variables cannot overlap, and must be acyclic:
   * Example of correct use:
   *   P(A,B) = P(A|B) * P(B)
   *   P(A,B|C) = P(A|B) * P(B|C)
   *   P(A,B,C) = P(A,B|C) * P(C)
   * Example of incorrect use:
   *   P(A|B) * P(A|C) = ?
   *   P(A|B) * P(B|A) = ?
   * We check for overlapping frontals, but do *not* check for cyclic.
   */
  HybridConditional operator*(const HybridConditional& other) const;

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
};
// DiscreteConditional

// traits
template <>
struct traits<HybridConditional> : public Testable<DiscreteConditional> {};

}  // namespace gtsam

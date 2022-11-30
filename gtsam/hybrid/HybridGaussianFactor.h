/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridGaussianFactor.h
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#pragma once

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/linear/JacobianFactor.h>

namespace gtsam {

/**
 * A HybridGaussianFactor is a layer over GaussianFactor so that we do not have
 * a diamond inheritance i.e. an extra factor type that inherits from both
 * HybridFactor and GaussianFactor.
 *
 * @ingroup hybrid
 */
class GTSAM_EXPORT HybridGaussianFactor : public HybridFactor {
 private:
  GaussianFactor::shared_ptr inner_;

 public:
  using Base = HybridFactor;
  using This = HybridGaussianFactor;
  using shared_ptr = boost::shared_ptr<This>;

  HybridGaussianFactor() = default;

  // Explicit conversion from a shared ptr of GF
  explicit HybridGaussianFactor(GaussianFactor::shared_ptr other);

  // Forwarding constructor from concrete JacobianFactor
  explicit HybridGaussianFactor(JacobianFactor &&jf);

 public:
  /// @name Testable
  /// @{

  /// Check equality.
  virtual bool equals(const HybridFactor &lf, double tol) const override;

  /// GTSAM print utility.
  void print(
      const std::string &s = "HybridGaussianFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}

  GaussianFactor::shared_ptr inner() const { return inner_; }
};

// traits
template <>
struct traits<HybridGaussianFactor> : public Testable<HybridGaussianFactor> {};

}  // namespace gtsam

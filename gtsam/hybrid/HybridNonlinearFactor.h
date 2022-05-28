/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file HybridNonlinearFactor.h
 *  @date May 28, 2022
 *  @author Varun Agrawal
 */

#pragma once

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

/**
 * A HybridNonlinearFactor is a layer over NonlinearFactor so that we do not
 * have a diamond inheritance.
 */
class HybridNonlinearFactor : public HybridFactor {
 private:
  NonlinearFactor::shared_ptr inner_;

 public:
  using Base = HybridFactor;
  using This = HybridNonlinearFactor;
  using shared_ptr = boost::shared_ptr<This>;

  // Explicit conversion from a shared ptr of GF
  explicit HybridNonlinearFactor(NonlinearFactor::shared_ptr other);

 public:
  /// @name Testable
  /// @{

  /// Check equality.
  virtual bool equals(const HybridFactor &lf, double tol) const override;

  /// GTSAM print utility.
  void print(
      const std::string &s = "HybridNonlinearFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}

  NonlinearFactor::shared_ptr inner() const { return inner_; }
};
}  // namespace gtsam

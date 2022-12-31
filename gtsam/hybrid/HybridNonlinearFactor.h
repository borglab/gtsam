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
#include <gtsam/hybrid/HybridGaussianFactor.h>
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

  // Explicit conversion from a shared ptr of NonlinearFactor
  explicit HybridNonlinearFactor(const NonlinearFactor::shared_ptr &other);

  /// @name Testable
  /// @{

  /// Check equality.
  virtual bool equals(const HybridFactor &lf, double tol) const override;

  /// GTSAM print utility.
  void print(
      const std::string &s = "HybridNonlinearFactor\n",
      const KeyFormatter &formatter = DefaultKeyFormatter) const override;

  /// @}
  /// @name Standard Interface
  /// @{

  NonlinearFactor::shared_ptr inner() const { return inner_; }

  /// Error for HybridValues is not provided for nonlinear factor.
  double error(const HybridValues &values) const override {
    throw std::runtime_error(
        "HybridNonlinearFactor::error(HybridValues) not implemented.");
  }

  /// Linearize to a HybridGaussianFactor at the linearization point `c`.
  boost::shared_ptr<HybridGaussianFactor> linearize(const Values &c) const {
    return boost::make_shared<HybridGaussianFactor>(inner_->linearize(c));
  }

  /// @}
};
}  // namespace gtsam

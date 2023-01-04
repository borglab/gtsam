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

namespace gtsam {

// Forward declarations
class JacobianFactor;
class HessianFactor;
class HybridValues;

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

  /**
   * Constructor from shared_ptr of GaussianFactor.
   * Example:
   *  auto ptr = boost::make_shared<JacobianFactor>(...);
   *  HybridGaussianFactor factor(ptr);
   */
  explicit HybridGaussianFactor(const boost::shared_ptr<GaussianFactor> &ptr);

  /**
   * Forwarding constructor from shared_ptr of GaussianFactor.
   * Examples:
   *   HybridGaussianFactor factor = boost::make_shared<JacobianFactor>(...);
   *   HybridGaussianFactor factor(boost::make_shared<JacobianFactor>(...));
   */
  explicit HybridGaussianFactor(boost::shared_ptr<GaussianFactor> &&ptr);

  /**
   * Forwarding constructor from rvalue reference of JacobianFactor.
   *
   * Examples:
   *   HybridGaussianFactor factor = JacobianFactor(...);
   *   HybridGaussianFactor factor(JacobianFactor(...));
   */
  explicit HybridGaussianFactor(JacobianFactor &&jf);

  /**
   * Forwarding constructor from rvalue reference of JacobianFactor.
   *
   * Examples:
   *   HybridGaussianFactor factor = HessianFactor(...);
   *   HybridGaussianFactor factor(HessianFactor(...));
   */
  explicit HybridGaussianFactor(HessianFactor &&hf);

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
  /// @name Standard Interface
  /// @{

  /// Return pointer to the internal Gaussian factor.
  GaussianFactor::shared_ptr inner() const { return inner_; }

  /// Return the error of the underlying Gaussian factor.
  double error(const HybridValues &values) const override;
  /// @}
};

// traits
template <>
struct traits<HybridGaussianFactor> : public Testable<HybridGaussianFactor> {};

}  // namespace gtsam

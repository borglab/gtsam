/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    QpCost.h
 * @brief   Affine quadratic cost factor for QP and QCQP problems.
 * @author  Frank Dellaert
 */

#pragma once

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <stdexcept>

namespace gtsam {

/**
 * Nonlinear wrapper around a Gaussian-factor-defined affine quadratic cost over
 * direct Vector and Matrix Values entries.
 *
 * Matrix entries are interpreted by column-major vectorization. The
 * linearization is exact for Vector and Matrix values.
 */
class GTSAM_EXPORT QpCost : public NonlinearFactor {
 public:
  using Base = NonlinearFactor;
  using This = QpCost;
  using shared_ptr = std::shared_ptr<This>;

  /** Default constructor for I/O. */
  QpCost() = default;

  /** Construct from an affine quadratic Hessian factor. */
  explicit QpCost(const HessianFactor& factor)
      : Base(factor.keys()),
        hessianFactor_(std::make_shared<HessianFactor>(factor)) {}

  /**
   * Construct from a Gaussian factor, retaining its square-root representation
   * when available for sparse constrained solves and gradient evaluation.
   */
  explicit QpCost(const GaussianFactor& factor)
      : QpCost(HessianFactor(factor)) {
    if (const auto* jacobian = dynamic_cast<const JacobianFactor*>(&factor))
      jacobianFactor_ = std::make_shared<JacobianFactor>(*jacobian);
  }

  /**
   * Construct from a shared Gaussian factor, copying its original representation.
   */
  explicit QpCost(const GaussianFactor::shared_ptr& factor)
      : QpCost(factor ? *factor
                      : throw std::invalid_argument(
                            "QpCost: shared Gaussian factor is null.")) {}

  /**
   * Construct a row-space quadratic cost over direct Vector or Matrix Values.
   *
   * For matrix values X_i in R^{r_i x d}, this creates the cost
   * 0.5 * sum_ij trace(X_i' Q_ij X_j). Matrix values must use the supplied
   * columnDim. Vector QPs use columnDim = 1.
   */
  QpCost(const KeyVector& keys, const SymmetricBlockMatrix& Q,
         size_t columnDim = 1);

  /// Return the original Gaussian representation of the affine quadratic.
  const GaussianFactor& gaussianFactor() const {
    if (jacobianFactor_) return *jacobianFactor_;
    return *hessianFactor_;
  }

  /// Return the stored Hessian factor.
  const HessianFactor& hessianFactor() const { return *hessianFactor_; }

  /** Print the factor for debugging. */
  void print(const std::string& s = "", const KeyFormatter& formatter =
                                            DefaultKeyFormatter) const override;

  /** Check equality up to a tolerance. */
  bool equals(const NonlinearFactor& other, double tol = 1e-9) const override;

  /** Evaluate the original Gaussian factor at the direct Vector/Matrix values. */
  double error(const Values& values) const override;

  /** Return scalar cost dimension. */
  size_t dim() const override { return 1; }

  /** Return an exact Gaussian factor around the current vector-space values. */
  GaussianFactor::shared_ptr linearize(const Values& values) const override;

  /** Return a deep copy of this factor. */
  NonlinearFactor::shared_ptr clone() const override {
    return NonlinearFactor::shared_ptr(new This(*this));
  }

 private:
  HessianFactor::shared_ptr hessianFactor_;
  JacobianFactor::shared_ptr jacobianFactor_;
};

}  // namespace gtsam

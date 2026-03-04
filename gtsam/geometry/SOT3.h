/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SOT3.h
 * @brief   The scaled orthogonal transforms SOT(3) = SO(3) x R>0
 *
 * This file implements the SOT(3) group as defined by eqVIO (van Goor et al., https://arxiv.org/pdf/2205.01980)
 *
 * @author  Rohan Bansal
 * @date    2026
 */

#pragma once

#include <gtsam/base/MatrixLieGroup.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/SO3.h>

#include <string>

namespace gtsam {

/**
 * @class SOT3
 * @brief Scaled orthogonal transforms: SOT(3) = SO(3) x R>0.
 *
 * Represents group elements Q = (R_Q, c_Q) with R_Q in SO(3) and c_Q > 0.
 * The 4x4 matrix representation is block-diagonal: [[R, 0], [0, c]].
 * The Lie algebra has dimension 4: tangent vector xi = (Omega, s) in R^4.
 */
class GTSAM_EXPORT SOT3 : public MatrixLieGroup<SOT3, 4, 4> {
 public:
  /// @name Type definitions
  /// @{

  static constexpr int dimension = 4;
  using MatrixNN = Eigen::Matrix4d;
  using LieAlgebra = MatrixNN;
  using TangentVector = Eigen::Matrix<double, 4, 1>;
  using ChartJacobian = OptionalJacobian<4, 4>;
  using Jacobian = Eigen::Matrix<double, 4, 4>;

  /// @}
  /// @name Constructors
  /// @{

  /// Default constructor: identity element (R = I_3, c = 1).
  SOT3() : R_(), c_(1.0) {}

  /// Construct from rotation and positive scale.
  SOT3(const SO3& R, double c) : R_(R), c_(c) {}

  /// Construct 4x4 [[R, 0], [0, c]].
  explicit SOT3(const MatrixNN& M)
      : R_(SO3(M.topLeftCorner<3, 3>())), c_(M(3, 3)) {}

  /// @}
  /// @name Testable
  /// @{

  void print(const std::string& s = "") const;
  bool equals(const SOT3& other, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// Identity element.
  static SOT3 Identity() { return SOT3(); }

  /// Inverse element
  SOT3 inverse() const { return SOT3(R_.inverse(), 1.0 / c_); }

  /// Group multiplication: (R1,c1)*(R2,c2) = (R1*R2, c1*c2).
  SOT3 operator*(const SOT3& other) const {
    return SOT3(R_ * other.R_, c_ * other.c_);
  }

  /// @}
  /// @name MatrixLieGroup interface
  /// @{

  /// Return 4x4 [[R, 0], [0, c]].
  MatrixNN matrix() const;

  /// Hat: (Omega, s) -> [[Omega^x, 0], [0, s]].
  static MatrixNN Hat(const TangentVector& xi);

  /// Vee: inverse of Hat.
  static TangentVector Vee(const MatrixNN& X);

  /// Exponential map at identity.
  static SOT3 Expmap(const TangentVector& xi, ChartJacobian H = {});

  /// Logarithmic map at identity.
  static TangentVector Logmap(const SOT3& Q, ChartJacobian H = {});

  /// Adjoint map Ad_Q: block-diagonal [[R, 0], [0, 1]].
  Jacobian AdjointMap() const;

  // Chart at origin uses Expmap/Logmap directly.
  struct ChartAtOrigin {
    static SOT3 Retract(const TangentVector& xi, ChartJacobian H = {});
    static TangentVector Local(const SOT3& Q, ChartJacobian H = {});
  };

  using LieGroup<SOT3, 4>::inverse;

  /// @}
  /// @name Group action on R^3
  /// @{

  /// Apply Q to point p in R^3: Qp = c * R * p.
  Vector3 operator*(const Vector3& p) const { return c_ * (R_.matrix() * p); }

  /// Apply inverse of Q to point p: Q^{-1}p = (1/c) * R^T * p.
  Vector3 applyInverse(const Vector3& p) const {
    return (1.0 / c_) * (R_.matrix().transpose() * p);
  }

  /// @}
  /// @name Accessors
  /// @{

  /// Return R_Q, the SO(3) rotation component.
  const SO3& rotation() const { return R_; }

  /// Return c_Q, the positive scale component.
  double scalar() const { return c_; }

  /// @}

 private:
  SO3 R_;       ///< Rotation component R_Q in SO(3).
  double c_;    ///< Scale component c_Q > 0.
};

template <>
struct traits<SOT3> : public internal::MatrixLieGroup<SOT3, 4> {};

template <>
struct traits<const SOT3> : public internal::MatrixLieGroup<SOT3, 4> {};

}  // namespace gtsam

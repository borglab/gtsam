/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    SOT3.cpp
 * @brief   The scaled orthogonal transforms SOT(3) = SO(3) x R>0
 * @author  Rohan Bansal
 */

#include <gtsam/geometry/SOT3.h>

#include <cmath>
#include <iostream>

namespace gtsam {

void SOT3::print(const std::string& s) const {
  if (!s.empty()) std::cout << s << "\n";
  std::cout << "SOT3:\n";
  R_.print("  R: ");
  std::cout << "  c: " << c_ << "\n";
}

bool SOT3::equals(const SOT3& other, double tol) const {
  return R_.equals(other.R_, tol) && std::abs(c_ - other.c_) < tol;
}

// MatrixLieGroup interface

SOT3::MatrixNN SOT3::matrix() const {
  MatrixNN M = MatrixNN::Zero();
  M.topLeftCorner<3, 3>() = R_.matrix();
  M(3, 3) = c_;
  return M;
}

SOT3::MatrixNN SOT3::Hat(const TangentVector& xi) {
  // (Omega, s) -> [[Omega^x, 0], [0, s]]
  MatrixNN X = MatrixNN::Zero();
  X.topLeftCorner<3, 3>() = SO3::Hat(Vector3(xi.head<3>()));
  X(3, 3) = xi(3);
  return X;
}

SOT3::TangentVector SOT3::Vee(const MatrixNN& X) {
  // [[Omega^x, 0], [0, s]] -> (Omega, s)
  TangentVector xi;
  xi.head<3>() = SO3::Vee(Matrix3(X.topLeftCorner<3, 3>()));
  xi(3) = X(3, 3);
  return xi;
}

//******************************************************************************
// Lie Group

SOT3 SOT3::Expmap(const TangentVector& xi, ChartJacobian H) {
  // SOT(3) = SO(3) x R>0 is direct product, so:
  // exp(Omega, s) = (SO3::Expmap(Omega), exp(s))
  const Vector3 Omega = xi.head<3>();
  const double s = xi(3);
  const double c = std::exp(s);

  if (H) {
    // block-diagonal Jacobian where top-left 3x3: right Jacobian of 
    // SO3::Expmap at Omega, and bottom-right: identity, since 
    // Local(exp(s), exp(s + ds)) = ds
    Matrix3 H_R;
    const SO3 R = SO3::Expmap(Omega, H ? &H_R : nullptr);
    H->setZero();
    H->topLeftCorner<3, 3>() = H_R;
    (*H)(3, 3) = 1.0;
    return SOT3(R, c);
  }

  return SOT3(SO3::Expmap(Omega), c);
}

SOT3::TangentVector SOT3::Logmap(const SOT3& Q, ChartJacobian H) {
  // log(R, c) = (SO3::Logmap(R), log(c))
  const double s = std::log(Q.c_);

  if (H) {
    // block-diagonal Jacobian where top-left 3x3: derivative of 
    // SO3::Logmap at R, and bottom-right: identity, since 
    // Local(exp(s), exp(s + ds)) = ds
    Matrix3 H_R;
    const Vector3 Omega = SO3::Logmap(Q.R_, H ? &H_R : nullptr);
    H->setZero();
    H->topLeftCorner<3, 3>() = H_R;
    (*H)(3, 3) = 1.0;
    TangentVector xi;
    xi.head<3>() = Omega;
    xi(3) = s;
    return xi;
  }

  TangentVector xi;
  xi.head<3>() = SO3::Logmap(Q.R_);
  xi(3) = s;
  return xi;
}

SOT3::Jacobian SOT3::AdjointMap() const {
  Jacobian Ad = Jacobian::Identity();
  Ad.topLeftCorner<3, 3>() = R_.matrix();
  return Ad;
}

SOT3 SOT3::ChartAtOrigin::Retract(const TangentVector& xi, ChartJacobian H) {
  return SOT3::Expmap(xi, H);
}

SOT3::TangentVector SOT3::ChartAtOrigin::Local(const SOT3& Q,
                                                ChartJacobian H) {
  return SOT3::Logmap(Q, H);
}

}  // namespace gtsam

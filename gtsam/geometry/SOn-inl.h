/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#pragma once

/**
 * @file    SOn-inl.h
 * @brief   Template implementations for SO(n)
 * @author  Frank Dellaert
 * @date    March 2019
 */

#include <gtsam/base/Matrix.h>

namespace gtsam {

// Implementation for N>5 just uses dynamic version
template <int N>
typename SO<N>::MatrixNN SO<N>::Hat(const TangentVector& xi) {
  return SOn::Hat(xi);
}

// Implementation for N>5 just uses dynamic version
template <int N>
typename SO<N>::TangentVector SO<N>::Vee(const MatrixNN& X) {
  return SOn::Vee(X);
}

template <int N>
SO<N> SO<N>::ChartAtOrigin::Retract(const TangentVector& xi, ChartJacobian H) {
  const Matrix X = Hat(xi / 2.0);
  size_t n = AmbientDim(xi.size());
  const auto I = Eigen::MatrixXd::Identity(n, n);
  return SO((I + X) * (I - X).inverse());
}

template <int N>
typename SO<N>::TangentVector SO<N>::ChartAtOrigin::Local(const SO& R,
                                                          ChartJacobian H) {
  const size_t n = R.rows();
  const auto I = Eigen::MatrixXd::Identity(n, n);
  const Matrix X = (I - R.matrix_) * (I + R.matrix_).inverse();
  return -2 * Vee(X);
}

template <int N>
typename SO<N>::MatrixDD SO<N>::AdjointMap() const {
  throw std::runtime_error(
      "SO<N>::AdjointMap only implemented for SO3 and SO4.");
}

template <int N>
SO<N> SO<N>::Expmap(const TangentVector& omega, ChartJacobian H) {
  throw std::runtime_error("SO<N>::Expmap only implemented for SO3 and SO4.");
}

template <int N>
typename SO<N>::MatrixDD SO<N>::ExpmapDerivative(const TangentVector& omega) {
  throw std::runtime_error("SO<N>::ExpmapDerivative only implemented for SO3.");
}

template <int N>
typename SO<N>::TangentVector SO<N>::Logmap(const SO& R, ChartJacobian H) {
  throw std::runtime_error("SO<N>::Logmap only implemented for SO3.");
}

template <int N>
typename SO<N>::MatrixDD SO<N>::LogmapDerivative(const TangentVector& omega) {
  throw std::runtime_error("O<N>::LogmapDerivative only implemented for SO3.");
}

template <int N>
typename SO<N>::VectorN2 SO<N>::vec(
    OptionalJacobian<internal::NSquaredSO(N), dimension> H) const {
  const size_t n = rows();
  const size_t n2 = n * n;

  // Vectorize
  VectorN2 X(n2);
  X << Eigen::Map<const Matrix>(matrix_.data(), n2, 1);

  // If requested, calculate H as (I \oplus Q) * P
  if (H) {
    // Calculate P matrix of vectorized generators
    const size_t d = dim();
    Matrix P(n2, d);
    for (size_t j = 0; j < d; j++) {
      const auto X = Hat(Eigen::VectorXd::Unit(d, j));
      P.col(j) = Eigen::Map<const Matrix>(X.data(), n2, 1);
    }
    H->resize(n2, d);
    for (size_t i = 0; i < n; i++) {
      H->block(i * n, 0, n, d) = matrix_ * P.block(i * n, 0, n, d);
    }
  }
  return X;
}

}  // namespace gtsam

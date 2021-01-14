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

#include <iostream>

namespace gtsam {

// Implementation for N>=5 just uses dynamic version
template <int N>
typename SO<N>::MatrixNN SO<N>::Hat(const TangentVector& xi) {
  return SOn::Hat(xi);
}

// Implementation for N>=5 just uses dynamic version
template <int N>
typename SO<N>::TangentVector SO<N>::Vee(const MatrixNN& X) {
  return SOn::Vee(X);
}

template <int N>
SO<N> SO<N>::ChartAtOrigin::Retract(const TangentVector& xi, ChartJacobian H) {
  if (H) throw std::runtime_error("SO<N>::Retract jacobian not implemented.");
  const Matrix X = Hat(xi / 2.0);
  size_t n = AmbientDim(xi.size());
  const auto I = Eigen::MatrixXd::Identity(n, n);
  // https://pdfs.semanticscholar.org/6165/0347b2ccac34b5f423081d1ce4dbc4d09475.pdf
  return SO((I + X) * (I - X).inverse());
}

template <int N>
typename SO<N>::TangentVector SO<N>::ChartAtOrigin::Local(const SO& R,
                                                          ChartJacobian H) {
  if (H) throw std::runtime_error("SO<N>::Local jacobian not implemented.");
  const size_t n = R.rows();
  const auto I = Eigen::MatrixXd::Identity(n, n);
  const Matrix X = (I - R.matrix_) * (I + R.matrix_).inverse();
  return -2 * Vee(X);
}

template <int N>
typename SO<N>::MatrixDD SO<N>::AdjointMap() const {
  if (N==2) return I_1x1; // SO(2) case
  throw std::runtime_error(
      "SO<N>::AdjointMap only implemented for SO2, SO3 and SO4.");
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

// Default fixed size version (but specialized elsewehere for N=2,3,4)
template <int N>
typename SO<N>::VectorN2 SO<N>::vec(
    OptionalJacobian<internal::NSquaredSO(N), dimension> H) const {
  // Vectorize
  VectorN2 X = Eigen::Map<const VectorN2>(matrix_.data());

  // If requested, calculate H as (I \oplus Q) * P,
  // where Q is the N*N rotation matrix, and P is calculated below.
  if (H) {
    // Calculate P matrix of vectorized generators
    // TODO(duy): Should we refactor this as the jacobian of Hat?
    Matrix P = SO<N>::VectorizedGenerators();
    for (size_t i = 0; i < N; i++) {
      H->block(i * N, 0, N, dimension) =
          matrix_ * P.block(i * N, 0, N, dimension);
    }
  }
  return X;
}

template <int N>
void SO<N>::print(const std::string& s) const {
    std::cout << s << matrix_ << std::endl;
}

}  // namespace gtsam

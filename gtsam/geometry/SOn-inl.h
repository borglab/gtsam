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

template <int N>
Matrix SO<N>::Hat(const Vector& xi) {
  size_t n = AmbientDim(xi.size());
  if (n < 2) throw std::invalid_argument("SO<N>::Hat: n<2 not supported");

  Matrix X(n, n);  // allocate space for n*n skew-symmetric matrix
  X.setZero();
  if (n == 2) {
    // Handle SO(2) case as recursion bottom
    assert(xi.size() == 1);
    X << 0, -xi(0), xi(0), 0;
  } else {
    // Recursively call SO(n-1) call for top-left block
    const size_t dmin = (n - 1) * (n - 2) / 2;
    X.topLeftCorner(n - 1, n - 1) = Hat(xi.tail(dmin));

    // Now fill last row and column
    double sign = 1.0;
    for (size_t i = 0; i < n - 1; i++) {
      const size_t j = n - 2 - i;
      X(n - 1, j) = sign * xi(i);
      X(j, n - 1) = -X(n - 1, j);
      sign = -sign;
    }
  }
  return X;
}

template <int N>
Vector SO<N>::Vee(const Matrix& X) {
  const size_t n = X.rows();
  if (n < 2) throw std::invalid_argument("SO<N>::Hat: n<2 not supported");

  if (n == 2) {
    // Handle SO(2) case as recursion bottom
    Vector xi(1);
    xi(0) = X(1, 0);
    return xi;
  } else {
    // Calculate dimension and allocate space
    const size_t d = n * (n - 1) / 2;
    Vector xi(d);

    // Fill first n-1 spots from last row of X
    double sign = 1.0;
    for (size_t i = 0; i < n - 1; i++) {
      const size_t j = n - 2 - i;
      xi(i) = sign * X(n - 1, j);
      sign = -sign;
    }

    // Recursively call Vee to fill remainder of x
    const size_t dmin = (n - 1) * (n - 2) / 2;
    xi.tail(dmin) = Vee(X.topLeftCorner(n - 1, n - 1));
    return xi;
  }
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
